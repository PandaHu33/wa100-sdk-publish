import cv2
import mediapipe as mp
import numpy as np
import time
import socket
import struct
import math
import os

# ========= 配置 =========
UDP_IP = "127.0.0.1"
UDP_PORT = 8888
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_landmarker.task")

# 机械手行程
POS_OPEN_VAL = 2000
POS_CLOSE_VAL = 0

# 安全参数
LOSS_TIMEOUT = 0.5       # 超过0.5秒未检测到手，视为信号丢失
MAX_STEP_PER_FRAME = 200 # 每帧最大允许变化的 PWM 数值 (防瞬移)

# ========= 滤波器与工具 =========
class OneEuroFilter:
    def __init__(self, t0, x0, dx0=0.0, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.x_prev = float(x0)
        self.dx_prev = float(dx0)
        self.t_prev = float(t0)

    def smoothing_factor(self, t_e, cutoff):
        r = 2 * math.pi * cutoff * t_e
        return r / (r + 1)

    def exponential_smoothing(self, a, x, x_prev):
        return a * x + (1 - a) * x_prev

    def __call__(self, t, x, dynamic_beta=None):
        t_e = t - self.t_prev
        if t_e <= 0.0: return self.x_prev
        a_d = self.smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e
        dx_hat = self.exponential_smoothing(a_d, dx, self.dx_prev)
        cutoff = self.min_cutoff + (dynamic_beta if dynamic_beta else self.beta) * abs(dx_hat)
        a = self.smoothing_factor(t_e, cutoff)
        x_hat = self.exponential_smoothing(a, x, self.x_prev)
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat

    def reset(self, t, x):
        """强制重置滤波器状态 (用于解决重捕获后的跳变)"""
        self.x_prev = float(x)
        self.dx_prev = 0.0
        self.t_prev = float(t)
        return x

class HandController:
    def __init__(self):
        self.last_seen_time = 0
        self.is_lost = True
        
        # 上一次发送给硬件的目标值 (用于限速)
        self.last_hw_targets = [POS_OPEN_VAL] * 6
        
        # 滤波器组
        t0 = time.time()
        # beta 越小越平滑但滞后越大。0.05 比较平衡
        self.filters = [OneEuroFilter(t0, 0, min_cutoff=0.5, beta=0.05) for _ in range(5)]
        
        # [修改] 移除大拇指滤波器，改为突变剔除的状态变量
        self.last_thumb_raw_yaw = None
        self.last_thumb_raw_pitch = None
        
        # 记录每根手指的滤波器是否需要重置
        self.filter_inited = False

    def _calculate_angle(self, v1, v2):
        """计算两个向量的夹角 (角度制)"""
        # 归一化
        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)
        if norm1 == 0 or norm2 == 0: return 0
        
        u1 = v1 / norm1
        u2 = v2 / norm2
        
        # 点积限制在 [-1, 1] 避免数值误差
        dot = np.clip(np.dot(u1, u2), -1.0, 1.0)
        return np.degrees(np.arccos(dot))

    def _process_thumb(self, landmarks):
        """
        【中指基准稳定版】
        1. 解决干扰：使用中指(Point 9)建立坐标系，食指怎么动都不会影响拇指 Yaw。
        2. 修正区间：拇指贴近食指时，离中指仍有距离，Yaw 保持张开；只有指向掌心才算闭合。
        """
        pts = np.array([[lm.x, lm.y, lm.z] for lm in landmarks])
        
        # === 1. 定义关键点 (Key Change) ===
        p_wrist = pts[0]
        # 【关键修改】原点参考改为中指根部 (Point 9)，它是全手最稳的点
        p_middle_mcp = pts[9]  
        p_pinky_mcp = pts[17]
        p_thumb_tip = pts[4]
        
        # 手掌尺度
        palm_scale = np.linalg.norm(p_middle_mcp - p_wrist)
        
        # ==============================================================
        # 2. 构建基于中指的稳定坐标系
        # ==============================================================
        # Y轴: 手腕 -> 中指根 (手掌的绝对中轴线)
        v_y = p_middle_mcp - p_wrist
        v_y /= np.linalg.norm(v_y)
        
        # 辅助向量 (用于算Z轴)
        v_aux = p_pinky_mcp - p_wrist
        
        # Z轴: 手掌法线
        v_z = np.cross(v_y, v_aux)
        v_z /= np.linalg.norm(v_z)
        
        # X轴: 侧向 (垂直于中指和法线)
        # 这个轴现在指向拇指一侧
        v_x = np.cross(v_y, v_z)
        v_x /= np.linalg.norm(v_x)
        
        # 拇指向量
        v_thumb = p_thumb_tip - p_wrist
        
        # ==============================================================
        # 3. 计算 Yaw (横摇) - 距离中指的横向宽度
        # ==============================================================
        # 计算拇指尖在 X 轴上的投影
        # 物理意义：拇指尖距离"中指中轴线"的横向距离
        yaw_proj = np.dot(v_thumb, v_x)
        norm_yaw = np.abs(yaw_proj) / palm_scale
        
        # 【逻辑修正】
        # L型张开时：拇指离中指很远 (Norm ≈ 1.5)
        # 贴近食指时：拇指离中指较远 (Norm ≈ 0.8) -> 仍算张开区域
        # 指向掌心时：拇指在中指正上方 (Norm ≈ 0.0) -> 闭合区域
        
        # 阈值调整 (根据中指基准微调)
        YAW_OPEN_THRESH  = 0.8  # 大于0.8都算完全张开 (2000)
        YAW_CLOSE_THRESH = 0.1  # 小于0.1才算完全对掌 (0)
        
        yaw_ratio = (norm_yaw - YAW_CLOSE_THRESH) / (YAW_OPEN_THRESH - YAW_CLOSE_THRESH)
        yaw_ratio = np.clip(yaw_ratio, 0.0, 1.0)
        
        # 硬件映射 (2000=Open, 0=Close)
        yaw_hw = int(0 + yaw_ratio * (2000 - 0))

        # ==============================================================
        # 4. 计算 Pitch (弯曲) - 动态阈值 (保持原有逻辑)
        # ==============================================================
        raw_height = np.abs(np.dot(v_thumb, v_z))
        norm_height = raw_height / palm_scale
        
        is_palm_view = v_z[2] > 0 # 需根据实际Z轴方向确认
        
        # 这里的阈值可以保持，或者根据手型微调
        if is_palm_view: 
            h_straight_close = 0.55
            h_straight_open  = 0.30  
            h_bent = 0.20
        else:
            h_straight_close = 0.50
            h_straight_open  = 0.15 
            h_bent = 0.01

        current_straight_thresh = h_straight_close + yaw_ratio * (h_straight_open - h_straight_close)
        
        pitch_ratio = (norm_height - h_bent) / (current_straight_thresh - h_bent)
        pitch_ratio = np.clip(pitch_ratio, 0.0, 1.0)
        
        pitch_hw = int(pitch_ratio * (2000 - 0) + 0)
        
        # Debug
        # print(f"YawDist:{norm_yaw:.2f} | PitchH:{norm_height:.2f} -> Y:{yaw_hw} P:{pitch_hw}")

        return yaw_hw, pitch_hw

    def process_landmarks(self, landmarks):
        """处理一帧数据，返回 (是否有效, [6个目标值])"""
        current_time = time.time()
        
        # 1. 计算原始弯曲度 (0.0=直, 1.0=弯)
        raw_bends = self._calc_bends(landmarks)
        
        # 2. 状态检查：是从"丢失"状态刚刚恢复吗？
        if self.is_lost:
            print(">>> [Hand Re-acquired] 手部重新捕获，平滑重置...")
            self.is_lost = False
            self.filter_inited = False # 标记需要重置滤波器
            self.last_thumb_raw_yaw = None   # 重置突变检测
            self.last_thumb_raw_pitch = None
            
        # 3. 滤波
        filtered_bends = []
        for i, bend in enumerate(raw_bends):
            if not self.filter_inited:
                # 如果是第一帧，强制重置滤波器到当前值，避免从0开始的爬升
                val = self.filters[i].reset(current_time, bend)
            else:
                val = self.filters[i](current_time, bend)
            filtered_bends.append(val)
        
        self.filter_inited = True
        self.last_seen_time = current_time
        
        # 4. 映射到硬件值 (根据用户提供的确切映射)
        # filtered_bends: [0:Index, 1:Middle, 2:Ring, 3:Pinky, 4:Thumb]
        targets = []
        
        # [NEW LOGIC] 大拇指使用专门的解耦算法
        # 计算出的原始值 (Raw)
        raw_yaw, raw_pitch = self._process_thumb(landmarks)
        
        # [防抖参数] 针对 Pitch/Yaw 的正负100抖动，设置一个“静止死区”
        # 变化量 < 150: 认为是抖动，进行强力滤波 (90%保持 + 10%更新)
        # 变化量 >= 150: 认为是运动，直接响应
        STABLE_THRESH = 150 

        if self.last_thumb_raw_yaw is None:
            self.last_thumb_raw_yaw = raw_yaw
            self.last_thumb_raw_pitch = raw_pitch
        else:
            # 1. Yaw 轴: 动态死区 (胶水防抖)
            diff_yaw = abs(raw_yaw - self.last_thumb_raw_yaw)
            if diff_yaw < STABLE_THRESH:
                # [静止区] 像胶水一样粘住，消除抖动
                self.last_thumb_raw_yaw = 0.9 * self.last_thumb_raw_yaw + 0.1 * raw_yaw
            else:
                # [运动区] 快速响应
                self.last_thumb_raw_yaw = raw_yaw
            
            # 2. Pitch 轴: 动态死区 (胶水防抖)
            diff_pitch = abs(raw_pitch - self.last_thumb_raw_pitch)
            if diff_pitch < STABLE_THRESH:
                # [静止区] 像胶水一样粘住，消除抖动
                self.last_thumb_raw_pitch = 0.9 * self.last_thumb_raw_pitch + 0.1 * raw_pitch
            else:
                # [运动区] 快速响应
                self.last_thumb_raw_pitch = raw_pitch

        # ID 1 (Pos[0]): 拇指横摇 (Yaw)
        targets.append(int(self.last_thumb_raw_yaw))

        # ID 2 (Pos[1]): 拇指弯曲 (Pitch)
        targets.append(int(self.last_thumb_raw_pitch))
        
        # ID 3 (Pos[2]): 食指 -> bends[0]
        targets.append(int(filtered_bends[0] * (POS_CLOSE_VAL - POS_OPEN_VAL) + POS_OPEN_VAL))
        
        # ID 4 (Pos[3]): 中指 -> bends[1]
        targets.append(int(filtered_bends[1] * (POS_CLOSE_VAL - POS_OPEN_VAL) + POS_OPEN_VAL))
        
        # ID 5 (Pos[4]): 无名指 -> bends[2]
        targets.append(int(filtered_bends[2] * (POS_CLOSE_VAL - POS_OPEN_VAL) + POS_OPEN_VAL))
        
        # ID 6 (Pos[5]): 小指 -> bends[3]
        targets.append(int(filtered_bends[3] * (POS_CLOSE_VAL - POS_OPEN_VAL) + POS_OPEN_VAL))
        
        # === 5. 包络抓取增强 (Grasp Assist) ===
        
        index_bent = targets[2] < 1000  # 食指弯了一半以上 (2000=Open, 0=Close)
        thumb_bent = targets[1] < 1000  # 拇指弯了一半以上
        
        if index_bent and thumb_bent:
            # 进入"强力抓取模式"
            IDEAL_OPPOSITION_YAW = 0 
            
            # 使用简单的互补滤波平滑过渡过去，而不是生硬跳变
            # targets[0] 是当前的 Yaw 值
            targets[0] = int(0.7 * targets[0] + 0.3 * IDEAL_OPPOSITION_YAW)
        
        # 6. [核心安全机制] 变化率限制 (Slew Rate Limiting)
        safe_targets = []
        for i, target in enumerate(targets):
            # 限制数值范围
            target = max(0, min(4095, target))
            
            # 限制变化速度：不能超过上一帧 +- MAX_STEP
            diff = target - self.last_hw_targets[i]
            if abs(diff) > MAX_STEP_PER_FRAME:
                # 截断变化量
                target = self.last_hw_targets[i] + int(math.copysign(MAX_STEP_PER_FRAME, diff))
            
            safe_targets.append(target)
            
        self.last_hw_targets = safe_targets
        return True, safe_targets

    def check_loss(self):
        """检查是否丢失信号"""
        if not self.is_lost and (time.time() - self.last_seen_time > LOSS_TIMEOUT):
            self.is_lost = True
            print(">>> [Signal Lost] 信号丢失，停止发送")
        return self.is_lost

    def _calc_bends(self, landmarks):
        pts = np.array([[lm.x, lm.y, lm.z] for lm in landmarks])
        wrist = pts[0]
        fingers_indices = [(8, 5), (12, 9), (16, 13), (20, 17), (4, 2)]
        bends = []
        for i, (tip_idx, mcp_idx) in enumerate(fingers_indices):
            len_tip = np.linalg.norm(pts[tip_idx] - wrist)
            len_mcp = np.linalg.norm(pts[mcp_idx] - wrist)
            ratio = len_tip / len_mcp
            if i == 4: # Thumb
                open_thresh, close_thresh = 1.6, 1.0
            else:
                open_thresh, close_thresh = 2.2, 0.9
            bend = np.interp(ratio, [close_thresh, open_thresh], [1.0, 0.0])
            bends.append(bend)
        return bends

def main():
    if not os.path.exists(MODEL_PATH):
        print(f"错误: 找不到 {MODEL_PATH}")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # MediaPipe 初始化
    BaseOptions = mp.tasks.BaseOptions
    HandLandmarker = mp.tasks.vision.HandLandmarker
    HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
    VisionRunningMode = mp.tasks.vision.RunningMode

    # 读取模型文件到内存，避开 Windows 中文路径问题
    with open(MODEL_PATH, 'rb') as f:
        model_buffer = f.read()

    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_buffer=model_buffer),
        running_mode=VisionRunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.6, # 调高置信度，减少误识别
        min_hand_presence_confidence=0.6,
        min_tracking_confidence=0.6,
    )

    # 尝试寻找可用摄像头
    cap = None
    for index in range(2): # 尝试 0 和 1
        print(f">>> 尝试打开摄像头 ID {index}...")
        temp_cap = cv2.VideoCapture(index, cv2.CAP_DSHOW) # Windows下推荐使用 CAP_DSHOW 增加稳定性
        if temp_cap.isOpened():
            temp_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            temp_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            temp_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            temp_cap.set(cv2.CAP_PROP_FPS, 30)
            print(f">>> 成功打开摄像头 ID {index}")
            cap = temp_cap
            break
        else:
             temp_cap.release()

    if cap is None or not cap.isOpened():
        print(">>> 错误: 无法打开任何摄像头 (ID 0 或 1)")
        print(">>> 请检查: 1. 摄像头是否连接 2. 隐私设置是否允许应用访问摄像头 3. 是否被其他程序占用")
        return

    controller = HandController()
    start_time_ms = int(time.time() * 1000)
    
    print(">>> 正在启动视频流处理...")
    
    try:
        with HandLandmarker.create_from_options(options) as landmarker:
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret: 
                    print(">>> 警告: 无法获取帧 (End of stream?)")
                    break

                frame = cv2.flip(frame, 1)
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
                current_timestamp_ms = int(time.time() * 1000) - start_time_ms
                
                # 推理
                detection_result = landmarker.detect_for_video(mp_image, current_timestamp_ms)
    
                valid_control = False
                hw_targets = []
    
                if detection_result.hand_landmarks:
                    # 识别到了手 -> 计算控制量
                    # 解包所有的返回值
                    valid_control, hw_targets = controller.process_landmarks(detection_result.hand_landmarks[0])
                    
                    # 画图反馈
                    h, w, _ = frame.shape
                    for lm in detection_result.hand_landmarks[0]:
                        cv2.circle(frame, (int(lm.x * w), int(lm.y * h)), 4, (0, 255, 0), -1)
                
                else:
                    # 没识别到 -> 检查超时
                    if controller.check_loss():
                        # 丢失状态：显示红色警告
                        cv2.putText(frame, "SIGNAL LOST - HOLDING", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        valid_control = False # 重要：丢失时不产生新指令，让C++保持上一帧
                    else:
                        # 虽然这帧没识别到，但还没超时（可能是眨眼般的闪烁）
                        # 选择不更新，维持上一帧，或者进行预测（这里选择维持）
                        pass
    
                # 发送逻辑
                if valid_control and hw_targets:
                    packet = struct.pack('<6H', *hw_targets)
                    sock.sendto(packet, (UDP_IP, UDP_PORT))
                    cv2.putText(frame, f"Send: {hw_targets}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
                cv2.imshow('Safe Hand Control', frame)
                if cv2.waitKey(5) & 0xFF == 27:
                    break

    except Exception as e:
        print(f">>> 运行时错误: {e}") 
    finally:
        sock.close()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
