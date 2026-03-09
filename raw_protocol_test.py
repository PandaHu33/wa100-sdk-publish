import serial
import struct
import time
import math
import matplotlib.pyplot as plt
import threading

# ================= 配置区域 =================
# 请修改为实际端口，注意: 也是 serial_config.json 里的端口
PORT = "COM4"  
BAUDRATE = 1000000
TIMEOUT = 0.005    # 5ms limit for read

# 自动处理中文显示问题
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS'] 
plt.rcParams['axes.unicode_minus'] = False 

class WA100RawController:
    def __init__(self, port, baud):
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT,
            write_timeout=TIMEOUT
        )
        print(f">>> 串口 {port} 已打开，波特率 {baud}")
        # 清空缓冲区
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        
    def close(self):
        self.ser.close()

    def calculate_checksum(self, length, cmd, data):
        # Checksum = (Length + Cmd + sum(Data)) & 0xFF
        total = length + cmd + sum(data)
        return total & 0xFF

    def enable_all(self):
        # Cmd 0x04 Enable, ID=0 (All)
        # Frame: 48 4D 01 04 00 [CS]
        cmd = 0x04
        data = [0x00] # ID=0
        self.send_frame(cmd, data)
        # 接收应答
        resp = self.read_frame()
        return resp

    def send_frame(self, cmd, data_bytes):
        # Header: 48 4D
        # Length: len(data_bytes)
        length = len(data_bytes)
        checksum = self.calculate_checksum(length, cmd, data_bytes)
        
        frame = bytearray()
        frame.append(0x48)
        frame.append(0x4D)
        frame.append(length)
        frame.append(cmd)
        frame.extend(data_bytes)
        frame.append(checksum)
        
        self.ser.write(frame)
        self.ser.flush()

    def read_frame(self):
        # 改进的读取逻辑：具备自动帧头对齐功能
        
        # 1. 尝试寻找帧头 0x48 0x4D
        # 我们最多读几十个字节来找头，防止陷入死循环
        max_scan = 100
        while max_scan > 0:
            b = self.ser.read(1)
            if not b: 
                return None # 超时
            
            if b == b'\x48':
                b2 = self.ser.read(1)
                if not b2: return None
                if b2 == b'\x4D':
                    break # 找到帧头！
                # 否则继续找
            max_scan -= 1
            
        if max_scan == 0:
            return None # 没找到帧头

        # 2. Read Length
        length_b = self.ser.read(1)
        if not length_b: return None
        length = length_b[0]
        
        # 3. Read Cmd
        cmd_b = self.ser.read(1)
        if not cmd_b: return None
        cmd = cmd_b[0]
        
        # 4. Read Data
        data = self.ser.read(length)
        if len(data) != length: 
            return None
        
        # 5. Read Checksum
        cs_b = self.ser.read(1)
        
        # 可选：校验和检查（为了性能先跳过，或者加上）
        # calc_cs = self.calculate_checksum(length, cmd, data)
        # if calc_cs != cs_b[0]: return None
        
        return data

    def set_follow_positions(self, target_positions):
        """
        发送跟随指令 (0x05) 并解析返回的状态包
        """
        if len(target_positions) != 6: return None, None
        
        # 策略改进：发送前清空 Input 缓冲区，确保读到的是本次的回复（降低延迟，避免错位）
        self.ser.reset_input_buffer()

        # Cmd 0x05 Data: 6 * uint16_le
        data_bytes = bytearray()
        for pos in target_positions:
            pos = max(0, min(2000, int(pos))) # 修正范围 0-2000
            data_bytes.extend(struct.pack('<H', pos)) 
            
        self.send_frame(0x05, data_bytes)
        
        # 读取返回
        resp_data = self.read_frame()
        
        if resp_data and len(resp_data) == 37:
            current_positions = []
            current_amps = []
            offset = 0
            for i in range(6):
                p = struct.unpack('<H', resp_data[offset:offset+2])[0]
                cur = struct.unpack('<H', resp_data[offset+2:offset+4])[0]
                current_positions.append(p)
                current_amps.append(cur)
                offset += 6 
            return current_positions, current_amps
        
        return None, None

def main():
    try:
        controller = WA100RawController(PORT, BAUDRATE)
    except Exception as e:
        print(f"无法打开串口: {e}")
        print("请检查 USB 连线或端口号 COM4 是否正确。")
        return

    # 打开 CSV 文件
    import csv
    csv_filename = "raw_protocol_log.csv"
    
    try:
        print(">>> 正在使能 (Enable)...")
        controller.enable_all()
        time.sleep(1) # 等待就绪

        print(f">>> 开始 P2P 轨迹测试 (20秒)... 数据将保存至 {csv_filename}")
        print(">>> 请确保已修改 Windows 设备管理器 Latency Timer 为 1 !")
        
        # CSV 初始化
        f_csv = open(csv_filename, 'w', newline='')
        writer = csv.writer(f_csv)
        writer.writerow(["Time(ms)","LoopCount","Freq(Hz)","CmdPos","RealPos","Current(mA)"])

        log_data_list = [] # 用于最后简单画图 (Time, Cmd, Real)
        start_time = time.perf_counter()
        
        loop_cnt = 0
        freq_monitor_start = start_time
        last_freq_update_cnt = 0
        current_freq = 0.0
        
        # 运行 20 秒
        while True:
            now = time.perf_counter()
            t = now - start_time
            if t > 20.0: break
            
            # 生成 P2P 梯形轨迹 (与 basic_demo 一致: 1000-2000)
            # Cycle: 6.0s
            cycle_t = t % 6.0 
            if cycle_t < 1.0:
                 cmd_val = 1000
            elif cycle_t < 3.0: # 1000->2000 in 2.0s
                 r = (cycle_t - 1.0) / 2.0
                 cmd_val = 1000 + r * 1000
            elif cycle_t < 4.0:
                 cmd_val = 2000
            else: # 2000->1000 in 2.0s
                 r = (cycle_t - 4.0) / 2.0
                 cmd_val = 2000 - r * 1000
                
            targets = [cmd_val] * 6
            
            # 发送并接收
            real_pos_list, real_amp_list = controller.set_follow_positions(targets)
            
            # 手动限频至约 200 Hz (5ms) 或 500 Hz (2ms)
            # 避免对下位机造成过大中断压力，观察是否能改善超调
            time.sleep(0.01) # 1ms sleep -> ~1000Hz
            
            current_real = 0
            current_amp = 0
            
            if real_pos_list:
                current_real = real_pos_list[0]
                current_amp = real_amp_list[0]
            else:
                # 丢包或超时，沿用上一次的值
                if len(log_data_list) > 0:
                    current_real = log_data_list[-1][2]
                
            log_data_list.append([t, cmd_val, current_real])
            
            loop_cnt += 1
            if loop_cnt % 100 == 0:
                dt = now - freq_monitor_start
                if dt > 0:
                    current_freq = (loop_cnt - last_freq_update_cnt) / dt
                print(f"\rFreq: {current_freq:.1f} Hz | Last Pos: {current_real}", end="")
                freq_monitor_start = now
                last_freq_update_cnt = loop_cnt

            # 写入 CSV
            writer.writerow([
                int(time.time() * 1000), 
                loop_cnt, 
                f"{current_freq:.2f}", 
                int(cmd_val), 
                int(current_real), 
                int(current_amp)
            ])
            
            loop_cnt += 1
            if loop_cnt % 100 == 0:
                now = time.perf_counter()
                freq = 100 / (now - freq_monitor_start)
                print(f"\rFreq: {freq:.1f} Hz | Last Pos: {current_real}", end="")
                freq_monitor_start = now
                
            # 无需 sleep，全速运行

        print("\n测试结束")
        f_csv.close()
        print(f"数据已保存至 {csv_filename}")
        
    finally:
        controller.close()

if __name__ == "__main__":
    main()
