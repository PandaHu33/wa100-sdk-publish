# 手柄位姿与手套传感数据时空标定

该标定只读取数据，不发送机械臂或灵巧手命令。正常运行时：

- `udp_receiver_unity.exe` 将已接收的手套原始 JSON 只读镜像到 UDP `25005`；
- `Hand_Tracker/hand_arm_control.py` 将 PICO 右手柄原始位姿只读发布到 UDP `25006`；
- `glove_controller_alignment.py` 同时监听两个端口，使用同一台 PC 的接收时钟完成对齐。

Hi5 骨骼槽位固定按官方枚举解释：`0=ForeArm`、`1=Hand`、`2–20=局部手指骨骼`。腕/手背姿态只使用 `rightRotations[1]`，槽位 0 不参与融合，也不再用掌内关节平均值充当腕姿。

标定和运行时必须把 PICO 右手柄用非磁性支架刚性固定在手套腕背/前臂，不能戴着手套手持控制器。若安装发生松动或无法保证刚性，恒定手眼外参不成立，应在 `Hand_Tracker/hand_control_config.yaml` 中切回 `mode: "m1"`。

输出内容包括：

- 手柄时间戳加到手套时间轴所需的偏移；
- 满足 `A·X=X·B` 的手柄—手套旋转手眼外参；
- 两个 Unity/追踪世界旋转坐标系之间的常量变换；
- 手柄坐标系到人体腕部旋转中心的平移；
- 本次佩戴的中立手柄姿态和手套姿态；
- 角速度互相关、配对时间差、独立验证段旋转残差、三轴激励度，以及可选的 pivot 残差。

## 启动顺序

1. 正常启动显控。显控会启动本地 `Hand_Tracker` 服务和 WA100 UDP 接收器。
2. 显控就绪后再启动 PICO 程序，确认右手柄处于 tracked 状态。
3. 不按机械臂 deadman 按键，保持机器人不运动。
4. 在 `wa100-sdk-publish` 目录运行手动分阶段窗口：

```powershell
python examples/glove_controller_alignment_gui.py
```

窗口不会自动倒计时推进。每个阶段都等操作者准备好后，手动点击“开始当前阶段”：

1. 中立阶段：手腕自然伸直，五指自然张开，保持静止 3 秒。
2. 运动阶段：前臂尽量固定、五指保持不动，围绕腕关节中心缓慢完成俯仰、左右偏转和旋前/旋后，持续 12 秒。不要平移整个前臂。

采集完成后自动计算；标定通过后写入：

```text
build/bin/glove_controller_alignment.json
```

无论是否通过，原始同步样本都会保存为：

```text
build/bin/glove_controller_alignment_samples.npz
```

若质量不通过，程序只写入 `glove_controller_alignment.failed.json`，不会覆盖上一份有效配置。常见原因包括手指在运动阶段活动、腕部只绕单一轴运动、前臂整体平移、手柄跟踪丢失或手套数据中断。

## 时间与空间约定

时间偏移定义为：

```text
aligned_controller_time = controller_receive_time + offset
```

旋转外参定义为：

```text
R_hand = R_gloveWorld_from_controllerWorld
         × R_controller
         × R_controller_to_hand
```

腕心位置定义为：

```text
p_wrist = p_controller + R_controller × p_controller_to_wristPivot
```

手套没有位置观测，因此腕心平移完全由 PICO 手柄的多姿态 pivot calibration 得到；静止若干帧只能估计中立旋转，不能独立确定腕心平移。5-DoF 增量控制默认不要求 pivot，只有需要人体腕心绝对位置时才用 `--require-pivot` 将其纳入合格门槛。

固定合格门槛为：时间相关系数不低于 `0.8`、配对时间差 P95 不高于 `60 ms`、独立验证段旋转残差中位数不高于 `4°`、P95 不高于 `10°`、多轴激励比不低于 `0.03`。需要 pivot 时，其 P95 还必须不高于 `30 mm`。

四指端点写入同一份手套 JSON：

```powershell
python examples/finger_endpoint_calibration.py
```

运行该工具前停止 `udp_receiver_unity.exe`，保持 Unity/Hi5 继续发送；工具仅在四指开闭跨度全部合格时原子覆盖配置。
