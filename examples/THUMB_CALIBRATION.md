# Hi5 手套拇指 Rotation 标定

本流程只改变手套模式的拇指横摇和弯曲通道。当前基线按“拇指仅一个物理姿态传感器、标定和运行时手腕固定”设计。横摇以中立姿态为参考，继续使用 Hi5 三个派生拇指骨骼 rotation 的四元数判别方向；弯曲不再使用同一个线性解耦模型，而是与四指采用相同的 rotation 模长、端点死区和 smoothstep 流程。由于拇指只有一个物理传感器，不把 MCP/IP 两个派生槽位当作独立传感器加权，弯曲特征只取 `thumb_mcp_derived` 的 rotation 模长。食指到小指、VR hand tracking 和预设姿态不使用该标定。

运行时六通道严格采用硬件 ID 顺序：`ID1=拇指弯曲（pitch）`、`ID2=拇指横摇/对掌（yaw）`、`ID3~ID6=食指~小指`。标定器内部仍分别计算语义化的 `pitch_target` 和 `yaw_target`，由运行时统一按此硬件顺序打包。

## 运行标定

1. 停止 `udp_receiver_unity.exe`，保持 Hi5 手套连接正常。
2. 启动 Unity 程序 `right1/My project.exe`，确认它向 `127.0.0.1:25001` 发送数据。
3. 在仓库根目录运行图形化在线标定：

```powershell
python examples/thumb_calibration_gui.py --hand right
```

在窗口中逐阶段点击“开始当前阶段”，固定手腕完成中立姿态、拇指横摇、拇指弯曲、组合动作和中立复测。无图形界面时仍可运行 `thumb_calibration.py --hand right` 使用命令行流程。

通过质量门限后，配置写入：

```text
build/bin/thumb_glove_calibration.json
```

标定仍学习横摇判别方向；弯曲的开/闭边界直接取弯曲端点样本的单传感器 rotation 模长。由于单个物理传感器无法同时唯一分解横摇和弯曲，弯曲通道不再把“横摇动作造成的弯曲特征变化”作为拒绝标定条件。本基线明确不消除腕部运动；后续接入 VR 手柄后再使用手柄腕姿构造腕部参考系，并与本版对比。

已有原始样本可直接重新拟合，无需再次采集：

```powershell
python examples/thumb_calibration.py --hand right --from-samples build/bin/thumb_glove_calibration_samples.npz
```

## 启动和回放

正常启动参数保持不变：

```powershell
build/bin/udp_receiver_unity.exe right
```

默认使用静态配置模式：启动时只读取一次可执行文件同目录的 `thumb_glove_calibration.json`，运行期间不会启动标定窗口，也不会热加载文件变化。

只有需要在线调参时，才显式开启热加载并手动启动边界调节窗口：

```powershell
build/bin/udp_receiver_unity.exe right --thumb-hot-reload
python examples/thumb_boundary_tuner.py
```

窗口可分别修改 `yaw.open/close`、`pitch.open/close`、端点死区和 `open_target/close_target`。其中 `yaw` 仍是横摇判别分数，`pitch` 已改为单传感器 rotation 模长（度），两者不是同一单位。点击应用后配置采用原子写入，接收器约 0.5 秒内热加载；无效或写入中的配置不会替换上一份有效参数。

也可以显式指定配置：

```powershell
build/bin/udp_receiver_unity.exe right --thumb-calibration path/to/thumb_glove_calibration.json
```

对监控记录进行旧、新映射比较：

```powershell
python examples/thumb_mapping_replay.py unity_input.jsonl build/bin/thumb_glove_calibration.json `
  --actual-csv wa100_target_command_actual.csv `
  --output thumb_mapping_replay.csv `
  --plot thumb_mapping_replay.png
```

运行时配置缺失或无效时，两个拇指通道保持安全张开值 2000，食指到小指继续工作。测试目录中的 `synthetic_thumb_calibration.json` 只能用于自动测试，禁止部署到真实手套。
