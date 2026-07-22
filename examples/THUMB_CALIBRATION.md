# Hi5 手套拇指 Rotation 标定

本流程只改变手套模式的拇指横摇和弯曲通道。当前基线按“拇指仅一个物理姿态传感器、标定和运行时手腕固定”设计，以中立姿态为参考，对 Hi5 数据包暴露的三个派生拇指骨骼 rotation 做可观测性与解耦标定。食指到小指、VR hand tracking 和预设姿态不使用该标定。

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

标定使用连续动作作为反例学习两个判别方向：横摇方向压低弯曲样本方差，弯曲方向压低横摇样本方差。若固定腕姿下中立复测漂移超过 100、任一通道串扰超过 100、横摇或弯曲有效跨度不足 1600，脚本只写诊断文件，不覆盖运行时配置。本基线明确不消除腕部运动；后续接入 VR 手柄后再使用手柄腕姿构造腕部参考系，并与本版对比。

已有原始样本可直接重新拟合，无需再次采集：

```powershell
python examples/thumb_calibration.py --hand right --from-samples build/bin/thumb_glove_calibration_samples.npz
```

## 启动和回放

正常启动参数保持不变：

```powershell
build/bin/udp_receiver_unity.exe right
```

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
