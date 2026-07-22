# Hi5 手套拇指 Rotation 标定

本流程只改变手套模式的拇指横摇和弯曲通道。食指到小指、VR hand tracking 和预设姿态不使用该标定。

## 运行标定

1. 停止 `udp_receiver_unity.exe`，保持 Hi5 手套连接正常。
2. 启动 Unity 程序 `right1/My project.exe`，确认它向 `127.0.0.1:25001` 发送数据。
3. 在仓库根目录运行：

```powershell
python examples/thumb_calibration.py --hand right
```

按提示完成中立姿态、腕部三轴旋转、四指干扰、拇指横摇、拇指弯曲和验证动作。标定约需 80 秒有效采集时间。

通过质量门限后，配置写入：

```text
build/bin/thumb_glove_calibration.json
```

若腕部串扰超过 100、横摇或弯曲有效跨度不足 1600，脚本只写诊断文件，不覆盖运行时配置。

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
