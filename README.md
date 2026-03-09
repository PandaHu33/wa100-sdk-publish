# WA100 水下机械手 SDK 使用说明

本仓库提供 WA100 系列水下机械手（6 个关节，`ALLID=6`）的 SDK 头文件 + 动态库/静态导入库 + 示例工程。
SDK 使用 C++ 类接口（`WA100DeviceManager` / `WA100Device`）完成多设备管理、运动控制、状态读取以及异常回调处理。

> 说明：本仓库不包含 SDK 源码，仅提供已编译的库文件（Windows: `.dll`/`.lib` 等）。

## 1. 目录结构

```text
WA100-SDK-Publish/
├── include/                     # SDK 头文件（公开 API）
│   ├── wa100_device.h
│   ├── wa100_device_manager.h
│   └── wa100_types.h
├── lib/                         # 预编译库
│   ├── win-msvc/                # Windows (MSVC) 版本：wa100_sdk.dll + wa100_sdk.lib
│   ├── win-wingw32/             # Windows (MinGW) 版本（如有）
│   ├── linux-x86_64/            # Linux x86_64 版本（如有）
│   └── linux-arm_64/            # Linux arm64 版本（如有）
├── examples/                    # 示例工程（可直接编译运行）
│   ├── basic_demo.cpp
│   ├── advanced_demo.cpp
│   ├── serial_config.json
│   └── quickstart_demo.cpp
└── README.md
```

## 2. 环境与集成

### 2.1 编译器/工具

- C++ 标准：C++14 或以上
- 构建工具：CMake 3.10+

### 2.2 Windows（MSVC）集成要点

- 头文件：把 `include/` 加到工程 include path。
- 链接库：链接 `lib/win-msvc/wa100_sdk.lib`。
- 运行时：确保 `lib/win-msvc/wa100_sdk.dll` 在可执行文件同目录，或在 `PATH` 可搜索路径中。

示例 CMake（仅示意，按你的工程路径调整）：

```cmake
target_include_directories(your_app PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_directories(your_app PRIVATE ${CMAKE_SOURCE_DIR}/lib/win-msvc)
target_link_libraries(your_app PRIVATE wa100_sdk)
```

> 注意：作为 SDK 使用方，不要定义 `WA100_SDK_EXPORTS`；该宏仅用于构建 SDK 时导出符号。

### 2.3 示例工程编译与运行

```powershell
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

运行前准备：

- 默认情况下，构建脚本会自动把 `examples/serial_config.json` 和 `lib/<platform>/wa100_sdk.dll` 复制到输出目录 `build/bin/`。
- 如需手动运行/拷贝：确保 `serial_config.json` 与 `.exe` 同目录，且 Windows 下 `.dll` 与 `.exe` 同目录。

## 3. 串口配置文件（serial_config.json）

SDK 通过 `WA100Device::GetConfig()` / `WA100DeviceManager::AddDevice()` 读取串口配置文件。

文件字段说明（JSON Schema 语义）：

- 顶层字段：
    - `serial_port`: `array<object>`，数组中的每一项代表一套串口参数

- 每个 `serial_port[i]` 对象字段：
    - `port`: `string`，Windows 下如 `"COM3"`；Linux 下通常如 `"/dev/ttyUSB0"`
    - `baudrate`: `number`，波特率（示例为 `1000000`；也可按设备实际支持配置）
    - `databyte`: `number`，数据位（常用 8）
    - `parity`: `number`，校验位：
        - `0` none
        - `1` odd
        - `2` even
        - `3` mark
        - `4` space
    - `stopbyte`: `number`，停止位（常用 1）
    - `flowcontrol`: `number`，流控：
        - `0` none
        - `1` software
        - `2` hardware
    - `timeout_ms`: `number`，超时时间（毫秒，常用 100）
- 注：如果json格式不合理或者缺少部分信息等异常情况下，默认设置的串口信息是port是COM1，baudrate是1000000，databyte是8，parity是0，stopbyte是1，flowcontrol是0，timeout_ms是100。

示例（建议不要在真实 JSON 中写注释，下面仅用于说明）：

```json
{
    "serial_port": [
        {
            "port": "COM6",
            "baudrate": 1000000,
            "databyte": 8,
            "parity": 0,
            "stopbyte": 1,
            "flowcontrol": 0,
            "timeout_ms": 100
        }
    ]
}
```

## 4. 快速上手（推荐流程）

推荐顺序（多设备/单设备一致）：

1. `WA100DeviceManager::Instance()` 获取管理器
2. `AddDevice()` 添加一个或多个设备（指定配置文件与配置索引）
3. `GetDevice()` 获取设备指针
4. （可选）注册各类异常回调
5. `StartAll()` 启动通信线程
6. `SetFollowPositions()` / `SetPositions()` 等控制指令
7. `GetStatus()` / `GetPositions()` / `GetTemperatures()` 等读状态
8. 退出前 `StopAll()` + `RemoveDevice()` / `Clear()`

## 5. API 参考（接口/参数类型/取值范围）

本节接口信息来自公开头文件：

- `include/wa100_device_manager.h`
- `include/wa100_device.h`
- `include/wa100_types.h`

### 5.1 设备数量与 ID 约束

- 关节数量：`ALLID = 6`
- 关节 ID 取值：
    - `1..6` 表示单个关节
    - `0` 表示“全部关节”（适用于 `Enable()` / `EmergencyStop()` 等）

### 5.2 WA100DeviceManager（设备管理器）

头文件：`include/wa100_device_manager.h`

#### `static WA100DeviceManager& Instance()`

- 功能：获取管理器单例
- 返回：管理器引用

#### `bool AddDevice(const std::string& name, const std::string filename, const uint8_t flag)`

- 功能：添加一个设备实例并完成初始化配置读取
- 参数：
    - `name`: `std::string`，设备名称（唯一键，用于后续查找）
    - `filename`: `std::string`，串口配置文件路径（建议传绝对路径；相对路径以进程工作目录为基准）
    - `flag`: `uint8_t`，配置索引（对应 `serial_port` 数组的第几项）
        - 约束：示例工程使用 `1` 表示第一项、`2` 表示第二项（即 1-based）
- 返回：
    - `true` 成功
    - `false` 失败（名称重复、配置读取失败、设备初始化失败等）

#### `WA100Device* GetDevice(const std::string& name)`

- 功能：按名称获取设备指针
- 返回：
    - 找到：非空指针
    - 未找到：`nullptr`

#### `bool RemoveDevice(const std::string& name)`

- 功能：移除设备并释放资源
- 返回：`true` 成功，`false` 设备不存在

#### `bool HasDevice(const std::string& name) const`

- 功能：判断设备是否存在
- 返回：存在为 `true`

#### `bool StartAll()` / `void StopAll()`

- 功能：批量启动/停止通信线程
- 返回：
    - `StartAll()`：全部成功为 `true`；至少一个失败为 `false`

#### `size_t GetDeviceCount() const` / `std::vector<std::string> GetDeviceNames() const` / `void Clear()`

- 功能：查询设备数量/名称列表/清空所有设备

### 5.3 WA100Device（设备控制）

头文件：`include/wa100_device.h`

#### 构造与生命周期

##### `explicit WA100Device(const std::string& device_name)`

- 参数：
    - `device_name`: `std::string`，用于标识设备（日志/管理用途）

##### `bool GetConfig(const std::string& filename, uint8_t flag)`

- 功能：读取 `serial_config.json` 中的串口参数
- 参数：
    - `filename`: `std::string`，配置文件路径
    - `flag`: `uint8_t`，配置索引（与 `AddDevice` 同义）
- 返回：成功 `true`

##### `bool Init()` / `bool Start()` / `void Stop()` / `bool IsRunning() const`

- `Init()`：初始化设备（通常由管理器内部调用）
- `Start()`：启动设备（开启通信线程）
- `Stop()`：停止通信线程
- `IsRunning()`：是否运行中

#### 运动控制

##### `bool SetFollowPositions(const uint16_t* positions, uint8_t count)`

- 功能：随动模式，发送目标位置数组，设备平滑跟随
- 参数：
    - `positions`: `const uint16_t*`，长度至少为 `count` 的数组
    - `count`: `uint8_t`，建议固定传 `ALLID(6)`
- 约束：
    - `positions` 不能为空
    - `count` 建议等于 `6`（固定 6 关节）
    - 位置值类型为 `uint16_t`；实际可用范围由设备规格/标定决定。示例工程使用了约 `0~4000` 的值。
- 返回：发送成功为 `true`；失败为 `false`

##### `bool SetPositions(const uint16_t* positions, uint8_t count)`

- 功能：定位模式，发送目标位置数组
- 参数/约束：同 `SetFollowPositions()`

#### 设备控制

##### `bool Enable(uint8_t id)`

- 功能：使能关节
- 参数：
    - `id`: `uint8_t`
        - `0` 表示全部关节
        - `1..6` 表示单个关节
- 返回：成功 `true`

##### `bool EmergencyStop(uint8_t id)`

- 功能：急停（立即停止运动）
- 参数：`id` 取值同 `Enable()`

##### `bool ClearError(uint8_t id)`

- 功能：清除故障码
- 参数：`id` 建议传 `1..6`；若需清全部，建议循环 `1..6` 调用

#### 参数配置

##### `bool SetCurrentProtection(uint8_t id, uint16_t threshold)`

- 功能：设置过流保护阈值
- 参数：
    - `id`: `1..6`
    - `threshold`: `uint16_t`，单位 mA
        - 取值范围（头文件注明）：`300..1500`

##### `bool SetOverTemp(uint8_t id, uint16_t threshold)`

- 功能：设置过温保护阈值
- 参数：
    - `id`: `1..6`
    - `threshold`: `uint16_t`，单位 ℃
        - 约束（`LA_StatusData_t` 注释）：可选范围为 `risetemp + 5 .. 80`

##### `bool SetRiseTemp(uint8_t id, uint16_t threshold)`

- 功能：设置回温启动温度
- 参数：
    - `id`: `1..6`
    - `threshold`: `uint16_t`，单位 ℃
        - 约束（`LA_StatusData_t` 注释）：可选范围为 `20 .. overtemp - 5`

##### `bool SetDeviceID(uint8_t old_id, uint8_t new_id)`

- 功能：修改关节 ID
- 参数：
    - `old_id`: `uint8_t`，建议 `1..6`
    - `new_id`: `uint8_t`，建议 `1..6` 且不与其他关节冲突

##### `bool SetBaudrate(uint8_t id, uint8_t baudrate_code)`

- 功能：设置波特率（按 code 映射）
- 参数：
    - `id`: `1..6`
    - `baudrate_code`: `uint8_t`
        - `0` → 19200
        - `1` → 57600
        - `2` → 115200
        - `3` → 921600

#### 状态查询

##### `bool QueryConfig(uint8_t id)`

- 功能：查询关节配置（将配置回读到状态结构体中）
- 参数：`id` 建议 `1..6`

##### `const LA_StatusData_t* GetStatus() const`

- 功能：获取设备状态快照（只读指针）
- 返回：
    - 非空：当前状态结构体指针
    - 为空：设备未启动/状态尚不可用等

##### `void GetPositions(uint16_t* positions) const` / `void GetTemperatures(uint8_t* temps) const` / `void GetCurrents(uint16_t* currents) const`

- 功能：读取位置/温度/电流数组
- 参数：
    - `positions`: `uint16_t*`，长度至少 `ALLID(6)`
    - `temps`: `uint8_t*`，长度至少 `ALLID(6)`
    - `currents`: `uint16_t*`，长度至少 `ALLID(6)`

#### 异常回调（回调函数签名）

回调类型定义（`include/wa100_device.h`）：

- `using ErrorCallback = std::function<void(uint8_t joint_id, uint8_t error_code)>;`
- `using LeakCallback  = std::function<void(uint8_t leak_status)>;`

注册函数：

- `void RegisterLockedRotorCallback(ErrorCallback callback);`
- `void RegisterTempCallback(ErrorCallback callback);`
- `void RegisterCurrentCallback(ErrorCallback callback);`
- `void RegisterMotorErrorCallback(ErrorCallback callback);`
- `void RegisterLeakCallback(LeakCallback callback);`

回调参数说明：

- `joint_id`: `uint8_t`，关节 ID（通常为 `1..6`）
- `error_code`: `uint8_t`，错误码（按位定义，见下文）
- `leak_status`: `uint8_t`，漏水状态（`0` 表示正常；非 `0` 表示漏水告警）

> 线程模型：回调通常在 SDK 通信线程触发。请避免在回调中做耗时/阻塞操作；如需更新 UI/记录日志，建议投递到你自己的线程/队列。

## 6. 数据结构与错误码

### 6.1 LA_StatusData_t（状态结构体 / 关节状态）

头文件：`include/wa100_types.h`

```cpp
typedef struct WA100_SDK_API LA_StatusData_t {
        uint16_t position[ALLID];
        uint16_t current[ALLID];
        uint16_t target_position[ALLID];
        uint16_t currentprotection[ALLID];
        uint16_t overtemp[ALLID];
        uint16_t risetemp[ALLID];
        uint8_t  id[ALLID];
        uint8_t  baudrate[ALLID];
        uint8_t  temperature[ALLID];
        uint8_t  error_message[ALLID];
        uint8_t  leakwarming;
} LA_StatusData_t;
```

字段含义与范围/约束（来自注释 + 协议宏）：

- `position[i]`: `uint16_t`，当前位置（具体可用范围由设备固件/标定决定）
- `target_position[i]`: `uint16_t`，目标位置
- `current[i]`: `uint16_t`，电流（单位由设备定义；示例按 mA 使用）
- `temperature[i]`: `uint8_t`，温度（℃）
- `currentprotection[i]`: `uint16_t`，过流阈值（mA），范围 `300..1500`
- `overtemp[i]`: `uint16_t`，过温阈值（℃），范围 `risetemp + 5 .. 80`
- `risetemp[i]`: `uint16_t`，回温启动温度（℃），范围 `20 .. overtemp - 5`
- `baudrate[i]`: `uint8_t`，波特率代码：`0`=19200, `1`=57600, `2`=115200, `3`=921600
- `error_message[i]`: `uint8_t`，故障码（按位）
- `leakwarming`: `uint8_t`，漏水告警（`0`=正常，`1`=上端盖报警，`2`=下端盖报警，`3`=上下端盖报警）

> 说明：类型名 `LA_StatusData_t` 属于 SDK 的对外 API（历史命名）。对外语义可按“关节状态”理解即可。

### 6.2 故障码（按位）

头文件：`include/wa100_types.h`

- `LA_ERROR_STALL` (`0x01`)：堵转保护
- `LA_ERROR_OVERTEMP` (`0x02`)：过温保护
- `LA_ERROR_OVERCURRENT` (`0x04`)：过流保护
- `LA_ERROR_MOTOR_FAULT` (`0x08`)：电机异常
- `LA_ERROR_COMMUNICATION` (`0x10`)：通信错误
- `LA_ERROR_TIMEOUT` (`0x20`)：响应超时
- `LA_ERROR_FOLLOW_MODE` (`0x40`)：随动模式标志
- `LA_ERROR_OFFLINE` (`0x80`)：设备离线

解析方式示例：

```cpp
uint8_t err = status->error_message[i];
if (err & LA_ERROR_OVERCURRENT) {
        // 过流
}
```

## 7. 常见问题（FAQ）

### 7.1 Windows 终端中文乱码

如果示例程序输出中文是乱码：

- 推荐做法：在 `main()` 开头调用 `SetConsoleOutputCP(CP_UTF8);`（示例工程已处理）
- 或在 PowerShell/CMD 中执行：`chcp 65001`

## 8. 示例代码索引

- `examples/basic_demo.cpp`：多设备添加、启动、随动位置控制、基础状态读取
- `examples/advanced_demo.cpp`：阈值配置、配置查询、错误码解析与清除、状态表格打印

## 9. RS485 协议对接（可选）

如果你不使用本仓库提供的 DLL/头文件，而是希望直接通过串口自研对接，请参考协议文档：

- `docs/wa100_uart_protocol.md`
