#ifndef WA100_DEVICE_H
#define WA100_DEVICE_H

#include "wa100_types.h"
#include <string>
#include <thread>
#include <memory>
#include <functional>
#include <mutex>
#include <condition_variable>

#ifdef _WIN32
#ifdef WA100_SDK_EXPORTS
#define WA100_SDK_API __declspec(dllexport)
#else
#define WA100_SDK_API __declspec(dllimport)
#endif
#else
#define WA100_SDK_API
#endif

// 设备配置结构
struct WA100_SDK_API WA100Config {
    std::string port;              // 串口名称
    unsigned long baudrate;        // 波特率
    int databyte;                  // 数据位
    int parity;                    // 校验位 (0-none, 1-odd, 2-even, 3-mark, 4-space)
    int stopbyte;                  // 停止位
    int flowcontrol;               // 流控制 (0-none, 1-software, 2-hardware)
    uint32_t timeout_ms;           // 超时时间(ms)
    
    // 默认配置
    WA100Config() : 
        port("COM1"),
        baudrate(921600),
        databyte(8),
        parity(0),
        stopbyte(1),
        flowcontrol(0),
        timeout_ms(100) {}
};

// 错误回调类型
using ErrorCallback = std::function<void(uint8_t actuator_id, uint8_t error_code)>;
using LeakCallback = std::function<void(uint8_t leak_status)>;

/**
 * @brief WA100设备类 - 支持多设备独立控制
 * 
 * 每个设备实例管理：
 * - 独立的串口连接
 * - 独立的状态数据
 * - 独立的命令队列
 * - 独立的通信线程
 */
class WA100_SDK_API WA100Device {
public:
    /**
     * @brief 构造函数
     * @param device_name 设备名称（用于标识和日志）
     */
    explicit WA100Device(const std::string& device_name);
    
    /**
     * @brief 析构函数 - 自动停止设备并清理资源
     */
    ~WA100Device();
    
    // 禁止拷贝
    WA100Device(const WA100Device&) = delete;
    WA100Device& operator=(const WA100Device&) = delete;
    
    // 设备生命周期管理
    
    /**
     * @brief 读取串口配置
     * @param filename 串口配置文件
     * @param flag 串口配置数组标识
     * @return true-成功, false-失败
     */
    bool GetConfig(const std::string& filename,uint8_t flag);

    /**
     * @brief 初始化设备
     * @return true-成功, false-失败
     */
    bool Init();
    
    /**
     * @brief 启动设备（开启通信线程）
     * @return true-成功, false-失败
     */
    bool Start();
    
    /**
     * @brief 停止设备（停止通信线程）
     */
    void Stop();
    
    /**
     * @brief 检查设备是否正在运行
     */
    bool IsRunning() const;
    
    // 位置控制
    
    /**
     * @brief 设置随动模式位置
     * @param positions 位置数组
     * @param count 数量（应该等于ALLID）
     * @return true-成功, false-失败
     */
    bool SetFollowPositions(const uint16_t* positions, uint8_t count);
    
    /**
     * @brief 设置定位模式位置
     * @param positions 位置数组
     * @param count 数量
     * @return true-成功, false-失败
     */
    bool SetPositions(const uint16_t* positions, uint8_t count);
    
    // 设备控制
    
    /**
     * @brief 使能电缸
     * @param id 电缸ID (0表示全部，1-6表示单个)
     */
    bool Enable(uint8_t id);
    
    /**
     * @brief 急停
     * @param id 电缸ID (0表示全部，1-6表示单个)
     */
    bool EmergencyStop(uint8_t id);
    
    /**
     * @brief 清除故障码
     * @param id 电缸ID
     */
    bool ClearError(uint8_t id);
    
    // 参数配置
    
    /**
     * @brief 设置过流保护阈值
     * @param id 电缸ID
     * @param threshold 阈值(mA)，范围300-1500
     */
    bool SetCurrentProtection(uint8_t id, uint16_t threshold);
    
    /**
     * @brief 设置过温保护阈值
     * @param id 电缸ID
     * @param threshold 阈值(℃)
     */
    bool SetOverTemp(uint8_t id, uint16_t threshold);
    
    /**
     * @brief 设置回温启动温度
     * @param id 电缸ID
     * @param threshold 温度(℃)
     */
    bool SetRiseTemp(uint8_t id, uint16_t threshold);
    
    /**
     * @brief 设置电缸ID
     * @param old_id 旧ID
     * @param new_id 新ID
     */
    bool SetDeviceID(uint8_t old_id, uint8_t new_id);
    
    /**
     * @brief 设置波特率
     * @param id 电缸ID
     * @param baudrate_code 波特率代码 (0-19200, 1-57600, 2-115200, 3-921600)
     */
    bool SetBaudrate(uint8_t id, uint8_t baudrate_code);
    
    // 状态查询
    
    /**
     * @brief 查询电缸配置
     * @param id 电缸ID
     */
    bool QueryConfig(uint8_t id);
    
    /**
     * @brief 获取设备状态数据
     * @return 状态数据指针（只读）
     */
    const LA_StatusData_t* GetStatus() const;
    
    /**
     * @brief 获取所有电缸位置
     * @param positions 输出位置数组（至少ALLID个元素）
     */
    void GetPositions(uint16_t* positions) const;
    
    /**
     * @brief 获取所有电缸温度
     * @param temps 输出温度数组（至少ALLID个元素）
     */
    void GetTemperatures(uint8_t* temps) const;
    
    /**
     * @brief 获取所有电缸电流
     * @param currents 输出电流数组（至少ALLID个元素）
     */
    void GetCurrents(uint16_t* currents) const;
    
    // 回调注册
    
    /**
     * @brief 注册堵转异常回调
     */
    void RegisterLockedRotorCallback(ErrorCallback callback);
    
    /**
     * @brief 注册过温异常回调
     */
    void RegisterTempCallback(ErrorCallback callback);
    
    /**
     * @brief 注册过流异常回调
     */
    void RegisterCurrentCallback(ErrorCallback callback);
    
    /**
     * @brief 注册电机异常回调
     */
    void RegisterMotorErrorCallback(ErrorCallback callback);
    
    /**
     * @brief 注册漏水异常回调
     */
    void RegisterLeakCallback(LeakCallback callback);
    
    // 设备信息
    
    /**
     * @brief 获取设备名称
     */
    std::string GetDeviceName() const { return device_name_; }
    
    /**
     * @brief 获取串口名称
     */
    std::string GetPort() const { return config_.port; }

private:
    // Pimpl模式 - 隐藏实现细节
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    
    std::string device_name_;
    WA100Config config_;
    bool is_running_;
};

#endif // WA100_DEVICE_H
