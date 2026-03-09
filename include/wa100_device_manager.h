#ifndef WA100_DEVICE_MANAGER_H
#define WA100_DEVICE_MANAGER_H

#include "wa100_device.h"
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#ifdef _WIN32
#ifdef WA100_SDK_EXPORTS
#define WA100_SDK_API __declspec(dllexport)
#else
#define WA100_SDK_API __declspec(dllimport)
#endif
#else
#define WA100_SDK_API
#endif

/**
 * @brief WA100设备管理器 - 单例模式
 * 
 * 统一管理多个WA100设备，提供：
 * - 设备创建和销毁
 * - 设备查找
 * - 批量操作
 */
class WA100_SDK_API WA100DeviceManager {
public:
    /**
     * @brief 获取管理器单例
     */
    static WA100DeviceManager& Instance();
    
    /**
     * @brief 添加设备
     * @param name 设备名称（唯一标识）
     * @param filename 设备配置文件目录
     * @param flag 设备号
     * @return true-成功, false-失败（设备已存在或初始化失败）
     */
    bool AddDevice(const std::string& name, const std::string filename,const uint8_t flag);
    
    /**
     * @brief 移除设备
     * @param name 设备名称
     * @return true-成功, false-失败（设备不存在）
     */
    bool RemoveDevice(const std::string& name);
    
    /**
     * @brief 获取设备
     * @param name 设备名称
     * @return 设备指针，如果不存在返回nullptr
     */
    WA100Device* GetDevice(const std::string& name);
    
    /**
     * @brief 检查设备是否存在
     * @param name 设备名称
     */
    bool HasDevice(const std::string& name) const;
    
    // 批量操作
    
    /**
     * @brief 启动所有设备
     * @return true-全部成功, false-至少一个失败
     */
    bool StartAll();
    
    /**
     * @brief 停止所有设备
     */
    void StopAll();
    
    /**
     * @brief 获取设备数量
     */
    size_t GetDeviceCount() const;
    
    /**
     * @brief 获取所有设备名称
     */
    std::vector<std::string> GetDeviceNames() const;
    
    /**
     * @brief 清除所有设备
     */
    void Clear();

private:
    WA100DeviceManager() = default;
    ~WA100DeviceManager();
    
    // 禁止拷贝
    WA100DeviceManager(const WA100DeviceManager&) = delete;
    WA100DeviceManager& operator=(const WA100DeviceManager&) = delete;
    
    std::map<std::string, std::unique_ptr<WA100Device>> devices_;
    mutable std::mutex mutex_;
};

#endif // WA100_DEVICE_MANAGER_H
