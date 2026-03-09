/******************************************************************************
 * @file    advanced_demo.cpp
 * @brief   WA100水下机械手SDK高级功能示例
 * @details 本示例演示SDK的高级功能：
 *          - 参数配置（过流/过温保护等）
 *          - 状态查询
 *          - 设备控制（使能、急停、清除故障等）
 *          - 错误处理
 * @author  WA100 SDK Team
 * @date    2026-01-05
 ******************************************************************************/

#include "wa100_device.h"
#include "wa100_device_manager.h"
#include "wa100_types.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

#ifdef _WIN32
#include <Windows.h>
#endif

// 全局标志：用于错误处理演示
bool g_error_detected = false;

/******************************************************************************
 * 异常回调函数
 ******************************************************************************/
void OnLockedRotor(uint8_t id, uint8_t error_code) {
    std::cout << "\n>>> 堵转异常 - 电缸ID: " << (int)id 
              << ", 错误码: 0x" << std::hex << (int)error_code << std::dec << std::endl;
    g_error_detected = true;
}

void OnOverTemp(uint8_t id, uint8_t error_code) {
    std::cout << "\n>>> 过温异常 - 电缸ID: " << (int)id 
              << ", 错误码: 0x" << std::hex << (int)error_code << std::dec << std::endl;
    g_error_detected = true;
}

void OnOverCurrent(uint8_t id, uint8_t error_code) {
    std::cout << "\n>>> 过流异常 - 电缸ID: " << (int)id 
              << ", 错误码: 0x" << std::hex << (int)error_code << std::dec << std::endl;
    g_error_detected = true;
}

void OnLeak(uint8_t leak_status) {
    if (leak_status != 0) {
        std::cout << "\n>>> 漏水警报！状态: " << (int)leak_status << std::endl;
        g_error_detected = true;
    }
}

/******************************************************************************
 * @brief   打印设备完整状态信息
 * @param   device - 设备指针
 * @param   device_name - 设备名称
 ******************************************************************************/
void PrintDeviceStatus(WA100Device* device, const std::string& device_name) {
    const LA_StatusData_t* status = device->GetStatus();
    if (!status) {
        std::cout << "无法获取设备状态" << std::endl;
        return;
    }
    
    std::cout << "\n========== " << device_name << " 设备状态 ==========" << std::endl;
    std::cout << std::left;
    std::cout << std::setw(8) << "电缸ID" 
              << std::setw(12) << "当前位置" 
              << std::setw(12) << "目标位置"
              << std::setw(10) << "温度(℃)" 
              << std::setw(12) << "电流(mA)"
              << std::setw(10) << "故障码" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    
    for (int i = 0; i < ALLID; i++) {
        std::cout << std::setw(8) << (int)status->id[i]
                  << std::setw(12) << status->position[i]
                  << std::setw(12) << status->target_position[i]
                  << std::setw(10) << (int)status->temperature[i]
                  << std::setw(12) << status->current[i]
                  << "0x" << std::hex << std::setw(2) << std::setfill('0') 
                  << (int)status->error_message[i] << std::dec << std::setfill(' ') << std::endl;
    }
    
    std::cout << "\n漏水状态: " << (status->leakwarming ? "检测到漏水" : "正常") << std::endl;
    std::cout << "===============================================\n" << std::endl;
}

/******************************************************************************
 * @brief   演示参数配置功能
 ******************************************************************************/
void DemoParameterConfiguration(WA100Device* device) {
    std::cout << "\n========== 参数配置演示 ==========" << std::endl;
    
    // 1. 设置过流保护阈值
    std::cout << "设置电缸1的过流保护阈值为800mA..." << std::endl;
    if (device->SetCurrentProtection(1, 800)) {
        std::cout << "✓ 过流保护阈值设置成功" << std::endl;
    } else {
        std::cout << "✗ 过流保护阈值设置失败" << std::endl;
    }
    
    // 2. 设置过温保护阈值
    std::cout << "设置电缸1的过温保护阈值为60℃..." << std::endl;
    if (device->SetOverTemp(1, 60)) {
        std::cout << "✓ 过温保护阈值设置成功" << std::endl;
    } else {
        std::cout << "✗ 过温保护阈值设置失败" << std::endl;
    }
    
    // 3. 设置回温启动温度
    std::cout << "设置电缸1的回温启动温度为40℃..." << std::endl;
    if (device->SetRiseTemp(1, 40)) {
        std::cout << "✓ 回温启动温度设置成功" << std::endl;
    } else {
        std::cout << "✗ 回温启动温度设置失败" << std::endl;
    }
    
    // 等待设置生效
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 4. 查询配置
    std::cout << "\n查询电缸1的配置..." << std::endl;
    if (device->QueryConfig(1)) {
        std::cout << "✓ 配置查询成功" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        const LA_StatusData_t* status = device->GetStatus();
        if (status) {
            std::cout << "  过流阈值: " << status->currentprotection[0] << " mA" << std::endl;
            std::cout << "  过温阈值: " << status->overtemp[0] << " ℃" << std::endl;
            std::cout << "  回温温度: " << status->risetemp[0] << " ℃" << std::endl;
        }
    } else {
        std::cout << "✗ 配置查询失败" << std::endl;
    }
    
    std::cout << "=====================================\n" << std::endl;
}

/******************************************************************************
 * @brief   演示设备控制功能
 ******************************************************************************/
void DemoDeviceControl(WA100Device* device) {
    std::cout << "\n========== 设备控制演示 ==========" << std::endl;
    
    // 1. 使能电缸
    std::cout << "使能所有电缸..." << std::endl;
    if (device->Enable(0)) {  // ID=0表示全部电缸
        std::cout << "✓ 电缸使能成功" << std::endl;
    } else {
        std::cout << "✗ 电缸使能失败" << std::endl;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 2. 设置位置（定位模式）
    std::cout << "\n使用定位模式移动到中间位置..." << std::endl;
    uint16_t mid_pos[6] = {2048, 2048, 2048, 2048, 2048, 2048};
    if (device->SetPositions(mid_pos, 6)) {
        std::cout << "✓ 定位指令发送成功" << std::endl;
    } else {
        std::cout << "✗ 定位指令发送失败" << std::endl;
    }
    
    // 等待运动完成
    std::cout << "等待运动完成..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // 3. 演示急停功能（仅演示，实际不执行）
    std::cout << "\n注意：急停功能会立即停止所有运动" << std::endl;
    std::cout << "（本演示不实际执行急停，仅说明用法）" << std::endl;
    std::cout << "调用方法: device->EmergencyStop(0); // 0表示所有电缸" << std::endl;
    
    // 如需实际测试急停：
    // device->EmergencyStop(0);
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "=====================================\n" << std::endl;
}

/******************************************************************************
 * @brief   演示错误处理
 ******************************************************************************/
void DemoErrorHandling(WA100Device* device) {
    std::cout << "\n========== 错误处理演示 ==========" << std::endl;
    
    // 检查设备状态中的错误信息
    const LA_StatusData_t* status = device->GetStatus();
    if (!status) {
        std::cout << "无法获取设备状态" << std::endl;
        return;
    }
    
    std::cout << "检查各电缸错误状态..." << std::endl;
    bool has_error = false;
    
    for (int i = 0; i < ALLID; i++) {
        uint8_t error = status->error_message[i];
        if (error != LA_ERROR_NONE) {
            has_error = true;
            std::cout << "电缸 " << (int)status->id[i] << " 错误码: 0x" 
                      << std::hex << (int)error << std::dec << " - ";
            
            // 解析错误码
            if (error & LA_ERROR_STALL) std::cout << "[堵转] ";
            if (error & LA_ERROR_OVERTEMP) std::cout << "[过温] ";
            if (error & LA_ERROR_OVERCURRENT) std::cout << "[过流] ";
            if (error & LA_ERROR_MOTOR_FAULT) std::cout << "[电机异常] ";
            if (error & LA_ERROR_COMMUNICATION) std::cout << "[通信错误] ";
            if (error & LA_ERROR_TIMEOUT) std::cout << "[响应超时] ";
            if (error & LA_ERROR_OFFLINE) std::cout << "[设备离线] ";
            std::cout << std::endl;
        }
    }
    
    if (!has_error) {
        std::cout << "✓ 所有电缸状态正常" << std::endl;
    } else {
        std::cout << "\n尝试清除故障..." << std::endl;
        // 清除所有电缸的故障
        for (int i = 1; i <= ALLID; i++) {
            if (status->error_message[i-1] != LA_ERROR_NONE) {
                std::cout << "清除电缸 " << i << " 的故障..." << std::endl;
                if (device->ClearError(i)) {
                    std::cout << "✓ 故障清除成功" << std::endl;
                } else {
                    std::cout << "✗ 故障清除失败" << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        }
    }
    
    std::cout << "=====================================\n" << std::endl;
}

/******************************************************************************
 * @brief   演示状态查询
 ******************************************************************************/
void DemoStatusQuery(WA100Device* device, const std::string& device_name) {
    std::cout << "\n========== 状态查询演示 ==========" << std::endl;
    
    // 1. 获取完整状态
    PrintDeviceStatus(device, device_name);
    
    // 2. 单独获取位置信息
    uint16_t positions[ALLID];
    device->GetPositions(positions);
    std::cout << "当前位置: [";
    for (int i = 0; i < ALLID; i++) {
        std::cout << positions[i];
        if (i < ALLID - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    // 3. 单独获取温度信息
    uint8_t temps[ALLID];
    device->GetTemperatures(temps);
    std::cout << "当前温度: [";
    for (int i = 0; i < ALLID; i++) {
        std::cout << (int)temps[i] << "℃";
        if (i < ALLID - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    // 4. 单独获取电流信息
    uint16_t currents[ALLID];
    device->GetCurrents(currents);
    std::cout << "当前电流: [";
    for (int i = 0; i < ALLID; i++) {
        std::cout << currents[i] << "mA";
        if (i < ALLID - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    std::cout << "=====================================\n" << std::endl;
}

/******************************************************************************
 * @brief   主函数
 ******************************************************************************/
int main()
{
#ifdef _WIN32
    // 设置Windows控制台代码页为UTF-8，解决中文乱码问题
    SetConsoleOutputCP(CP_UTF8);
#endif

    std::cout << "========================================" << std::endl;
    std::cout << "  WA100 SDK 高级功能演示" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 初始化设备管理器
    auto& manager = WA100DeviceManager::Instance();
    std::string config_file = "serial_config.json";
    
    // 添加设备
    std::cout << "\n添加设备..." << std::endl;
    if (!manager.AddDevice("HAND1", config_file, 1)) {
        std::cerr << "添加设备失败！" << std::endl;
        return -1;
    }
    std::cout << "✓ 设备添加成功" << std::endl;
    
    // 获取设备指针
    auto device = manager.GetDevice("HAND1");
    if (!device) {
        std::cerr << "获取设备指针失败！" << std::endl;
        return -1;
    }
    
    // 注册回调函数
    std::cout << "注册异常回调..." << std::endl;
    device->RegisterLockedRotorCallback(OnLockedRotor);
    device->RegisterTempCallback(OnOverTemp);
    device->RegisterCurrentCallback(OnOverCurrent);
    device->RegisterLeakCallback(OnLeak);
    std::cout << "✓ 回调注册完成" << std::endl;
    
    // 启动设备
    std::cout << "\n启动设备..." << std::endl;
    if (!manager.StartAll()) {
        std::cerr << "启动设备失败！" << std::endl;
        manager.Clear();
        return -1;
    }
    std::cout << "✓ 设备启动成功" << std::endl;
    
    // 等待初始化
    std::cout << "\n等待设备初始化..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // ========== 演示各项功能 ==========
    
    // 1. 参数配置演示
    DemoParameterConfiguration(device);
    
    // 2. 设备控制演示
    DemoDeviceControl(device);
    
    // 3. 状态查询演示
    DemoStatusQuery(device, "HAND1");
    
    // 4. 错误处理演示
    DemoErrorHandling(device);
    
    // 5. 随动模式演示
    std::cout << "\n========== 随动模式演示 ==========" << std::endl;
    std::cout << "执行5次平滑运动..." << std::endl;
    
    uint16_t pos_open[6]  = {500, 500, 2000, 2000, 2000, 2000};  // 动作1
    uint16_t pos_close[6] = {2000,  2000,  100,  100,  100,  100};   // 动作2
    
    for (int i = 0; i < 5; i++) {
        std::cout << "  周期 " << (i + 1) << "/5" << std::endl;
        
        device->SetFollowPositions(pos_open, 6);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        device->SetFollowPositions(pos_close, 6);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    std::cout << "✓ 随动演示完成" << std::endl;
    std::cout << "=====================================\n" << std::endl;
    
    // 停止并清理
    std::cout << "停止设备..." << std::endl;
    manager.StopAll();
    std::cout << "✓ 设备已停止" << std::endl;
    
    std::cout << "\n清理资源..." << std::endl;
    manager.RemoveDevice("HAND1");
    manager.Clear();
    std::cout << "✓ 资源清理完成" << std::endl;
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  高级功能演示完毕" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
