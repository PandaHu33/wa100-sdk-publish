/******************************************************************************
 * @file    basic_demo.cpp
 * @brief   WA100水下机械手SDK基础示例
 * @details 本示例演示如何使用WA100 SDK控制多个水下机械手设备
 * @author  WA100 SDK Team
 * @date    2026-01-05
 ******************************************************************************/

#include "wa100_device.h"
#include "wa100_device_manager.h"
#include "wa100_types.h"
#include <iostream>
#include <thread>
#include <chrono>

#ifdef _WIN32
#include <Windows.h>
#endif

/******************************************************************************
 * @brief   堵转异常回调函数
 * @details 当电缸发生堵转时，SDK会自动调用此回调函数通知上层应用
 * @param   id - 发生堵转的电缸ID（1-6）
 * @param   error_code - 错误代码（详见wa100_types.h中的故障码定义）
 ******************************************************************************/
void OnLockedRotor(uint8_t id, uint8_t error_code) {
    std::cout << "【堵转警告】电缸 #" << (int)id 
              << " 发生堵转！错误码: 0x" 
              << std::hex << (int)error_code << std::dec << std::endl;
}

/******************************************************************************
 * @brief   过温异常回调函数
 * @details 当电缸温度超过设定阈值时触发
 * @param   id - 发生过温的电缸ID（1-6）
 * @param   error_code - 错误代码
 ******************************************************************************/
void OnOverTemp(uint8_t id, uint8_t error_code) {
    std::cout << "【过温警告】电缸 #" << (int)id 
              << " 温度过高！错误码: 0x" 
              << std::hex << (int)error_code << std::dec << std::endl;
}

/******************************************************************************
 * @brief   过流异常回调函数
 * @details 当电缸电流超过设定阈值时触发
 * @param   id - 发生过流的电缸ID（1-6）
 * @param   error_code - 错误代码
 ******************************************************************************/
void OnOverCurrent(uint8_t id, uint8_t error_code) {
    std::cout << "【过流警告】电缸 #" << (int)id 
              << " 电流过大！错误码: 0x" 
              << std::hex << (int)error_code << std::dec << std::endl;
}

/******************************************************************************
 * @brief   漏水异常回调函数
 * @details 当检测到设备漏水时触发（水下设备的关键安全功能）
 * @param   leak_status - 漏水状态（0-正常，非0-检测到漏水）
 ******************************************************************************/
void OnLeak(uint8_t leak_status) {
    if (leak_status != 0) {
        std::cout << "【严重警告】检测到漏水！状态码: " 
                  << (int)leak_status << std::endl;
        std::cout << "请立即停止设备运行并检查密封性！" << std::endl;
    }
}

/******************************************************************************
 * @brief   主函数 - WA100 SDK使用示例
 * @details 演示了SDK的完整使用流程：
 *          1. 获取设备管理器实例
 *          2. 添加多个设备
 *          3. 注册异常回调函数
 *          4. 启动设备
 *          5. 控制设备运动
 *          6. 停止并清理设备
 ******************************************************************************/
int main()
{
#ifdef _WIN32
    // 设置Windows控制台代码页为UTF-8，解决中文乱码问题
    SetConsoleOutputCP(CP_UTF8);
#endif

    std::cout << "========================================" << std::endl;
    std::cout << "  WA100 水下机械手 SDK 基础示例" << std::endl;
    std::cout << "========================================" << std::endl << std::endl;

    // ========== 步骤1: 获取设备管理器单例 ==========
    // WA100DeviceManager采用单例模式，全局只有一个实例
    auto& manager = WA100DeviceManager::Instance();
    
    // ========== 步骤2: 配置文件路径 ==========
    // serial_config.json包含串口配置信息（波特率、端口号等）
    // 注意：实际使用时请修改为您的配置文件路径
    std::string config_file = "serial_config.json";
    std::cout << "配置文件路径: " << config_file << std::endl;
    
    // ========== 步骤3: 添加设备 ==========
    // AddDevice参数说明：
    //   参数1: 设备名称（字符串，用于唯一标识设备）
    //   参数2: 配置文件路径
    //   参数3: 设备索引（对应配置文件中serial_port数组的下标，从1开始）
    std::cout << std::endl << "正在添加设备..." << std::endl;
    
    if (!manager.AddDevice("HAND1", config_file, 1)) {
        std::cerr << "错误：添加设备 HAND1 失败！" << std::endl;
        return -1;
    }
    std::cout << "✓ 设备 HAND1 添加成功" << std::endl;
    
    if (!manager.AddDevice("HAND2", config_file, 2)) {
        std::cerr << "错误：添加设备 HAND2 失败！" << std::endl;
        return -1;
    }
    std::cout << "✓ 设备 HAND2 添加成功" << std::endl;
    
    // ========== 步骤4: 获取设备指针 ==========
    // 通过设备名称获取设备对象，用于后续控制
    auto device_1 = manager.GetDevice("HAND1");
    auto device_2 = manager.GetDevice("HAND2");
    
    if (!device_1 || !device_2) {
        std::cerr << "错误：获取设备指针失败！" << std::endl;
        return -1;
    }
    
    // ========== 步骤5: 注册异常回调函数（可选但推荐） ==========
    // 注册回调函数可以及时获得设备异常通知
    std::cout << std::endl << "正在注册异常回调函数..." << std::endl;
    device_1->RegisterLockedRotorCallback(OnLockedRotor);  // 堵转回调
    device_1->RegisterTempCallback(OnOverTemp);            // 过温回调
    device_1->RegisterCurrentCallback(OnOverCurrent);      // 过流回调
    device_1->RegisterLeakCallback(OnLeak);                // 漏水回调
    
    device_2->RegisterLockedRotorCallback(OnLockedRotor);
    device_2->RegisterTempCallback(OnOverTemp);
    device_2->RegisterCurrentCallback(OnOverCurrent);
    device_2->RegisterLeakCallback(OnLeak);
    std::cout << "✓ 异常回调函数注册完成" << std::endl;
    
    // ========== 步骤6: 定义位置数组 ==========
    // WA100设备有6个电缸（ALLID=6），每个电缸的位置范围：0-4095
    //   0    - 完全收缩
    //   2048 - 中间位置
    //   4095 - 完全伸展
    uint16_t pos1[6] = {500,  500,  2000, 2000, 2000, 2000};  // 姿态1
    uint16_t pos2[6] = {2000, 2000, 0,    0,    0,    0};     // 姿态2
    
    // ========== 步骤7: 启动所有设备 ==========
    // 启动设备后，SDK内部会创建通信线程，开始与设备进行数据交互
    std::cout << std::endl << "正在启动设备..." << std::endl;
    if (!manager.StartAll()) {
        std::cerr << "错误：启动设备失败！" << std::endl;
        manager.Clear();
        return -1;
    }
    std::cout << "✓ 所有设备启动成功" << std::endl;
    
    // 等待设备初始化完成
    std::cout << std::endl << "等待设备初始化..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // ========== 步骤8: 控制设备运动 ==========
    std::cout << std::endl << "开始控制设备运动（交替20次）..." << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    
    uint32_t cycle_count = 0;
    while (cycle_count < 20) {
        // 根据循环次数交替设置两种姿态
        // SetFollowPositions: 随动模式，电缸会平滑跟随目标位置
        switch(cycle_count % 2) {
            case 0:
                std::cout << "周期 " << (cycle_count + 1) << ": HAND1 -> 姿态1, HAND2 -> 姿态2" << std::endl;
                device_1->SetFollowPositions(&pos1[0], 6);  // HAND1设置为姿态1
                device_2->SetFollowPositions(&pos2[0], 6);  // HAND2设置为姿态2
                break;
            case 1:
                std::cout << "周期 " << (cycle_count + 1) << ": HAND1 -> 姿态2, HAND2 -> 姿态1" << std::endl;
                device_1->SetFollowPositions(&pos2[0], 6);  // HAND1设置为姿态2
                device_2->SetFollowPositions(&pos1[0], 6);  // HAND2设置为姿态1
                break;
        }
        
        // 可选：读取并打印设备状态
        const LA_StatusData_t* status1 = device_1->GetStatus();
        if (status1) {
            std::cout << "  HAND1 第1个电缸 - 当前位置: " << status1->position[0] 
                      << ", 温度: " << (int)status1->temperature[0] << "℃" 
                      << ", 电流: " << status1->current[0] << "mA" << std::endl;
        }
        
        cycle_count++;
        
        // 每个动作间隔2秒
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "运动控制完成！" << std::endl;
    
    // ========== 步骤9: 停止设备 ==========
    // 停止所有设备的通信线程
    std::cout << std::endl << "正在停止设备..." << std::endl;
    manager.StopAll();
    std::cout << "✓ 所有设备已停止" << std::endl;
    
    // ========== 步骤10: 移除设备 ==========
    // 从管理器中移除设备（会自动释放资源）
    std::cout << std::endl << "正在清理资源..." << std::endl;
    manager.RemoveDevice("HAND1");
    manager.RemoveDevice("HAND2");
    std::cout << "✓ 设备已移除" << std::endl;
    
    // ========== 步骤11: 清空管理器 ==========
    // 清除管理器中的所有设备（可选，程序退出时会自动清理）
    manager.Clear();
    std::cout << "✓ 资源清理完成" << std::endl;
    
    std::cout << std::endl << "========================================" << std::endl;
    std::cout << "  示例程序执行完毕" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
