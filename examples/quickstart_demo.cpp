#include "wa100_device.h"
#include "wa100_device_manager.h"
#include "wa100_types.h"
#include <chrono>
#include <iostream>
#include <thread>

#ifdef _WIN32
#include <Windows.h>
#endif

// 错误回调函数
void OnLockedRotor(uint8_t id, uint8_t error_code) {
  std::cout << "WARNING: Actuator " << (int)id
            << " locked rotor! Error code: 0x" << std::hex << (int)error_code
            << std::dec << std::endl;
}

void OnOverTemp(uint8_t id, uint8_t error_code) {
  std::cout << "WARNING: Actuator " << (int)id
            << " over temperature! Error code: 0x" << std::hex
            << (int)error_code << std::dec << std::endl;
}

void OnOverCurrent(uint8_t id, uint8_t error_code) {
  std::cout << "WARNING: Actuator " << (int)id
            << " over currentre! Error code: 0x" << std::hex << (int)error_code
            << std::dec << std::endl;
}

void OnLeak(uint8_t leak_status) {
  std::cout << "WARNING: Water leak detected! Status: " << (int)leak_status
            << std::endl;
}

int main() {
#ifdef _WIN32
  // Windows 控制台使用 UTF-8 输出，避免中文乱码
  SetConsoleOutputCP(CP_UTF8);
#endif

  // 创建设备
  auto &manager = WA100DeviceManager::Instance();
  // 建议使用相对路径：serial_config.json 与可执行文件放在同目录（示例 CMake 会自动复制）
  const std::string filename = "serial_config.json";
  std::cout << "Config: " << filename << std::endl;

  if (!manager.AddDevice("HAND1", filename, 1)) {
    std::cerr << "AddDevice(HAND1) failed" << std::endl;
    return -1;
  }
  // if (!manager.AddDevice("HAND2", filename, 2)) {
  //   std::cerr << "AddDevice(HAND2) failed" << std::endl;
  //   manager.RemoveDevice("HAND1");
  //   manager.Clear();
  //   return -1;
  // }
  // 设置动作
  auto device_1 = manager.GetDevice("HAND1");
  // auto device_2 = manager.GetDevice("HAND2");
  if (!device_1 /*|| !device_2*/) {
    std::cerr << "GetDevice failed" << std::endl;
    manager.StopAll();
    manager.Clear();
    return -1;
  }

  // （可选）注册异常回调，方便在示例中看到故障/告警
  device_1->RegisterLockedRotorCallback(OnLockedRotor);
  device_1->RegisterTempCallback(OnOverTemp);
  device_1->RegisterCurrentCallback(OnOverCurrent);
  device_1->RegisterLeakCallback(OnLeak);
  // device_2->RegisterLockedRotorCallback(OnLockedRotor);
  // device_2->RegisterTempCallback(OnOverTemp);
  // device_2->RegisterCurrentCallback(OnOverCurrent);
  // device_2->RegisterLeakCallback(OnLeak);
  uint16_t pos1[6] = {500, 500, 2000, 2000, 2000, 2000};
  uint16_t pos2[6] = {2000, 2000, 0, 0, 0, 0};
  uint32_t flag = 0;
  if (!manager.StartAll()) {
    std::cerr << "StartAll failed" << std::endl;
    manager.Clear();
    return -1;
  }
  while (flag < 20) {
    switch (flag % 2) {
    case 0:
      device_1->SetFollowPositions(&pos1[0], 6);
      break;
    case 1:
      device_1->SetFollowPositions(&pos2[0], 6);
      break;
    }
    // switch (flag % 2) {
    // case 0:
    //   device_2->SetFollowPositions(&pos2[0], 6);
    //   break;
    // case 1:
    //   device_2->SetFollowPositions(&pos1[0], 6);
    //   break;
    // }
    flag++;
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  manager.StopAll();
  manager.RemoveDevice("HAND1");
  // manager.RemoveDevice("HAND2");
  manager.Clear();

  return 0;
}