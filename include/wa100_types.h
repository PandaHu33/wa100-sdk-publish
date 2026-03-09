#ifndef WA100_TYPES_H
#define WA100_TYPES_H

#include <cstdint>

#ifdef _WIN32
#ifdef WA100_SDK_EXPORTS
#define WA100_SDK_API __declspec(dllexport)
#else
#define WA100_SDK_API __declspec(dllimport)
#endif
#else
#define WA100_SDK_API
#endif

// 协议常量
#define FRAME_HEADER_SIZE        2
#define FRAME_HEADER1            0x48  // H
#define FRAME_HEADER2            0x4D  // M
#define ALLID                    6     // 电缸数量

// 命令定义
#define CMD_SET_POSITIONS               0x01     // 设置定位模式的位置指令
#define CMD_QUERY_CONFIGURE             0x02     // 查询电缸配置指令
#define CMD_EMERGENCY_STOP              0x03     // 电缸急停指令
#define CMD_ENABLE                      0x04     // 电缸使能指令
#define CMD_FOLLOW_POSITIONS            0x05     // 随动模式指令
#define CMD_SET_ID                      0x06     // 修改电缸ID
#define CMD_OVERCURRENT_PROTECTION      0x07     // 过流保护设置指令
#define CMD_OVERTEMP_PROTECTION         0x08     // 过温保护设置指令
#define CMD_RISETEMP_START              0x09     // 回温启动设置指令
#define CMD_ERROR_CLEAR                 0x0A     // 故障码清除指令
#define CMD_SET_BAUDRATE                0x0B     // 修改电缸波特率

// 故障码定义
#define LA_ERROR_NONE               0x00    // 无错误
#define LA_ERROR_STALL              0x01    // 堵转保护 (Bit0)
#define LA_ERROR_OVERTEMP           0x02    // 过温保护 (Bit1)
#define LA_ERROR_OVERCURRENT        0x04    // 过流保护 (Bit2)
#define LA_ERROR_MOTOR_FAULT        0x08    // 电机异常 (Bit3)
#define LA_ERROR_COMMUNICATION      0x10    // 通信错误 (Bit4)
#define LA_ERROR_TIMEOUT            0x20    // 响应超时 (Bit5)
#define LA_ERROR_FOLLOW_MODE        0x40    // 随动模式标志 (Bit6)
#define LA_ERROR_OFFLINE            0x80    // 设备离线 (Bit7)

// 状态数据结构
typedef struct WA100_SDK_API LA_StatusData_t{
    uint16_t position[ALLID];                 // 当前位置
    uint16_t current[ALLID];                  // 电流
    uint16_t target_position[ALLID];          // 目标位置
    uint16_t currentprotection[ALLID];        // 过流阈值，单位mA，可选范围300-1500mA
    uint16_t overtemp[ALLID];                 // 过温阈值，单位℃，可选范围：回温启动温度+5-80℃
    uint16_t risetemp[ALLID];                 // 回温启动温度，单位℃，可选范围：20℃-过温阈值-5
    uint8_t id[ALLID];                        // ID
    uint8_t baudrate[ALLID];                  // 波特率，0--19200 1--57600 2--115200 3--921600，默认值3
    uint8_t temperature[ALLID];               // 温度
    uint8_t error_message[ALLID];             // 故障码
    uint8_t leakwarming;                      // 漏水警报
} LA_StatusData_t;


// 队列数据结构
#define MAX_QUEUE_SIZE 20

typedef struct {
    uint8_t    cmd;         // 发送指令
    uint8_t    id;
    uint8_t    size;
    uint8_t    priority;    // 优先级
    uint16_t   input_data[MAX_QUEUE_SIZE];
} Queue_data;

#endif // WA100_TYPES_H
