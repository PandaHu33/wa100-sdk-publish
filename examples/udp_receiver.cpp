// examples/udp_receiver.cpp
// 必须在包含任何 Windows 头文件之前定义 NOMINMAX，防止 min/max 宏冲突
#define NOMINMAX

#include <winsock2.h>
#include <ws2tcpip.h>
#include "wa100_fast_driver.h" // 引入新的高速驱动
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <fstream> // 引入文件流
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <algorithm> // for std::min, std::max
#include <chrono>    // Added for timeout handling
#include <cctype>
#include <string>

#pragma comment(lib, "ws2_32.lib")

#define PORT 8888
#define BUFFER_SIZE 1024

// 全局变量用于跨线程通信
std::mutex g_mutex;
uint16_t g_udp_target_pos[6] = { 2000, 2000, 2000, 2000, 2000, 2000 }; // Python下发的原始目标
std::atomic<long long> g_last_udp_pkt_time = 0; // 最后一次收到UDP包的时间(ms)
bool g_running = true;

// 全局驱动实例
WA100FastDriver g_fast_driver;

bool IsCsvLoggingEnabled() {
    const char* value = std::getenv("WA100_ENABLE_CSV_LOG");
    if (value == nullptr) {
        return false;
    }
    std::string flag(value);
    std::transform(flag.begin(), flag.end(), flag.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return flag == "1" || flag == "true" || flag == "yes" || flag == "on";
}

// 辅助函数: 获取当前毫秒时间戳
long long GetCurrentTimeMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

// 导纳控制参数 (二阶系统 params - 刚度回归版)
const uint16_t FORCE_THRESHOLD = 500;    
const double SYS_MASS = 0.1;             // 虚拟质量
const double SYS_DAMP = 2.0;             // 基础阻尼 (临界阻尼附近)
const double SYS_K = 8.0;                // [增大] 2.0 -> 8.0 提高刚度，加快回弹速度(解决粘滞)
const double FORCE_GAIN = 8.0;           // [增大] 2.5 -> 8.0 匹配高刚度，保持手感柔软
const double DT = 0.005;                 // 采样周期 200Hz
// 二阶系统状态
struct SecondOrderState {
    double offset = 0.0;   // 柔顺产生的位移偏差 (>= 0, 代表退让量)
    double velocity = 0.0; // 偏差的变化率
};

// 辅助函数: 限制范围
uint16_t Clamp(int val, int min_val, int max_val) {
    if (val < min_val) return (uint16_t)min_val;
    if (val > max_val) return (uint16_t)max_val;
    return (uint16_t)val;
}

// 独立的控制/柔顺线程
void ComplianceControlLoop() {
    uint16_t current_pos[6] = {0};
    uint16_t current_amp[6] = {0};
    uint16_t final_cmd[6];
    uint16_t desired_pos[6]; // 来自UDP的期望位置
    
    // 初始化6个关节的状态 (二阶系统)
    SecondOrderState states[6];

    // [新增] 打开 CSV 文件准备写入
    std::ofstream log_file;
    if (IsCsvLoggingEnabled()) {
        log_file.open("admittance_log.csv");
    }
    if (log_file.is_open()) {
        // 写入表头
        log_file << "TIME,ID,DESIRED,CMD,REAL,AMP\n";
        std::cout << ">>> 日志文件已创建: admittance_log.csv" << std::endl;
    } else if (IsCsvLoggingEnabled()) {
        std::cerr << ">>> 警告: 无法创建日志文件" << std::endl;
    }

    std::cout << ">>> 柔顺控制线程已启动 (连续阻抗模式, 阈值: " << FORCE_THRESHOLD << "mA)" << std::endl;

    auto last_freq_time = std::chrono::high_resolution_clock::now();
    long long loop_counter_freq = 0;

    // [新增] 用于物理积分的时间锚点
    auto last_step_time = std::chrono::high_resolution_clock::now();

    // 连接动作已移至 main 函数中，确保在开启控制循环前已完成配置

    while (g_running) {
        // 0. 计算真实的 dt (解决"频率低"导致的积分慢动作问题)
        auto now_step = std::chrono::high_resolution_clock::now();
        double real_dt = std::chrono::duration<double>(now_step - last_step_time).count();
        // 限制 dt 防止因卡顿导致积分发散 (例如限制在 1ms ~ 100ms)
        if (real_dt < 0.001) real_dt = 0.001;
        if (real_dt > 0.1)   real_dt = 0.1;
        last_step_time = now_step;

        // 1. 获取 Python 下发的最新目标
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            memcpy(desired_pos, g_udp_target_pos, sizeof(desired_pos));
        }

        // 2 & 4. 核心：合并读取和写入
        // 注意：final_cmd 在这里是上一帧计算出的结果，或者直接用 desired_pos 初始化
        // 为了避免第一帧是垃圾值，先赋值
        static bool first_run = true;
        if (first_run) {
            for(int i=0; i<6; i++) final_cmd[i] = desired_pos[i];
            first_run = false;
        }

        // 调用高速驱动: 发送 final_cmd，同时读回 current_pos/amp
        // 注意：这个函数内部已经包含了 15ms sleep，所以不需要额外的 Sleep
        if (!g_fast_driver.SyncWriteRead(final_cmd, current_pos, current_amp)) {
             // 通信失败，可能需要重连或忽略
             // std::cerr << "Comm Error" << std::endl;
        }

        // 3. 逐个关节计算导纳控制 (全物理连续模型)
        for (int i = 0; i < 6; i++) {
            // [新增] 软件急停保护: 既然硬件保护(1A)可能反应慢，软件层检测到 1500mA 立即卸力
            if (current_amp[i] > 1500) {
                 // 策略: 瞬间将 offset 设置为让 Target == RealPos 的值
                 // final = desired + offset => offset = real - desired
                 // 这样 final 就等于 real，电机瞬间不再出力
                 states[i].offset = (double)((int)current_pos[i] - (int)desired_pos[i]);
                 states[i].velocity = 0.0;
                 // 打印一次警告 (限制频率)
                 static int log_limit = 0;
                 if (log_limit++ % 50 == 0) 
                     std::cerr << ">>> [软保护] ID " << i+1 << " 电流过大(" << current_amp[i] << "mA)，强制卸力!" << std::endl;
            }

            // A. 计算虚拟外力 (F_ext)
            // [恢复] 单向导纳 (绳驱动特性): 
            double f_ext = 0.0;
            bool is_under_force = false; // 标记当前是否受力

            // 引入迟滞比较器 (Hysteresis): 
            // 触发阈值 500mA, 只有降到 300mA 以下才认为力消失
            // 防止在阈值附近反复横跳
            static bool force_trigger_state[6] = {false};
            int trigger_on = FORCE_THRESHOLD;
            int trigger_off = FORCE_THRESHOLD - 200;

            if (current_amp[i] > trigger_on) force_trigger_state[i] = true;
            else if (current_amp[i] < trigger_off) force_trigger_state[i] = false;

            if (force_trigger_state[i]) {
                // 线性映射：电流越大，推力越大
                f_ext = (double)(current_amp[i] - trigger_off) * FORCE_GAIN;
                is_under_force = true;
            }

            // B. 物理模拟 (二阶系统) - [核心改进: 非对称阻尼]
            // 策略: "快退让，慢回归"
            // 当受力退让时(is_under_force): 使用标准阻尼，保证响应敏捷。
            // 当力消失回归时(!is_under_force): 使用 5倍 超大阻尼，像陷入泥潭一样慢慢回去。
            
            double effective_damp = is_under_force ? SYS_DAMP : (SYS_DAMP * 8.0);
            
            // 弹簧力(K): 试图将 offset 拉回 0
            double f_spring = -SYS_K * states[i].offset;
            double f_damp   = -effective_damp * states[i].velocity;

            // 加速度 a = F_total / M
            double acc = (f_ext + f_spring + f_damp) / SYS_MASS;

            // 欧拉积分 (使用真实的 real_dt)
            states[i].velocity += acc * real_dt;
            states[i].offset   += states[i].velocity * real_dt;

            // [新增] 速度钳制 (防止动能过大导致超调)
            // 限制最大回弹速度
            // [放松限制] -500 -> -5000: 允许快速回弹，解决视觉错位问题
            if (states[i].velocity < -5000.0) states[i].velocity = -5000.0;
            if (states[i].velocity > 5000.0) states[i].velocity = 5000.0;

            // C. 物理约束 (单向限制)
            if (states[i].offset < 0.0) {
                states[i].offset = 0.0;
                states[i].velocity = 0.0; 
            }
            
            // 限制最大退让距离
            if (states[i].offset > 1800.0) {
                states[i].offset = 1800.0;
                states[i].velocity = 0.0;
            }
            // 注意：恢复了单向限制，适配绳驱动结构

            // D. 最终输出
            // 策略: 2000=张开, 0=闭合。
            int compliant_target = (int)desired_pos[i] + (int)states[i].offset;
            
            // 最后的 Clamp 保护，确保不超机械限位
            final_cmd[i] = Clamp(compliant_target, 0, 2000);
        }
        
        // [数据记录] 输出 CSV 格式日志，便于绘制"期望 vs 导纳后"曲线
        // 格式: TIME, CH, DESIRED, COMPLIANT_CMD, REAL_POS, CURRENT
        static int log_counter = 0;
        if (log_counter++ % 2 == 0) { // 25Hz 输出

            long long t = GetCurrentTimeMs();
            // 写入文件而不打印到控制台，保持控制台整洁
            if (log_file.is_open()) {
                // 记录 食指(ID3, idx=2)
                log_file << t << ",ID3," 
                         << desired_pos[2] << "," 
                         << final_cmd[2] << "," 
                         << current_pos[2] << "," 
                         << current_amp[2] << "\n";
            }
        }

        // 5. 控制频率 统计
        loop_counter_freq++;
        auto now_freq = std::chrono::high_resolution_clock::now();
        double elapsed_freq = std::chrono::duration<double>(now_freq - last_freq_time).count();
        if (elapsed_freq >= 2.0) {
            double freq = loop_counter_freq / elapsed_freq;
            //std::cout << ">>> Loop Freq: " << freq << " Hz" << std::endl;
            loop_counter_freq = 0;
            last_freq_time = now_freq;
        }
    }
}

int main() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    // 1. 初始化 WA100 灵巧手
    // 我们现在使用 g_fast_driver 在独立线程中连接
    std::cout << ">>> 准备连接灵巧手..." << std::endl;
    
    // 2. 初始化 UDP Socket Server (非阻塞模式 + Drain)
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    SOCKET server_fd;
    struct sockaddr_in server_addr, client_addr;
    int client_len = sizeof(client_addr);
    char buffer[BUFFER_SIZE];

    if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET) {
        std::cerr << "Socket 创建失败" << std::endl;
        return -1;
    }

    // 设置非阻塞
    u_long mode = 1;
    ioctlsocket(server_fd, FIONBIO, &mode);

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        std::cerr << "Bind 失败，端口可能被占用" << std::endl;
        return -1;
    }

    std::cout << ">>> UDP 服务已就绪 (端口 " << PORT << ")" << std::endl;
    std::cout << ">>> 模式: 二阶阻尼系统 (Mass=" << SYS_MASS << ", Damp=" << SYS_DAMP << ", K=" << SYS_K << ")" << std::endl;

    // 0. 注册异常回调 (在新驱动 Connect 之前注册是个好习惯)
    g_fast_driver.RegisterCurrentCallback([](uint8_t id, uint8_t code) {
        std::cerr << ">>> [严重警报] 关节 " << (int)id << " 触发过流保护 (OverCurrent)! 错误码: " << (int)code << std::endl;
    });

    g_fast_driver.RegisterTempCallback([](uint8_t id, uint8_t code) {
        std::cerr << ">>> [严重警报] 关节 " << (int)id << " 触发过温保护 (OverTemp)! 错误码: " << (int)code << std::endl;
    });

    // 3. 连接设备并配置严格保护参数
    if (!g_fast_driver.Connect("COM4", 1000000)) {
         std::cerr << ">>> 严重错误: 无法打开 COM4!" << std::endl;
         return -1; 
    }

    // [重要] 给串口和设备一点缓冲时间，等待总线稳定
    std::cout << ">>> [初始化] 等待总线稳定 (1s)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << ">>> [安全配置] 正在设置硬件过流保护 (1A)..." << std::endl;
    // 增加重试机制，确保配置成功
    int success_count = 0;
    for(int id=1; id<=6; id++) {
        bool ret = false;
        // 每个关节尝试 5 次
        for(int try_cnt=0; try_cnt<5; try_cnt++) {
             ret = g_fast_driver.SetCurrentProtection(id, 1000);
             if(ret) break;
             std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        if(!ret) {
             std::cerr << ">>> [错误] 关节 " << id << " 保护参数设置失败 (已重试)!" << std::endl;
        } else {
             std::cout << ">>> [成功] 关节 " << id << " 保护参数已设置" << std::endl;
             success_count++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (success_count < 6) {
        std::cerr << ">>> [警告] 部分关节保护参数设置失败，请检查连接或电源! (成功: " << success_count << "/6)" << std::endl;
        // 可选：如果必须严格保护，可以在此 return -1; 终止程序
    } else {
        std::cout << ">>> [安全配置] 所有关节硬件保护已生效" << std::endl;
    }

    // 开启电机使能
    g_fast_driver.EnableAll();

    // 4. 启动控制线程
    std::thread control_thread(ComplianceControlLoop);

    uint16_t temp_pos[6];
    
    // 4. 主循环接收
    while (true) {
        int len = recvfrom(server_fd, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&client_addr, &client_len);
        if (len > 0) {
            if (len == 12) {
                memcpy(temp_pos, buffer, 12);
                
                {
                    std::lock_guard<std::mutex> lock(g_mutex);
                    memcpy(g_udp_target_pos, temp_pos, 12);
                }

                // 清空缓冲区逻辑: 拿最新的一帧
                while (true) {
                    int next_len = recvfrom(server_fd, buffer, BUFFER_SIZE, 0, NULL, NULL);
                    if (next_len == 12) {
                        memcpy(temp_pos, buffer, 12);
                        std::lock_guard<std::mutex> lock(g_mutex);
                        memcpy(g_udp_target_pos, temp_pos, 12);
                    } else {
                        break;
                    }
                }
            }
        } else {
             // 避免空转
             std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    g_running = false;
    control_thread.join();

    // 关闭文件（虽然析构函数也会关，但显式关闭是个好习惯，尤其在多线程环境下）
    // 注意：log_file 是在线程函数中定义的局部变量，线程结束后自动关闭。
    // 这里无需操作。

    closesocket(server_fd);
    WSACleanup();
    return 0;
}
