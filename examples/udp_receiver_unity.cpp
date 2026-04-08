// examples/udp_receiver_unity.cpp
// Unity JSON UDP -> WA100 6DOF target mapping receiver
// Must define NOMINMAX before any Windows headers.
#define NOMINMAX

#include <winsock2.h>
#include <ws2tcpip.h>
#include "wa100_fast_driver.h"
#include "json.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <limits>
#include <vector>

#pragma comment(lib, "ws2_32.lib")

using json = nlohmann::json;

#define UDP_IP "127.0.0.1"
#define PORT 25001
#define BUFFER_SIZE 65536
#define MAX_UDP_DRAIN_PER_TICK 16

enum class UnityHandSide {
    Left,
    Right,
};

// Global cross-thread state
std::mutex g_mutex;
uint16_t g_udp_target_pos[6] = {2000, 2000, 2000, 2000, 2000, 2000};
std::atomic<long long> g_last_udp_pkt_time = 0;
bool g_running = true;
UnityHandSide g_unity_hand_side = UnityHandSide::Right;

WA100FastDriver g_fast_driver;

long long GetCurrentTimeMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

// Tendon-driven hand:
// motor current only reflects load when pulling in the cable tension direction,
// so this demo keeps a one-way admittance model on purpose.
const uint16_t FORCE_THRESHOLD = 500;
const double SYS_MASS = 0.18;
const double SYS_DAMP = 3.5;
const double SYS_K = 6.0;
const double FORCE_GAIN = 7.0;
const uint16_t HARD_CURRENT_LIMIT_MA = 1000;
const uint16_t SOFT_CURRENT_WARN_MA = 550;
const uint16_t SOFT_CURRENT_RELEASE_MA = 680;
const double CURRENT_FILTER_ALPHA = 0.25;
const double DESIRED_SLEW_RATE = 3500.0;
const double MAX_ADMITTANCE_VEL = 3500.0;
const double MAX_ADMITTANCE_OFFSET = 1800.0;
const uint16_t INVALID_POSITION_THRESHOLD = 10000;

struct SecondOrderState {
    double offset = 0.0;
    double velocity = 0.0;
};

uint16_t Clamp(int val, int min_val, int max_val) {
    if (val < min_val) return (uint16_t)min_val;
    if (val > max_val) return (uint16_t)max_val;
    return (uint16_t)val;
}

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

static inline Vec3 Sub(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

static inline double Dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline Vec3 Cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

static inline double Norm(const Vec3& v) {
    return std::sqrt(Dot(v, v));
}

static inline Vec3 Normalize(const Vec3& v) {
    double n = Norm(v);
    if (n < 1e-9) return {0.0, 0.0, 0.0};
    return {v.x / n, v.y / n, v.z / n};
}

static inline double Clip(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

static inline double WrapAngleDeg(double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}

static inline double AngleDiffDeg(double a, double b) {
    return WrapAngleDeg(a - b);
}

static inline double LerpMap(double value, double in_min, double in_max, double out_min, double out_max) {
    if (std::abs(in_max - in_min) < 1e-9) return out_min;
    double t = (value - in_min) / (in_max - in_min);
    t = Clip(t, 0.0, 1.0);
    return out_min + t * (out_max - out_min);
}

static bool GetJoint(const std::vector<double>& pos, int idx, Vec3& out) {
    int base = idx * 3;
    if (base + 2 >= (int)pos.size()) return false;
    out = {pos[base], pos[base + 1], pos[base + 2]};
    return true;
}

static const char* HandSideToString(UnityHandSide side) {
    return side == UnityHandSide::Left ? "left" : "right";
}

static bool ParseHandSideArg(const char* arg, UnityHandSide& out_side) {
    if (arg == nullptr) return false;
    std::string s(arg);
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    if (s == "left" || s == "l") {
        out_side = UnityHandSide::Left;
        return true;
    }
    if (s == "right" || s == "r") {
        out_side = UnityHandSide::Right;
        return true;
    }
    return false;
}

// Map Unity hand joints to WA100 targets [thumb_yaw, thumb_pitch, index, middle, ring, pinky]
static bool MapUnityToTargets(const json& j, uint16_t out_targets[6]) {
    const char* pos_key = g_unity_hand_side == UnityHandSide::Left ? "leftPositions" : "rightPositions";
    const char* rot_key = g_unity_hand_side == UnityHandSide::Left ? "leftRotations" : "rightRotations";

    if (!j.contains(pos_key) || !j[pos_key].is_array() ||
        !j.contains(rot_key) || !j[rot_key].is_array()) {
        return false;
    }

    std::vector<double> hand_pos;
    hand_pos.reserve(j[pos_key].size());
    for (const auto& v : j[pos_key]) {
        if (!v.is_number()) return false;
        hand_pos.push_back(v.get<double>());
    }

    if (hand_pos.size() < 63) {
        return false;
    }

    std::vector<double> hand_rot;
    hand_rot.reserve(j[rot_key].size());
    for (const auto& v : j[rot_key]) {
        if (!v.is_number()) return false;
        hand_rot.push_back(v.get<double>());
    }

    if (hand_rot.size() < 63) {
        return false;
    }

    Vec3 wrist_r;
    Vec3 thumb_cmc_r, thumb_mcp_r, thumb_ip_r;
    Vec3 thumb_cmc_p, thumb_mcp_p, thumb_ip_p, thumb_tip_p;
    Vec3 index_mcp_r, index_pip_r;
    Vec3 middle_mcp_r, middle_pip_r;
    Vec3 ring_mcp_r, ring_pip_r;
    Vec3 pinky_mcp_r, pinky_pip_r;

    if (!GetJoint(hand_rot, 0, wrist_r) ||
        !GetJoint(hand_rot, 1, thumb_cmc_r) || !GetJoint(hand_rot, 2, thumb_mcp_r) || !GetJoint(hand_rot, 3, thumb_ip_r) ||
        !GetJoint(hand_pos, 1, thumb_cmc_p) || !GetJoint(hand_pos, 2, thumb_mcp_p) || !GetJoint(hand_pos, 3, thumb_ip_p) ||
        !GetJoint(hand_pos, 4, thumb_tip_p) ||
        !GetJoint(hand_rot, 5, index_mcp_r) || !GetJoint(hand_rot, 6, index_pip_r) || !GetJoint(hand_rot, 9, middle_mcp_r) ||
        !GetJoint(hand_rot, 10, middle_pip_r) || !GetJoint(hand_rot, 13, ring_mcp_r) || !GetJoint(hand_rot, 14, ring_pip_r) ||
        !GetJoint(hand_rot, 17, pinky_mcp_r) || !GetJoint(hand_rot, 18, pinky_pip_r)) {
        return false;
    }

    auto rot_mag = [](const Vec3& r) {
        return std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
    };

    auto rot_to_target = [&](double mag_deg, double open_deg, double close_deg) -> uint16_t {
        // open -> 2000, close -> 0
        double bend = LerpMap(mag_deg, open_deg, close_deg, 0.0, 1.0);
        double target = 2000.0 + bend * (0.0 - 2000.0);
        return Clamp((int)std::lround(target), 0, 2000);
    };

    auto apply_target_sensitivity = [&](uint16_t target, double sensitivity) -> uint16_t {
        // Scale motion around the mid-point so sensitivity can be tuned independently from the feature range.
        double centered = ((double)target - 1000.0) * sensitivity;
        double scaled = 1000.0 + centered;
        return Clamp((int)std::lround(scaled), 0, 2000);
    };

    // Thumb calibration parameters.
    // Yaw: use the most local thumb joint delta to suppress whole-palm rotation coupling.
    // Keep the sign so the motion stays monotonic through the sweep.
    // Pitch: use distal joint local Y-axis delta, which is more stable than wrist-distance in Euler space.
    const double THUMB_YAW_MIN = -15;
    const double THUMB_YAW_MAX = 8.0;
    const double THUMB_PITCH_MIN = 40.0;
    const double THUMB_PITCH_MAX = 85.0;
    const double THUMB_YAW_SENSITIVITY = 1.35;
    const double THUMB_PITCH_SENSITIVITY_BASE = 2.0;
    const double THUMB_PITCH_SENSITIVITY_AT_MIN_YAW = 0.5;

    // Thumb yaw:
    // Prefer the distal local Z delta. It stays more local than root-vs-palm features and,
    // without abs(), preserves direction through the middle of the motion.
    double thumb_mcp_rel_x = AngleDiffDeg(thumb_mcp_r.x, thumb_cmc_r.x);
    double thumb_mcp_rel_y = AngleDiffDeg(thumb_mcp_r.y, thumb_cmc_r.y);
    double thumb_mcp_rel_z = AngleDiffDeg(thumb_mcp_r.z, thumb_cmc_r.z);
    double thumb_ip_rel_x = AngleDiffDeg(thumb_ip_r.x, thumb_mcp_r.x);
    double thumb_ip_rel_y = AngleDiffDeg(thumb_ip_r.y, thumb_mcp_r.y);
    double thumb_ip_rel_z = AngleDiffDeg(thumb_ip_r.z, thumb_mcp_r.z);

    double thumb_yaw_feature = thumb_ip_rel_z;

    auto rot_to_target_inverted = [&](double mag_deg, double open_deg, double close_deg) -> uint16_t {
        double bend = LerpMap(mag_deg, open_deg, close_deg, 0.0, 1.0);
        double target = bend * 2000.0;
        return Clamp((int)std::lround(target), 0, 2000);
    };
    uint16_t thumb_yaw_target_base = rot_to_target_inverted(thumb_yaw_feature, THUMB_YAW_MIN, THUMB_YAW_MAX);
    out_targets[0] = apply_target_sensitivity(thumb_yaw_target_base, THUMB_YAW_SENSITIVITY);

    // Thumb pitch: use the IP local Y-axis bend, which remains meaningful under whole-hand rotation.
    double thumb_pitch_feature = std::abs(thumb_ip_rel_y);
    uint16_t thumb_pitch_target_base = rot_to_target(thumb_pitch_feature, THUMB_PITCH_MIN, THUMB_PITCH_MAX);
    double yaw_feature_ratio = Clip((thumb_yaw_feature - THUMB_YAW_MIN) / (THUMB_YAW_MAX - THUMB_YAW_MIN), 0.0, 1.0);
    double thumb_pitch_sensitivity =
        THUMB_PITCH_SENSITIVITY_AT_MIN_YAW +
        (THUMB_PITCH_SENSITIVITY_BASE - THUMB_PITCH_SENSITIVITY_AT_MIN_YAW) * yaw_feature_ratio;
    out_targets[1] = apply_target_sensitivity(thumb_pitch_target_base, thumb_pitch_sensitivity);

    // Fingers: use weighted MCP/PIP rotation magnitude
    double idx_feature = 0.35 * rot_mag(index_mcp_r) + 0.65 * rot_mag(index_pip_r);
    double mid_feature = 0.35 * rot_mag(middle_mcp_r) + 0.65 * rot_mag(middle_pip_r);
    double ring_feature = 0.35 * rot_mag(ring_mcp_r) + 0.65 * rot_mag(ring_pip_r);
    double pinky_feature = 0.35 * rot_mag(pinky_mcp_r) + 0.65 * rot_mag(pinky_pip_r);

    // Fixed calibration ranges from your measured data.
    // min: fully straight, max: fully bent
    const double IDX_MIN = 9.75, IDX_MAX = 58.67;
    const double MID_MIN = 5.11, MID_MAX = 57.85;
    const double RING_MIN = 6.50, RING_MAX = 58.21;
    const double PINKY_MIN = 13.00, PINKY_MAX = 59.29;


    // Safety: avoid commanding full 0 close to reduce overcurrent risk.
    const int SAFE_OPEN = 2000;
    const int SAFE_CLOSE = 0;
    // Dead zones near min/max: close to straight stays fully open, close to bent stays fully closed.
    const double OPEN_DEADZONE = 0.06;   // normalized ratio [0,1]
    const double CLOSE_DEADZONE = 0.12;  // normalized ratio [0,1]

    auto fixed_calib_to_target = [&](double f, double fmin, double fmax) -> uint16_t {
        double span = fmax - fmin;
        if (span < 1e-6) return (uint16_t)SAFE_OPEN;

        double bent_ratio = Clip((f - fmin) / span, 0.0, 1.0);

        // Piecewise mapping with dead zones at both ends.
        double t = 0.0;
        if (bent_ratio <= OPEN_DEADZONE) {
            t = 0.0;
        } else if (bent_ratio >= (1.0 - CLOSE_DEADZONE)) {
            t = 1.0;
        } else {
            double mid = (bent_ratio - OPEN_DEADZONE) / (1.0 - OPEN_DEADZONE - CLOSE_DEADZONE);
            mid = Clip(mid, 0.0, 1.0);
            // Smoothstep: less jitter near transition boundaries than pure linear.
            t = mid * mid * (3.0 - 2.0 * mid);
        }

        double target = SAFE_OPEN - t * (SAFE_OPEN - SAFE_CLOSE);
        return Clamp((int)std::lround(target), SAFE_CLOSE, SAFE_OPEN);
    };

    out_targets[2] = fixed_calib_to_target(idx_feature, IDX_MIN, IDX_MAX);
    out_targets[3] = fixed_calib_to_target(mid_feature, MID_MIN, MID_MAX);
    out_targets[4] = fixed_calib_to_target(ring_feature, RING_MIN, RING_MAX);
    out_targets[5] = fixed_calib_to_target(pinky_feature, PINKY_MIN, PINKY_MAX);

    // Additional safety: per-packet slew-rate limit for index->pinky.
    static uint16_t last_finger_targets[4] = {2000, 2000, 2000, 2000};
    const int MAX_STEP_PER_PACKET = 80;
    for (int i = 0; i < 4; ++i) {
        uint16_t* t = &out_targets[2 + i];
        int diff = (int)(*t) - (int)last_finger_targets[i];
        if (diff > MAX_STEP_PER_PACKET) *t = (uint16_t)(last_finger_targets[i] + MAX_STEP_PER_PACKET);
        if (diff < -MAX_STEP_PER_PACKET) *t = (uint16_t)(last_finger_targets[i] - MAX_STEP_PER_PACKET);
        last_finger_targets[i] = *t;
    }

    // One-line live print (1Hz): thumb raw data + thumb target + finger calibration.
    // static auto last_cali_print_time = std::chrono::high_resolution_clock::now();
    // auto now_cali = std::chrono::high_resolution_clock::now();
    // double cali_elapsed = std::chrono::duration<double>(now_cali - last_cali_print_time).count();
    // if (cali_elapsed >= 1.0) {
    //     std::cout << "\n>>> THUMB RAW pos(cmc/mcp/ip/tip)="
    //               << "(" << thumb_cmc_p.x << "," << thumb_cmc_p.y << "," << thumb_cmc_p.z << ")/"
    //               << "(" << thumb_mcp_p.x << "," << thumb_mcp_p.y << "," << thumb_mcp_p.z << ")/"
    //               << "(" << thumb_ip_p.x << "," << thumb_ip_p.y << "," << thumb_ip_p.z << ")/"
    //               << "(" << thumb_tip_p.x << "," << thumb_tip_p.y << "," << thumb_tip_p.z << ")"
    //               << " rot(cmc/mcp/ip)="
    //               << "(" << thumb_cmc_r.x << "," << thumb_cmc_r.y << "," << thumb_cmc_r.z << ")/"
    //               << "(" << thumb_mcp_r.x << "," << thumb_mcp_r.y << "," << thumb_mcp_r.z << ")/"
    //               << "(" << thumb_ip_r.x << "," << thumb_ip_r.y << "," << thumb_ip_r.z << ")"
    //               << " feat(mcpRelX/mcpRelY/mcpRelZ/yawSigned/ipRelX/ipRelY/ipRelZ/pitch/yawSens/pitchSens)="
    //               << thumb_mcp_rel_x << "/" << thumb_mcp_rel_y << "/" << thumb_mcp_rel_z << "/"
    //               << thumb_yaw_feature << "/"
    //               << thumb_ip_rel_x << "/" << thumb_ip_rel_y << "/" << thumb_ip_rel_z << "/" << thumb_pitch_feature << "/"
    //               << THUMB_YAW_SENSITIVITY << "/" << thumb_pitch_sensitivity
    //               << " tgt(yaw/pitch)=[" << out_targets[0] << "," << out_targets[1] << "]"
    //               << std::flush;
    //     last_cali_print_time = now_cali;
    // }

    return true;
}

static bool ParseUnityUdpPacket(const char* data, int len, uint16_t out_targets[6]) {
    if (data == nullptr || len <= 0) return false;

    try {
        std::string payload(data, data + len);
        json j = json::parse(payload);

        return MapUnityToTargets(j, out_targets);
    } catch (...) {
        return false;
    }
}

void ComplianceControlLoop() {
    uint16_t current_pos[6] = {0};
    uint16_t current_amp[6] = {0};
    uint16_t final_cmd[6];
    uint16_t desired_pos[6];
    uint16_t shaped_desired[6];
    uint16_t last_valid_pos[6] = {0};

    SecondOrderState states[6];
    double desired_filters[6] = {0.0};
    double filtered_amp[6] = {0.0};

    std::ofstream log_file("admittance_log.csv");
    if (log_file.is_open()) {
        log_file << "TIME,ID,RAW_DESIRED,SHAPED_DESIRED,CMD,REAL,AMP,OFFSET\n";
        std::cout << ">>> 日志文件已创建: admittance_log.csv" << std::endl;
    } else {
        std::cerr << ">>> 警告: 无法创建日志文件" << std::endl;
    }

    std::cout << ">>> 柔顺控制线程已启动 (Unity输入, 阈值: " << FORCE_THRESHOLD << "mA)" << std::endl;

    auto last_freq_time = std::chrono::high_resolution_clock::now();
    long long loop_counter_freq = 0;
    auto last_step_time = std::chrono::high_resolution_clock::now();

    while (g_running) {
        auto now_step = std::chrono::high_resolution_clock::now();
        double real_dt = std::chrono::duration<double>(now_step - last_step_time).count();
        if (real_dt < 0.001) real_dt = 0.001;
        if (real_dt > 0.1) real_dt = 0.1;
        last_step_time = now_step;

        {
            std::lock_guard<std::mutex> lock(g_mutex);
            memcpy(desired_pos, g_udp_target_pos, sizeof(desired_pos));
        }

        static bool first_run = true;
        if (first_run) {
            for (int i = 0; i < 6; i++) {
                final_cmd[i] = desired_pos[i];
                shaped_desired[i] = desired_pos[i];
                desired_filters[i] = (double)desired_pos[i];
                last_valid_pos[i] = desired_pos[i];
            }
            first_run = false;
        }

        if (!g_fast_driver.SyncWriteRead(final_cmd, current_pos, current_amp)) {
            // Ignore transient communication failures.
        }

        for (int i = 0; i < 6; i++) {
            // Filter out clearly invalid position feedback such as 0xFFFF.
            // Small overshoot around 2000 is still considered valid.
            if (current_pos[i] <= INVALID_POSITION_THRESHOLD) {
                last_valid_pos[i] = current_pos[i];
            } else {
                static int invalid_pos_log_limit[6] = {0};
                if (invalid_pos_log_limit[i]++ % 50 == 0) {
                    std::cerr << ">>> [Position Filter] ID " << i + 1 << " invalid real position "
                              << current_pos[i] << ", using last valid " << last_valid_pos[i] << std::endl;
                }
                current_pos[i] = last_valid_pos[i];
            }

            filtered_amp[i] += ((double)current_amp[i] - filtered_amp[i]) * CURRENT_FILTER_ALPHA;

            double max_step = DESIRED_SLEW_RATE * real_dt;
            double delta = (double)desired_pos[i] - desired_filters[i];
            delta = Clip(delta, -max_step, max_step);
            desired_filters[i] += delta;
            shaped_desired[i] = Clamp((int)std::lround(desired_filters[i]), 0, 2000);

            if (current_amp[i] > SOFT_CURRENT_RELEASE_MA || filtered_amp[i] > SOFT_CURRENT_RELEASE_MA) {
                states[i].offset = (double)((int)current_pos[i] - (int)shaped_desired[i]);
                states[i].velocity = 0.0;
                static int log_limit = 0;
                if (log_limit++ % 50 == 0)
                    std::cerr << ">>> [软保护] ID " << i + 1 << " 电流过大(" << current_amp[i] << "mA)，强制卸力!"
                              << std::endl;
            }

            double f_ext = 0.0;
            bool is_under_force = false;
            static bool force_trigger_state[6] = {false};
            int trigger_on = FORCE_THRESHOLD;
            int trigger_off = FORCE_THRESHOLD - 200;
            if (trigger_off < 0) trigger_off = 0;

            if (filtered_amp[i] > trigger_on)
                force_trigger_state[i] = true;
            else if (filtered_amp[i] < trigger_off)
                force_trigger_state[i] = false;

            if (force_trigger_state[i]) {
                // Tendon-driven one-way admittance:
                // only pulling load produces meaningful current, so only positive retreat is allowed.
                f_ext = (filtered_amp[i] - (double)trigger_off) * FORCE_GAIN;
                if (filtered_amp[i] > SOFT_CURRENT_WARN_MA) {
                    double soft_span = (double)(SOFT_CURRENT_RELEASE_MA - SOFT_CURRENT_WARN_MA);
                    if (soft_span < 1.0) soft_span = 1.0;
                    double soft_scale = 1.0 - ((filtered_amp[i] - (double)SOFT_CURRENT_WARN_MA) / soft_span);
                    f_ext *= Clip(soft_scale, 0.2, 1.0);
                }
                is_under_force = true;
            }

            double effective_damp = is_under_force ? SYS_DAMP : (SYS_DAMP * 6.0);
            double f_spring = -SYS_K * states[i].offset;
            double f_damp = -effective_damp * states[i].velocity;
            double acc = (f_ext + f_spring + f_damp) / SYS_MASS;

            states[i].velocity += acc * real_dt;
            states[i].offset += states[i].velocity * real_dt;

            if (states[i].velocity < -MAX_ADMITTANCE_VEL) states[i].velocity = -MAX_ADMITTANCE_VEL;
            if (states[i].velocity > MAX_ADMITTANCE_VEL) states[i].velocity = MAX_ADMITTANCE_VEL;

            if (states[i].offset < 0.0) {
                states[i].offset = 0.0;
                states[i].velocity = 0.0;
            }

            if (states[i].offset > MAX_ADMITTANCE_OFFSET) {
                states[i].offset = MAX_ADMITTANCE_OFFSET;
                states[i].velocity = 0.0;
            }

            int compliant_target = (int)shaped_desired[i] + (int)states[i].offset;
            final_cmd[i] = Clamp(compliant_target, 0, 2000);
        }

        static int log_counter = 0;
        if (log_counter++ % 2 == 0) {
            long long t = GetCurrentTimeMs();
            if (log_file.is_open()) {
                for (int i = 0; i < 6; i++) {
                    log_file << t << ",ID" << (i + 1) << "," << desired_pos[i] << "," << shaped_desired[i] << ","
                             << final_cmd[i] << "," << current_pos[i] << "," << (int)std::lround(filtered_amp[i]) << ","
                             << (int)std::lround(states[i].offset) << "\n";
                }
            }
        }

        loop_counter_freq++;
        auto now_freq = std::chrono::high_resolution_clock::now();
        double elapsed_freq = std::chrono::duration<double>(now_freq - last_freq_time).count();
        if (elapsed_freq >= 2.0) {
            loop_counter_freq = 0;
            last_freq_time = now_freq;
        }
    }
}

int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    std::cout << ">>> 准备连接灵巧手..." << std::endl;

    if (argc >= 2) {
        UnityHandSide parsed_side;
        if (!ParseHandSideArg(argv[1], parsed_side)) {
            std::cerr << "Usage: udp_receiver_unity [left|right]" << std::endl;
            return -1;
        }
        g_unity_hand_side = parsed_side;
    }

    std::cout << ">>> Unity hand source: " << HandSideToString(g_unity_hand_side) << std::endl;

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

    u_long mode = 1;
    ioctlsocket(server_fd, FIONBIO, &mode);

    server_addr.sin_family = AF_INET;
    // Bind to loopback only, matching Unity sender on same machine.
    server_addr.sin_addr.s_addr = inet_addr(UDP_IP);
    server_addr.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        std::cerr << "Bind 失败，端口可能被占用" << std::endl;
        return -1;
    }

    std::cout << ">>> UDP 服务已就绪 (" << UDP_IP << ":" << PORT << ")" << std::endl;
    std::cout << ">>> 输入格式: Unity JSON (RightPositions等), 输出: 6自由度目标" << std::endl;

    std::cout << ">>> Unity JSON source: "
              << HandSideToString(g_unity_hand_side)
              << "Positions/"
              << HandSideToString(g_unity_hand_side)
              << "Rotations" << std::endl;
    std::cout << ">>> tendon-driven mode: one-way admittance only, soft current band "
              << SOFT_CURRENT_WARN_MA << "-" << SOFT_CURRENT_RELEASE_MA
              << "mA, hardware limit " << HARD_CURRENT_LIMIT_MA << "mA" << std::endl;

    g_fast_driver.RegisterCurrentCallback([](uint8_t id, uint8_t code) {
        std::cerr << ">>> [严重警报] 关节 " << (int)id << " 触发过流保护 (OverCurrent)! 错误码: " << (int)code
                  << std::endl;
    });

    g_fast_driver.RegisterTempCallback([](uint8_t id, uint8_t code) {
        std::cerr << ">>> [严重警报] 关节 " << (int)id << " 触发过温保护 (OverTemp)! 错误码: " << (int)code
                  << std::endl;
    });

    if (!g_fast_driver.Connect("COM4", 1000000)) {
        std::cerr << ">>> 严重错误: 无法打开 COM4!" << std::endl;
        return -1;
    }

    std::cout << ">>> [初始化] 等待总线稳定 (1s)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << ">>> [安全配置] 正在设置硬件过流保护 (1A)..." << std::endl;
    int success_count = 0;
    for (int id = 1; id <= 6; id++) {
        bool ret = false;
        for (int try_cnt = 0; try_cnt < 5; try_cnt++) {
            ret = g_fast_driver.SetCurrentProtection(id, HARD_CURRENT_LIMIT_MA);
            if (ret) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        if (!ret) {
            std::cerr << ">>> [错误] 关节 " << id << " 保护参数设置失败 (已重试)!" << std::endl;
        } else {
            std::cout << ">>> [成功] 关节 " << id << " 保护参数已设置" << std::endl;
            success_count++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (success_count < 6) {
        std::cerr << ">>> [警告] 部分关节保护参数设置失败，请检查连接或电源! (成功: " << success_count << "/6)"
                  << std::endl;
    } else {
        std::cout << ">>> [安全配置] 所有关节硬件保护已生效" << std::endl;
    }

    g_fast_driver.EnableAll();

    std::thread control_thread(ComplianceControlLoop);

    uint16_t temp_pos[6];
    uint16_t latest_valid_pos[6] = {2000, 2000, 2000, 2000, 2000, 2000};
    int dropped_packet_log_counter = 0;
    while (true) {
        int len = recvfrom(server_fd, buffer, BUFFER_SIZE, 0, (struct sockaddr*)&client_addr, &client_len);
        if (len > 0) {
            bool got_valid = false;
            int latest_len = len;
            int drained_packets = 0;

            // Drain only a bounded number of packets and keep just the latest payload.
            // This avoids spending all CPU time parsing stale Unity frames.
            while (drained_packets < MAX_UDP_DRAIN_PER_TICK) {
                int next_len = recvfrom(server_fd, buffer, BUFFER_SIZE, 0, NULL, NULL);
                if (next_len > 0) {
                    latest_len = next_len;
                    drained_packets++;
                } else {
                    break;
                }
            }

            if (drained_packets >= MAX_UDP_DRAIN_PER_TICK) {
                if (dropped_packet_log_counter++ % 100 == 0) {
                    std::cerr << ">>> [UDP] queue busy, dropped stale Unity packets after "
                              << MAX_UDP_DRAIN_PER_TICK << " drains" << std::endl;
                }
            }

            if (ParseUnityUdpPacket(buffer, latest_len, temp_pos)) {
                memcpy(latest_valid_pos, temp_pos, sizeof(temp_pos));
                got_valid = true;
            }

            if (got_valid) {
                {
                    std::lock_guard<std::mutex> lock(g_mutex);
                    memcpy(g_udp_target_pos, latest_valid_pos, sizeof(g_udp_target_pos));
                }
                g_last_udp_pkt_time = GetCurrentTimeMs();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    g_running = false;
    control_thread.join();

    closesocket(server_fd);
    WSACleanup();
    return 0;
}
