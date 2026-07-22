// examples/udp_receiver_unity.cpp
// Unity JSON UDP -> WA100 6DOF target mapping receiver
// Must define NOMINMAX before any Windows headers.
#define NOMINMAX

#include <winsock2.h>
#include <ws2tcpip.h>
#include "wa100_fast_driver.h"
#include "json.hpp"
#include "glove_thumb_mapper.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
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
#define RECORDING_TELEMETRY_IP "127.0.0.1"
#define RECORDING_TELEMETRY_PORT 25003
#define BUFFER_SIZE 65536
#define MAX_UDP_DRAIN_PER_TICK 16

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

bool IsUnityInputLoggingEnabled() {
    const char* value = std::getenv("WA100_ENABLE_UNITY_INPUT_LOG");
    if (value == nullptr) {
        return false;
    }
    std::string flag(value);
    std::transform(flag.begin(), flag.end(), flag.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return flag == "1" || flag == "true" || flag == "yes" || flag == "on";
}

std::string GetLogPath(const char* env_name, const char* fallback) {
    const char* value = std::getenv(env_name);
    return value != nullptr && value[0] != '\0' ? std::string(value) : std::string(fallback);
}

enum class UnityHandSide {
    Left,
    Right,
};

enum class ControlMode {
    Glove = 0,
    Vr = 1,
    Preset = 2,
};

struct SerialSettings {
    std::string port = "COM4";
    uint32_t baudrate = 1000000;
};

// Global cross-thread state
std::mutex g_mutex;
uint16_t g_udp_target_pos[6] = {2000, 2000, 2000, 2000, 2000, 2000};
uint16_t g_preset_target_pos[6] = {2000, 2000, 2000, 2000, 2000, 2000};
std::atomic<long long> g_last_udp_pkt_time = 0;
std::atomic<int> g_control_mode((int)ControlMode::Glove);
bool g_running = true;
UnityHandSide g_unity_hand_side = UnityHandSide::Right;
glove_thumb::Mapper g_glove_thumb_mapper;
glove_thumb::MappingDebug g_last_thumb_mapping_debug;
std::string g_thumb_calibration_path;

WA100FastDriver g_fast_driver;

long long GetCurrentTimeMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

long long GetCurrentTimeNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

long long GetSteadyTimeNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
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
const double DESIRED_SLEW_RATE = 4500.0;
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

static inline double AngleDegBetween(const Vec3& a, const Vec3& b) {
    double denom = Norm(a) * Norm(b);
    if (denom < 1e-9) return 0.0;
    return std::acos(Clip(Dot(a, b) / denom, -1.0, 1.0)) * 180.0 / 3.14159265358979323846;
}

static inline double JointAngleDeg(const Vec3& parent, const Vec3& joint, const Vec3& child) {
    return AngleDegBetween(Sub(parent, joint), Sub(child, joint));
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

static const char* ControlModeToString(ControlMode mode) {
    if (mode == ControlMode::Vr) return "vr";
    if (mode == ControlMode::Preset) return "preset";
    return "glove";
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

static bool JsonStringEquals(const json& j, const char* key, const char* value) {
    return j.contains(key) && j[key].is_string() && j[key].get<std::string>() == value;
}

static bool JsonStringContains(const json& j, const char* key, const char* needle) {
    if (!j.contains(key) || !j[key].is_string()) return false;
    return j[key].get<std::string>().find(needle) != std::string::npos;
}

struct VrHandCalibration {
    double thumb_yaw_open = 0.988191;
    double thumb_yaw_close = 0.558984;
    double thumb_pitch_open = 165.31;
    double thumb_pitch_close = 125.0;
    double index_open = 153.179;
    double index_close = 55.0;
    double middle_open = 147.612;
    double middle_close = 55.0;
    double ring_open = 154.575;
    double ring_close = 56.0;
    double pinky_open = 160.095;
    double pinky_close = 55.0;
    double open_deadzone = 0.05;
    double close_deadzone = 0.08;
    double thumb_yaw_sensitivity = 1;
    double thumb_pitch_sensitivity = 1;
};

static const VrHandCalibration VR_HAND_CALIBRATION;

static uint16_t RatioToTarget(double ratio, double sensitivity = 1.0) {
    double t = Clip(ratio, 0.0, 1.0);
    if (t <= VR_HAND_CALIBRATION.open_deadzone) {
        t = 0.0;
    } else if (t >= 1.0 - VR_HAND_CALIBRATION.close_deadzone) {
        t = 1.0;
    } else {
        t = (t - VR_HAND_CALIBRATION.open_deadzone) /
            (1.0 - VR_HAND_CALIBRATION.open_deadzone - VR_HAND_CALIBRATION.close_deadzone);
        t = Clip(t, 0.0, 1.0);
        t = t * t * (3.0 - 2.0 * t);
    }

    double centered = (t - 0.5) * sensitivity + 0.5;
    centered = Clip(centered, 0.0, 1.0);
    double target = 2000.0 - centered * 2000.0;
    return Clamp((int)std::lround(target), 0, 2000);
}

static double BendRatioFromAngle(double angle_deg, double open_deg, double close_deg) {
    return Clip((open_deg - angle_deg) / std::max(1e-6, open_deg - close_deg), 0.0, 1.0);
}

static double FingerCurlRatio(
    const std::vector<double>& hand_pos,
    int mcp_idx,
    int pip_idx,
    int tip_idx,
    double open_deg,
    double close_deg) {
    Vec3 mcp, pip, tip;
    if (!GetJoint(hand_pos, mcp_idx, mcp) ||
        !GetJoint(hand_pos, pip_idx, pip) ||
        !GetJoint(hand_pos, tip_idx, tip)) {
        return 0.0;
    }

    double pip_angle = JointAngleDeg(mcp, pip, tip);
    return BendRatioFromAngle(pip_angle, open_deg, close_deg);
}

// Independent VR-hand-tracking calibration.
// This path is selected only for packets marked as Unity XR Hands and does not
// reuse the legacy/data-glove Euler calibration in MapUnityToTargets().
static bool MapVrHandTrackingToTargets(const json& j, uint16_t out_targets[6]) {
    const char* pos_key = g_unity_hand_side == UnityHandSide::Left ? "leftPositions" : "rightPositions";

    if (!j.contains(pos_key) || !j[pos_key].is_array()) {
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

    Vec3 wrist, thumb_cmc, thumb_mcp, thumb_ip, thumb_tip;
    Vec3 index_mcp, middle_mcp;
    if (!GetJoint(hand_pos, 0, wrist) ||
        !GetJoint(hand_pos, 1, thumb_cmc) ||
        !GetJoint(hand_pos, 2, thumb_mcp) ||
        !GetJoint(hand_pos, 3, thumb_ip) ||
        !GetJoint(hand_pos, 4, thumb_tip) ||
        !GetJoint(hand_pos, 5, index_mcp) ||
        !GetJoint(hand_pos, 9, middle_mcp)) {
        return false;
    }

    Vec3 palm_x = Normalize(Sub(index_mcp, wrist));
    Vec3 palm_y = Normalize(Sub(middle_mcp, wrist));
    Vec3 palm_z = Normalize(Cross(palm_x, palm_y));
    Vec3 thumb_vec = Normalize(Sub(thumb_tip, thumb_cmc));

    double thumb_across_palm = std::abs(Dot(thumb_vec, palm_x));
    double thumb_yaw_ratio = LerpMap(
        thumb_across_palm,
        VR_HAND_CALIBRATION.thumb_yaw_open,
        VR_HAND_CALIBRATION.thumb_yaw_close,
        0.0,
        1.0);

    double thumb_pitch_angle = JointAngleDeg(thumb_cmc, thumb_mcp, thumb_tip);
    double thumb_pitch_ratio = BendRatioFromAngle(
        thumb_pitch_angle,
        VR_HAND_CALIBRATION.thumb_pitch_open,
        VR_HAND_CALIBRATION.thumb_pitch_close);

    out_targets[0] = RatioToTarget(thumb_yaw_ratio, VR_HAND_CALIBRATION.thumb_yaw_sensitivity);
    out_targets[1] = RatioToTarget(thumb_pitch_ratio, VR_HAND_CALIBRATION.thumb_pitch_sensitivity);
    out_targets[2] = RatioToTarget(FingerCurlRatio(hand_pos, 5, 6, 8, VR_HAND_CALIBRATION.index_open, VR_HAND_CALIBRATION.index_close));
    out_targets[3] = RatioToTarget(FingerCurlRatio(hand_pos, 9, 10, 12, VR_HAND_CALIBRATION.middle_open, VR_HAND_CALIBRATION.middle_close));
    out_targets[4] = RatioToTarget(FingerCurlRatio(hand_pos, 13, 14, 16, VR_HAND_CALIBRATION.ring_open, VR_HAND_CALIBRATION.ring_close));
    out_targets[5] = RatioToTarget(FingerCurlRatio(hand_pos, 17, 18, 20, VR_HAND_CALIBRATION.pinky_open, VR_HAND_CALIBRATION.pinky_close));

    (void)palm_z;
    return true;
}

static bool LooksLikeComPort(const std::string& value) {
    if (value.size() < 4) return false;
    std::string prefix = value.substr(0, 3);
    std::transform(prefix.begin(), prefix.end(), prefix.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    if (prefix != "com") return false;
    for (size_t i = 3; i < value.size(); ++i) {
        if (!std::isdigit((unsigned char)value[i])) return false;
    }
    return true;
}

static std::vector<SerialSettings> LoadSerialSettingsList() {
    SerialSettings defaults;
    std::vector<SerialSettings> settings_list;
    std::ifstream config_file("serial_config.json");
    if (!config_file.is_open()) {
        settings_list.push_back(defaults);
        return settings_list;
    }

    try {
        json cfg;
        config_file >> cfg;
        if (!cfg.contains("serial_port") || !cfg["serial_port"].is_array() || cfg["serial_port"].empty()) {
            settings_list.push_back(defaults);
            return settings_list;
        }

        for (const auto& serial_item : cfg["serial_port"]) {
            if (!serial_item.is_object()) continue;
            SerialSettings settings = defaults;
            if (serial_item.contains("port") && serial_item["port"].is_string()) {
                settings.port = serial_item["port"].get<std::string>();
            }
            if (serial_item.contains("baudrate") && serial_item["baudrate"].is_number_integer()) {
                settings.baudrate = serial_item["baudrate"].get<uint32_t>();
            }
            settings_list.push_back(settings);
        }
    } catch (const std::exception& exc) {
        std::cerr << ">>> Warning: failed to read serial_config.json: " << exc.what() << std::endl;
    }

    if (settings_list.empty()) {
        settings_list.push_back(defaults);
    }
    return settings_list;
}

static bool ParseBaudrate(const std::string& text, uint32_t& out_baudrate) {
    try {
        size_t consumed = 0;
        unsigned long parsed = std::stoul(text, &consumed, 10);
        if (consumed != text.size() || parsed == 0) return false;
        out_baudrate = (uint32_t)parsed;
        return true;
    } catch (...) {
        return false;
    }
}

static std::string LowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    return value;
}

static bool SameSerialPort(const std::string& a, const std::string& b) {
    return LowerCopy(a) == LowerCopy(b);
}

static void AddSerialCandidate(std::vector<SerialSettings>& candidates, const SerialSettings& candidate) {
    for (const auto& existing : candidates) {
        if (SameSerialPort(existing.port, candidate.port) && existing.baudrate == candidate.baudrate) {
            return;
        }
    }
    candidates.push_back(candidate);
}

static std::vector<std::string> EnumerateAvailableComPorts() {
    std::vector<std::string> ports;
#ifdef _WIN32
    char target[512];
    for (int i = 1; i <= 32; ++i) {
        std::string port = "COM" + std::to_string(i);
        if (QueryDosDeviceA(port.c_str(), target, sizeof(target)) != 0) {
            ports.push_back(port);
        }
    }
#endif
    return ports;
}

static bool ReadSixTargets(const json& arr, uint16_t out_targets[6]) {
    if (!arr.is_array() || arr.size() != 6) return false;
    for (int i = 0; i < 6; ++i) {
        if (!arr[i].is_number()) return false;
        out_targets[i] = Clamp((int)std::lround(arr[i].get<double>()), 0, 2000);
    }
    return true;
}

static void StorePresetTargets(const uint16_t targets[6]) {
    std::lock_guard<std::mutex> lock(g_mutex);
    memcpy(g_preset_target_pos, targets, sizeof(g_preset_target_pos));
    memcpy(g_udp_target_pos, targets, sizeof(g_udp_target_pos));
}

static json ThumbMappingDebugToJson(const glove_thumb::MappingDebug& debug) {
    return {
        {"configured", debug.configured},
        {"valid", debug.valid},
        {"calibration_id", debug.calibration_id},
        {"error", debug.error},
        {"feature_norm", debug.feature_norm},
        {"residual_norm", debug.residual_norm},
        {"yaw_projected", debug.yaw_projected},
        {"pitch_projected", debug.pitch_projected},
        {"yaw_score", debug.yaw_score},
        {"pitch_score", debug.pitch_score},
        {"yaw_ratio", debug.yaw_ratio},
        {"pitch_ratio", debug.pitch_ratio},
        {"yaw_target", debug.yaw_target},
        {"pitch_target", debug.pitch_target},
    };
}

static bool ReadFixedVector(
    const json& value,
    std::array<double, glove_thumb::kFeatureDimension>& output,
    std::string& error,
    const char* name) {
    if (!value.is_array() || value.size() != glove_thumb::kFeatureDimension) {
        error = std::string(name) + " must contain 21 numbers";
        return false;
    }
    for (std::size_t index = 0; index < output.size(); ++index) {
        if (!value[index].is_number()) {
            error = std::string(name) + " contains a non-number";
            return false;
        }
        output[index] = value[index].get<double>();
    }
    return true;
}

static bool LoadGloveThumbCalibration(const std::string& path, std::string& error) {
    std::ifstream stream(path);
    if (!stream.is_open()) {
        error = "cannot open calibration file: " + path;
        return false;
    }

    try {
        json root;
        stream >> root;
        glove_thumb::Calibration calibration;
        calibration.schema_version = root.at("schema_version").get<int>();
        calibration.calibration_id = root.at("calibration_id").get<std::string>();
        calibration.hand = root.at("hand").get<std::string>();
        calibration.euler_order = root.at("euler_order").get<std::string>();

        const auto& joints = root.at("feature_joint_indices");
        if (!joints.is_array() || joints.size() != glove_thumb::kSelectedJointCount) {
            error = "feature_joint_indices must contain 7 integers";
            return false;
        }
        for (std::size_t index = 0; index < calibration.joint_indices.size(); ++index) {
            calibration.joint_indices[index] = joints[index].get<int>();
        }

        const auto& references = root.at("reference_quaternions_wxyz");
        if (!references.is_array() || references.size() != glove_thumb::kSelectedJointCount) {
            error = "reference_quaternions_wxyz must contain 7 quaternions";
            return false;
        }
        for (std::size_t index = 0; index < calibration.reference_quaternions.size(); ++index) {
            const auto& values = references[index];
            if (!values.is_array() || values.size() != 4) {
                error = "each reference quaternion must contain [w,x,y,z]";
                return false;
            }
            calibration.reference_quaternions[index] = {
                values[0].get<double>(), values[1].get<double>(), values[2].get<double>(), values[3].get<double>()};
        }

        const auto& nuisance = root.at("nuisance_basis");
        if (!nuisance.is_array()) {
            error = "nuisance_basis must be an array";
            return false;
        }
        for (const auto& row : nuisance) {
            std::array<double, glove_thumb::kFeatureDimension> vector{};
            if (!ReadFixedVector(row, vector, error, "nuisance_basis row")) return false;
            calibration.nuisance_basis.push_back(vector);
        }
        if (!ReadFixedVector(root.at("yaw_axis"), calibration.yaw_axis, error, "yaw_axis")) return false;
        if (!ReadFixedVector(root.at("pitch_axis"), calibration.pitch_axis, error, "pitch_axis")) return false;

        const auto& demix = root.at("demix_matrix");
        if (!demix.is_array() || demix.size() != 2 || !demix[0].is_array() || !demix[1].is_array() ||
            demix[0].size() != 2 || demix[1].size() != 2) {
            error = "demix_matrix must be 2x2";
            return false;
        }
        for (std::size_t row = 0; row < 2; ++row) {
            for (std::size_t column = 0; column < 2; ++column) {
                calibration.demix[row][column] = demix[row][column].get<double>();
            }
        }

        const auto& ranges = root.at("score_ranges");
        calibration.yaw_open_score = ranges.at("yaw").at("open").get<double>();
        calibration.yaw_close_score = ranges.at("yaw").at("close").get<double>();
        calibration.pitch_open_score = ranges.at("pitch").at("open").get<double>();
        calibration.pitch_close_score = ranges.at("pitch").at("close").get<double>();
        if (root.contains("mapping")) {
            calibration.open_deadzone = root["mapping"].value("open_deadzone", calibration.open_deadzone);
            calibration.close_deadzone = root["mapping"].value("close_deadzone", calibration.close_deadzone);
        }

        if (calibration.hand != HandSideToString(g_unity_hand_side)) {
            error = "calibration hand does not match selected Unity hand";
            return false;
        }
        return g_glove_thumb_mapper.Configure(calibration, &error);
    } catch (const std::exception& exception) {
        error = std::string("invalid calibration JSON: ") + exception.what();
        return false;
    }
}

static std::string ExecutableDirectory(const char* argv0) {
    const std::string path = argv0 == nullptr ? std::string() : std::string(argv0);
    const std::size_t separator = path.find_last_of("\\/");
    return separator == std::string::npos ? std::string(".") : path.substr(0, separator);
}

static bool ParseH5ControlPacket(const json& j, uint16_t out_targets[6], bool& handled) {
    handled = false;
    if (!j.is_object()) return false;
    if (!j.contains("type") || !j["type"].is_string() || j["type"].get<std::string>() != "h5_hand_control") {
        return false;
    }

    handled = true;
    std::string mode = j.value("mode", "glove");
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) { return (char)std::tolower(c); });

    if (mode == "glove") {
        g_control_mode.store((int)ControlMode::Glove);
        std::cout << ">>> [H5] control mode -> glove" << std::endl;
        return false;
    }

    if (mode == "vr" || mode == "vision" || mode == "hand_tracking") {
        g_control_mode.store((int)ControlMode::Vr);
        std::cout << ">>> [H5] control mode -> vr" << std::endl;
        return false;
    }

    if (mode == "preset" || mode == "program" || mode == "preprogrammed") {
        g_control_mode.store((int)ControlMode::Preset);
        uint16_t next_targets[6];
        bool has_positions = j.contains("positions") && ReadSixTargets(j["positions"], next_targets);
        if (has_positions) {
            StorePresetTargets(next_targets);
            memcpy(out_targets, next_targets, sizeof(uint16_t) * 6);
        } else {
            std::lock_guard<std::mutex> lock(g_mutex);
            memcpy(out_targets, g_preset_target_pos, sizeof(uint16_t) * 6);
            memcpy(g_udp_target_pos, g_preset_target_pos, sizeof(uint16_t) * 6);
        }

        std::cout << ">>> [H5] control mode -> preset";
        if (j.contains("name") && j["name"].is_string()) {
            std::cout << " (" << j["name"].get<std::string>() << ")";
        }
        std::cout << " targets=[";
        for (int i = 0; i < 6; ++i) {
            if (i) std::cout << ",";
            std::cout << out_targets[i];
        }
        std::cout << "]" << std::endl;
        return true;
    }

    std::cerr << ">>> [H5] unknown control mode: " << mode << std::endl;
    return false;
}

// Map Unity hand joints to WA100 targets [thumb_yaw, thumb_pitch, index, middle, ring, pinky]
static bool MapUnityToTargets(const json& j, uint16_t out_targets[6]) {
    const char* rot_key = g_unity_hand_side == UnityHandSide::Left ? "leftRotations" : "rightRotations";

    if (!j.contains(rot_key) || !j[rot_key].is_array()) {
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

    Vec3 index_mcp_r, index_pip_r;
    Vec3 middle_mcp_r, middle_pip_r;
    Vec3 ring_mcp_r, ring_pip_r;
    Vec3 pinky_mcp_r, pinky_pip_r;

    if (!GetJoint(hand_rot, 5, index_mcp_r) || !GetJoint(hand_rot, 6, index_pip_r) || !GetJoint(hand_rot, 9, middle_mcp_r) ||
        !GetJoint(hand_rot, 10, middle_pip_r) || !GetJoint(hand_rot, 13, ring_mcp_r) || !GetJoint(hand_rot, 14, ring_pip_r) ||
        !GetJoint(hand_rot, 17, pinky_mcp_r) || !GetJoint(hand_rot, 18, pinky_pip_r)) {
        return false;
    }

    auto rot_mag = [](const Vec3& r) {
        return std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
    };

    std::vector<glove_thumb::EulerDeg> rotations;
    rotations.reserve(21);
    for (std::size_t joint_index = 0; joint_index < 21; ++joint_index) {
        const std::size_t base = joint_index * 3;
        rotations.push_back({hand_rot[base], hand_rot[base + 1], hand_rot[base + 2]});
    }
    g_last_thumb_mapping_debug = g_glove_thumb_mapper.Map(rotations);
    // Missing or invalid calibration fails safe for the two thumb motors only.
    out_targets[0] = g_last_thumb_mapping_debug.valid ? g_last_thumb_mapping_debug.yaw_target : 2000;
    out_targets[1] = g_last_thumb_mapping_debug.valid ? g_last_thumb_mapping_debug.pitch_target : 2000;

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
    const int MAX_STEP_PER_PACKET = 100;
    for (int i = 0; i < 4; ++i) {
        uint16_t* t = &out_targets[2 + i];
        int diff = (int)(*t) - (int)last_finger_targets[i];
        if (diff > MAX_STEP_PER_PACKET) *t = (uint16_t)(last_finger_targets[i] + MAX_STEP_PER_PACKET);
        if (diff < -MAX_STEP_PER_PACKET) *t = (uint16_t)(last_finger_targets[i] - MAX_STEP_PER_PACKET);
        last_finger_targets[i] = *t;
    }

    return true;
}

static bool ParseUnityUdpPacket(const char* data, int len, uint16_t out_targets[6]) {
    if (data == nullptr || len <= 0) return false;

    try {
        std::string payload(data, data + len);
        json j = json::parse(payload);

        bool handled_control = false;
        if (ParseH5ControlPacket(j, out_targets, handled_control)) {
            return true;
        }
        if (handled_control) {
            return false;
        }

        const bool is_vr_packet = JsonStringEquals(j, "source", "xr-hands") ||
                                  JsonStringContains(j, "jointFormat", "unity-xr-hands");
        const ControlMode active_mode = (ControlMode)g_control_mode.load();
        if (active_mode == ControlMode::Vr) {
            return is_vr_packet ? MapVrHandTrackingToTargets(j, out_targets) : false;
        }
        if (active_mode == ControlMode::Glove) {
            return is_vr_packet ? false : MapUnityToTargets(j, out_targets);
        }
        return false;
    } catch (...) {
        return false;
    }
}

void ComplianceControlLoop() {
    constexpr long long kLogPeriodMs = 20;  // 50 Hz
    uint16_t current_pos[6] = {0};
    uint16_t current_amp[6] = {0};
    uint16_t final_cmd[6];
    uint16_t desired_pos[6];
    uint16_t shaped_desired[6];
    uint16_t last_valid_pos[6] = {0};
    bool position_feedback_valid[6] = {false};
    bool local_soft_release_active[6] = {false};

    SecondOrderState states[6];
    double desired_filters[6] = {0.0};
    double filtered_amp[6] = {0.0};

    std::ofstream log_file;
    if (IsCsvLoggingEnabled()) {
        log_file.open(GetLogPath("WA100_CSV_LOG_PATH", "admittance_log.csv"));
    }
    if (log_file.is_open()) {
        log_file << "TIME,WALL_TIME_MS,ID,RAW_DESIRED,SHAPED_DESIRED,CMD,REAL,AMP,OFFSET\n";
        std::cout << ">>> 日志文件已创建: admittance_log.csv" << std::endl;
    } else if (IsCsvLoggingEnabled()) {
        std::cerr << ">>> 警告: 无法创建日志文件" << std::endl;
    }

    std::cout << ">>> 柔顺控制线程已启动 (Unity输入, 阈值: " << FORCE_THRESHOLD << "mA)" << std::endl;

    auto last_freq_time = std::chrono::high_resolution_clock::now();
    long long loop_counter_freq = 0;
    auto last_step_time = std::chrono::high_resolution_clock::now();
    auto log_start_time = std::chrono::steady_clock::now();
    auto next_log_time = log_start_time;
    long long log_sample_index = 0;
    long long log_wall_time_start_ms = GetCurrentTimeMs();
    auto next_telemetry_time = log_start_time;
    std::uint64_t telemetry_sequence = 0;
    SOCKET telemetry_socket = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in telemetry_address{};
    telemetry_address.sin_family = AF_INET;
    telemetry_address.sin_port = htons(RECORDING_TELEMETRY_PORT);
    telemetry_address.sin_addr.s_addr = inet_addr(RECORDING_TELEMETRY_IP);

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

        const bool feedback_valid = g_fast_driver.SyncWriteRead(final_cmd, current_pos, current_amp);
        if (!feedback_valid) {
            // Ignore transient communication failures.
        }

        for (int i = 0; i < 6; i++) {
            // Filter out clearly invalid position feedback such as 0xFFFF.
            // Small overshoot around 2000 is still considered valid.
            position_feedback_valid[i] = feedback_valid && current_pos[i] <= INVALID_POSITION_THRESHOLD;
            if (position_feedback_valid[i]) {
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

            local_soft_release_active[i] =
                current_amp[i] > SOFT_CURRENT_RELEASE_MA || filtered_amp[i] > SOFT_CURRENT_RELEASE_MA;
            if (local_soft_release_active[i]) {
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

        if (log_file.is_open()) {
            auto now_log = std::chrono::steady_clock::now();
            while (now_log >= next_log_time) {
                long long sample_time_ms = log_sample_index * kLogPeriodMs;
                long long wall_time_ms = log_wall_time_start_ms + sample_time_ms;
                for (int i = 0; i < 6; i++) {
                    log_file << sample_time_ms << "," << wall_time_ms << ",ID" << (i + 1) << ","
                             << desired_pos[i] << "," << shaped_desired[i] << "," << final_cmd[i] << ","
                             << current_pos[i] << "," << (int)std::lround(filtered_amp[i]) << ","
                             << (int)std::lround(states[i].offset) << "\n";
                }
                ++log_sample_index;
                next_log_time += std::chrono::milliseconds(kLogPeriodMs);
            }
        }

        loop_counter_freq++;
        auto now_freq = std::chrono::high_resolution_clock::now();
        double elapsed_freq = std::chrono::duration<double>(now_freq - last_freq_time).count();
        if (elapsed_freq >= 2.0) {
            loop_counter_freq = 0;
            last_freq_time = now_freq;
        }

        auto telemetry_now = std::chrono::steady_clock::now();
        if (telemetry_now >= next_telemetry_time && telemetry_socket != INVALID_SOCKET) {
            json telemetry;
            telemetry["type"] = "wa100_recording_state";
            telemetry["schema_version"] = 2;
            telemetry["seq"] = telemetry_sequence++;
            telemetry["source_time_ns"] = GetCurrentTimeNs();
            telemetry["source_monotonic_ns"] = GetSteadyTimeNs();
            telemetry["control_mode"] = ControlModeToString((ControlMode)g_control_mode.load());
            telemetry["feedback_valid"] = feedback_valid;
            telemetry["position_valid"] = std::vector<bool>(position_feedback_valid, position_feedback_valid + 6);
            telemetry["actual_position_units"] = std::vector<uint16_t>(current_pos, current_pos + 6);
            telemetry["target_position_units"] = std::vector<uint16_t>(desired_pos, desired_pos + 6);
            telemetry["command_position_units"] = std::vector<uint16_t>(final_cmd, final_cmd + 6);
            telemetry["raw_current_ma"] = std::vector<uint16_t>(current_amp, current_amp + 6);
            telemetry["filtered_current_ma"] = std::vector<double>(filtered_amp, filtered_amp + 6);
            std::vector<double> offsets(6);
            for (int index = 0; index < 6; ++index) offsets[index] = states[index].offset;
            telemetry["compliance_offset_units"] = offsets;
            telemetry["local_soft_release_active"] =
                std::vector<bool>(local_soft_release_active, local_soft_release_active + 6);
            telemetry["local_protection"] = {
                {"hard_current_limit_ma", HARD_CURRENT_LIMIT_MA},
                {"soft_current_warn_ma", SOFT_CURRENT_WARN_MA},
                {"soft_current_release_ma", SOFT_CURRENT_RELEASE_MA},
                {"one_way_unloading_preserved", true},
            };
            telemetry["current_semantics"] = "motor_current_load_proxy_not_force_or_tactile";
            const std::string encoded = telemetry.dump();
            sendto(
                telemetry_socket, encoded.data(), static_cast<int>(encoded.size()), 0,
                reinterpret_cast<const sockaddr*>(&telemetry_address), sizeof(telemetry_address));
            next_telemetry_time += std::chrono::milliseconds(kLogPeriodMs);
            if (telemetry_now - next_telemetry_time > std::chrono::milliseconds(100)) {
                next_telemetry_time = telemetry_now + std::chrono::milliseconds(kLogPeriodMs);
            }
        }
    }
    if (telemetry_socket != INVALID_SOCKET) closesocket(telemetry_socket);
}

int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    std::cout << ">>> 准备连接灵巧手..." << std::endl;

    std::vector<SerialSettings> serial_candidates = LoadSerialSettingsList();
    SerialSettings serial_settings = serial_candidates.front();
    bool explicit_serial_override = false;
    const char* calibration_env = std::getenv("WA100_THUMB_CALIBRATION_PATH");
    g_thumb_calibration_path = calibration_env != nullptr && calibration_env[0] != '\0'
        ? std::string(calibration_env)
        : ExecutableDirectory(argc > 0 ? argv[0] : nullptr) + "\\thumb_glove_calibration.json";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        UnityHandSide parsed_side;
        if (ParseHandSideArg(arg.c_str(), parsed_side)) {
            g_unity_hand_side = parsed_side;
            continue;
        }
        if (arg == "--port" && i + 1 < argc) {
            serial_settings.port = argv[++i];
            explicit_serial_override = true;
            continue;
        }
        if (arg.rfind("--port=", 0) == 0) {
            serial_settings.port = arg.substr(7);
            explicit_serial_override = true;
            continue;
        }
        if (arg == "--baud" && i + 1 < argc) {
            if (!ParseBaudrate(argv[++i], serial_settings.baudrate)) {
                std::cerr << "Invalid baudrate" << std::endl;
                return -1;
            }
            explicit_serial_override = true;
            continue;
        }
        if (arg.rfind("--baud=", 0) == 0) {
            if (!ParseBaudrate(arg.substr(7), serial_settings.baudrate)) {
                std::cerr << "Invalid baudrate" << std::endl;
                return -1;
            }
            explicit_serial_override = true;
            continue;
        }
        if (arg == "--thumb-calibration" && i + 1 < argc) {
            g_thumb_calibration_path = argv[++i];
            continue;
        }
        if (arg.rfind("--thumb-calibration=", 0) == 0) {
            g_thumb_calibration_path = arg.substr(20);
            continue;
        }
        if (LooksLikeComPort(arg)) {
            serial_settings.port = arg;
            explicit_serial_override = true;
            continue;
        }

        std::cerr << "Usage: udp_receiver_unity [left|right] [COMx] [--port COMx] [--baud 1000000] "
                     "[--thumb-calibration path.json]" << std::endl;
        return -1;
    }

    std::string thumb_calibration_error;
    if (LoadGloveThumbCalibration(g_thumb_calibration_path, thumb_calibration_error)) {
        std::cout << ">>> Glove thumb calibration: " << g_thumb_calibration_path
                  << " (" << g_glove_thumb_mapper.calibration().calibration_id << ")" << std::endl;
    } else {
        std::cerr << ">>> WARNING: glove thumb calibration inactive: " << thumb_calibration_error << std::endl;
        std::cerr << ">>> Thumb yaw/pitch will stay at safe-open 2000; index-to-pinky mapping remains active." << std::endl;
    }

    std::vector<std::string> available_ports = EnumerateAvailableComPorts();
    if (explicit_serial_override) {
        serial_candidates.clear();
        serial_candidates.push_back(serial_settings);
    } else if (available_ports.size() == 1) {
        bool configured_port_is_available = false;
        for (const auto& candidate : serial_candidates) {
            if (SameSerialPort(candidate.port, available_ports[0])) {
                configured_port_is_available = true;
                break;
            }
        }
        if (!configured_port_is_available) {
            SerialSettings fallback = serial_candidates.front();
            fallback.port = available_ports[0];
            AddSerialCandidate(serial_candidates, fallback);
        }
    }

    std::cout << ">>> Unity hand source: " << HandSideToString(g_unity_hand_side) << std::endl;
    std::cout << ">>> Serial candidates:";
    for (const auto& candidate : serial_candidates) {
        std::cout << " " << candidate.port << "@" << candidate.baudrate;
    }
    std::cout << std::endl;

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
              << "Rotations (glove mode does not use Positions)" << std::endl;
    std::cout << ">>> H5 control: JSON {type:h5_hand_control, mode:vr|glove|preset, positions:[6]} on same UDP port" << std::endl;
    std::cout << ">>> initial hand control mode: "
              << ControlModeToString((ControlMode)g_control_mode.load()) << std::endl;
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

    bool serial_connected = false;
    for (const auto& candidate : serial_candidates) {
        std::cout << ">>> Trying serial port: " << candidate.port << " @ " << candidate.baudrate << std::endl;
        if (g_fast_driver.Connect(candidate.port, candidate.baudrate)) {
            serial_settings = candidate;
            serial_connected = true;
            break;
        }
    }
    if (!serial_connected) {
        std::cerr << ">>> 严重错误: 无法打开配置的串口!" << std::endl;
        return -1;
    }
    std::cout << ">>> Serial connected: " << serial_settings.port << " @ " << serial_settings.baudrate << std::endl;

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
    std::ofstream unity_input_log;
    if (IsUnityInputLoggingEnabled()) {
        const std::string input_log_path = GetLogPath("WA100_UNITY_INPUT_LOG_PATH", "unity_input_log.jsonl");
        unity_input_log.open(input_log_path, std::ios::out | std::ios::app);
        if (unity_input_log.is_open()) {
            std::cout << ">>> Unity input log: " << input_log_path << std::endl;
        } else {
            std::cerr << ">>> Warning: failed to create Unity input log: " << input_log_path << std::endl;
        }
    }
    while (true) {
        int len = recvfrom(server_fd, buffer, BUFFER_SIZE, 0, (struct sockaddr*)&client_addr, &client_len);
        if (len > 0) {
            bool got_valid = false;
            int drained_packets = 0;
            auto process_packet = [&](const char* packet, int packet_len) {
                const bool accepted = ParseUnityUdpPacket(packet, packet_len, temp_pos);
                if (unity_input_log.is_open()) {
                    json record;
                    record["recv_time_ns"] = GetCurrentTimeNs();
                    record["accepted"] = accepted;
                    if (accepted) {
                        record["mapped_target_units"] = std::vector<uint16_t>(temp_pos, temp_pos + 6);
                    }
                    try {
                        json parsed_payload = json::parse(std::string(packet, packet + packet_len));
                        record["unity_payload"] = parsed_payload;
                        const char* rotation_key = g_unity_hand_side == UnityHandSide::Left
                            ? "leftRotations" : "rightRotations";
                        if (accepted && parsed_payload.contains(rotation_key) &&
                            (ControlMode)g_control_mode.load() == ControlMode::Glove) {
                            record["thumb_mapping_debug"] = ThumbMappingDebugToJson(g_last_thumb_mapping_debug);
                        }
                    } catch (...) {
                        record["raw_payload"] = std::string(packet, packet + packet_len);
                    }
                    unity_input_log << record.dump() << '\n';
                    unity_input_log.flush();
                }
                if (accepted) {
                    memcpy(latest_valid_pos, temp_pos, sizeof(temp_pos));
                    got_valid = true;
                }
            };

            process_packet(buffer, len);

            // Drain only a bounded number of packets. H5 control packets must be
            // parsed as they appear; otherwise a following Unity frame can hide a
            // single click mode switch.
            while (drained_packets < MAX_UDP_DRAIN_PER_TICK) {
                int next_len = recvfrom(server_fd, buffer, BUFFER_SIZE, 0, NULL, NULL);
                if (next_len > 0) {
                    process_packet(buffer, next_len);
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
