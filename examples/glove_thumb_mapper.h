#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace glove_thumb {

constexpr std::size_t kSelectedJointCount = 3;
constexpr std::size_t kFeatureDimension = kSelectedJointCount * 3;

struct EulerDeg {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Calibration {
    int schema_version = 2;
    std::string calibration_id;
    std::string hand = "right";
    std::string euler_order = "zxy";
    std::array<int, kSelectedJointCount> joint_indices{{1, 2, 3}};
    std::array<Quaternion, kSelectedJointCount> reference_quaternions{};
    std::array<double, kFeatureDimension> yaw_axis{};
    std::array<double, kFeatureDimension> pitch_axis{};
    std::array<std::array<double, 2>, 2> demix{{{{1.0, 0.0}}, {{0.0, 1.0}}}};
    double yaw_open_score = 0.0;
    double yaw_close_score = 1.0;
    double pitch_open_score = 0.0;
    double pitch_close_score = 1.0;
    double open_deadzone = 0.04;
    double close_deadzone = 0.06;
};

struct MappingDebug {
    bool configured = false;
    bool valid = false;
    std::string calibration_id;
    std::string error;
    double feature_norm = 0.0;
    double residual_norm = 0.0;
    double yaw_projected = 0.0;
    double pitch_projected = 0.0;
    double yaw_score = 0.0;
    double pitch_score = 0.0;
    double yaw_ratio = 0.0;
    double pitch_ratio = 0.0;
    uint16_t yaw_target = 2000;
    uint16_t pitch_target = 2000;
};

class Mapper {
public:
    bool Configure(const Calibration& calibration, std::string* error = nullptr);
    void Reset();
    bool configured() const { return configured_; }
    const Calibration& calibration() const { return calibration_; }

    MappingDebug Map(const std::vector<EulerDeg>& rotations) const;

    static Quaternion EulerToQuaternion(const EulerDeg& value, const std::string& order);
    static std::array<double, 3> QuaternionLogDegrees(const Quaternion& value);

private:
    static double Dot(
        const std::array<double, kFeatureDimension>& left,
        const std::array<double, kFeatureDimension>& right);
    static double Norm(const std::array<double, kFeatureDimension>& value);
    static double MapRatio(double score, double open_score, double close_score, double open_deadzone, double close_deadzone);

    Calibration calibration_;
    bool configured_ = false;
};

}  // namespace glove_thumb
