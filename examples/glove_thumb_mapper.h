#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace glove_thumb {

constexpr std::size_t kSelectedJointCount = 3;
constexpr std::size_t kFeatureDimension = kSelectedJointCount * 3;
constexpr std::size_t kWristInfluenceTermCount = 9;
constexpr std::size_t kTipPositionFeatureDimension = 9;

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
    uint16_t open_target = 2000;
    uint16_t close_target = 0;
    bool wrist_compensation_enabled = false;
    int wrist_joint_index = 1;
    double wrist_influence_coefficient = 1.0;
    std::array<double, 3> wrist_pose_reference{};
    std::array<double, kWristInfluenceTermCount> wrist_influence_terms{};
    std::array<double, kWristInfluenceTermCount> wrist_pitch_influence_terms{};
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
    bool wrist_compensation_enabled = false;
    double wrist_influence_coefficient = 1.0;
    double wrist_rotation_deg = 0.0;
    bool wrist_wrap_adjusted = false;
    double yaw_score_uncompensated = 0.0;
    double yaw_wrist_component = 0.0;
    double yaw_wrist_direct_component = 0.0;
    double yaw_wrist_pose_component = 0.0;
    double pitch_wrist_direct_component = 0.0;
    double pitch_wrist_pose_component = 0.0;
    double yaw_score = 0.0;
    double pitch_score = 0.0;
    double yaw_ratio = 0.0;
    double pitch_ratio = 0.0;
    uint16_t yaw_target = 2000;
    uint16_t pitch_target = 2000;
};

struct PositionFusionDebug {
    bool valid = false;
    double segment_ratio = 0.0;
    double distance_to_segment = 0.0;
    double weight = 0.0;
    uint16_t pitch_target = 2000;
    uint16_t yaw_target = 2000;
};

class Mapper {
public:
    bool Configure(const Calibration& calibration, std::string* error = nullptr);
    void Reset();
    bool configured() const { return configured_; }
    const Calibration& calibration() const { return calibration_; }

    MappingDebug Map(const std::vector<EulerDeg>& rotations);

    static Quaternion EulerToQuaternion(const EulerDeg& value, const std::string& order);
    static std::array<double, 3> QuaternionLogDegrees(const Quaternion& value);
    static double EulerRotationMagnitudeDegrees(const EulerDeg& value, const std::string& order);
    static double MapScalarRatio(
        double value,
        double open_value,
        double close_value,
        double open_deadzone,
        double close_deadzone);
    static uint16_t RatioToTarget(double ratio, uint16_t open_target, uint16_t close_target);
    static PositionFusionDebug FuseTouchPositionAnchors(
        const std::array<double, kTipPositionFeatureDimension>& current_tip_positions,
        const std::array<double, kTipPositionFeatureDimension>& index_touch_tip_positions,
        const std::array<double, kTipPositionFeatureDimension>& middle_touch_tip_positions,
        const std::array<uint16_t, 2>& index_touch_thumb_targets,
        const std::array<uint16_t, 2>& middle_touch_thumb_targets,
        double support_radius,
        double influence,
        uint16_t rotation_pitch_target,
        uint16_t rotation_yaw_target);

private:
    static double Dot(
        const std::array<double, kFeatureDimension>& left,
        const std::array<double, kFeatureDimension>& right);
    static double Norm(const std::array<double, kFeatureDimension>& value);
    static std::array<double, 3> UnwrapRotationVectorDegrees(
        const std::array<double, 3>& principal,
        const std::array<double, 3>& previous,
        bool* adjusted);
    Calibration calibration_;
    bool configured_ = false;
    bool wrist_rotation_initialized_ = false;
    std::array<double, 3> previous_wrist_rotation_vector_deg_{{0.0, 0.0, 0.0}};
    bool wrist_pose_quaternion_initialized_ = false;
    Quaternion previous_wrist_pose_quaternion_{};
};

}  // namespace glove_thumb

