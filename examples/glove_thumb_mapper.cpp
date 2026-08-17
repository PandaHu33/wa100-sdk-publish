#include "glove_thumb_mapper.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace glove_thumb {
namespace {

constexpr double kPi = 3.14159265358979323846;

bool Finite(double value) {
    return std::isfinite(value) != 0;
}

Quaternion NormalizeQuaternion(const Quaternion& value) {
    const double norm = std::sqrt(
        value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z);
    if (norm < 1e-12 || !Finite(norm)) return {};
    Quaternion result{value.w / norm, value.x / norm, value.y / norm, value.z / norm};
    if (result.w < 0.0) {
        result.w = -result.w;
        result.x = -result.x;
        result.y = -result.y;
        result.z = -result.z;
    }
    return result;
}

Quaternion Multiply(const Quaternion& left, const Quaternion& right) {
    return {
        left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
        left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
    };
}

Quaternion Conjugate(const Quaternion& value) {
    return {value.w, -value.x, -value.y, -value.z};
}

Quaternion AxisQuaternion(char axis, double angle_deg) {
    const double half_angle = angle_deg * kPi / 360.0;
    const double sine = std::sin(half_angle);
    Quaternion value{std::cos(half_angle), 0.0, 0.0, 0.0};
    if (axis == 'x') value.x = sine;
    if (axis == 'y') value.y = sine;
    if (axis == 'z') value.z = sine;
    return value;
}

double Clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

bool ValidOrder(const std::string& order) {
    if (order.size() != 3) return false;
    std::string sorted = order;
    std::sort(sorted.begin(), sorted.end());
    return sorted == "xyz";
}

}  // namespace

Quaternion Mapper::EulerToQuaternion(const EulerDeg& value, const std::string& order) {
    Quaternion result;
    for (char axis : order) {
        double angle = axis == 'x' ? value.x : (axis == 'y' ? value.y : value.z);
        result = Multiply(result, AxisQuaternion(axis, angle));
    }
    return NormalizeQuaternion(result);
}

std::array<double, 3> Mapper::QuaternionLogDegrees(const Quaternion& input) {
    const Quaternion value = NormalizeQuaternion(input);
    const double vector_norm = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
    if (vector_norm < 1e-12) return {{0.0, 0.0, 0.0}};
    const double angle_deg = 2.0 * std::atan2(vector_norm, std::max(-1.0, std::min(1.0, value.w))) * 180.0 / kPi;
    const double scale = angle_deg / vector_norm;
    return {{value.x * scale, value.y * scale, value.z * scale}};
}

double Mapper::Dot(
    const std::array<double, kFeatureDimension>& left,
    const std::array<double, kFeatureDimension>& right) {
    double result = 0.0;
    for (std::size_t index = 0; index < kFeatureDimension; ++index) result += left[index] * right[index];
    return result;
}

double Mapper::Norm(const std::array<double, kFeatureDimension>& value) {
    return std::sqrt(std::max(0.0, Dot(value, value)));
}

double Mapper::EulerRotationMagnitudeDegrees(const EulerDeg& value, const std::string& order) {
    const auto rotation_vector = QuaternionLogDegrees(EulerToQuaternion(value, order));
    return std::sqrt(
        rotation_vector[0] * rotation_vector[0] +
        rotation_vector[1] * rotation_vector[1] +
        rotation_vector[2] * rotation_vector[2]);
}

std::array<double, 3> Mapper::UnwrapRotationVectorDegrees(
    const std::array<double, 3>& principal,
    const std::array<double, 3>& previous,
    bool* adjusted) {
    const double angle = std::sqrt(
        principal[0] * principal[0] + principal[1] * principal[1] + principal[2] * principal[2]);
    if (angle < 1e-9) {
        if (adjusted != nullptr) *adjusted = false;
        return principal;
    }
    const std::array<double, 3> axis{{
        principal[0] / angle, principal[1] / angle, principal[2] / angle,
    }};
    std::array<double, 3> best = principal;
    double best_distance_squared = std::numeric_limits<double>::infinity();
    int best_turn = 0;
    for (int turn = -2; turn <= 2; ++turn) {
        std::array<double, 3> candidate{};
        double distance_squared = 0.0;
        for (std::size_t component = 0; component < 3; ++component) {
            candidate[component] = principal[component] + 360.0 * turn * axis[component];
            const double distance = candidate[component] - previous[component];
            distance_squared += distance * distance;
        }
        if (distance_squared < best_distance_squared) {
            best = candidate;
            best_distance_squared = distance_squared;
            best_turn = turn;
        }
    }
    if (adjusted != nullptr) *adjusted = best_turn != 0;
    return best;
}

double Mapper::MapScalarRatio(
    double value,
    double open_value,
    double close_value,
    double open_deadzone,
    double close_deadzone) {
    double ratio = Clamp01((value - open_value) / (close_value - open_value));
    if (ratio <= open_deadzone) return 0.0;
    if (ratio >= 1.0 - close_deadzone) return 1.0;
    const double middle = Clamp01((ratio - open_deadzone) / (1.0 - open_deadzone - close_deadzone));
    return middle * middle * (3.0 - 2.0 * middle);
}

uint16_t Mapper::RatioToTarget(double ratio, uint16_t open_target, uint16_t close_target) {
    const double target = static_cast<double>(open_target) +
        (static_cast<double>(close_target) - static_cast<double>(open_target)) * Clamp01(ratio);
    return static_cast<uint16_t>(std::lround(std::max(0.0, std::min(2000.0, target))));
}

PositionFusionDebug Mapper::FuseTouchPositionAnchors(
    const std::array<double, kTipPositionFeatureDimension>& current,
    const std::array<double, kTipPositionFeatureDimension>& index_touch,
    const std::array<double, kTipPositionFeatureDimension>& middle_touch,
    const std::array<uint16_t, 2>& index_targets,
    const std::array<uint16_t, 2>& middle_targets,
    double support_radius,
    double influence,
    uint16_t rotation_pitch_target,
    uint16_t rotation_yaw_target) {
    PositionFusionDebug debug;
    debug.pitch_target = rotation_pitch_target;
    debug.yaw_target = rotation_yaw_target;
    if (!Finite(support_radius) || support_radius <= 1e-9 ||
        !Finite(influence) || influence < 0.0 || influence > 1.0) {
        return debug;
    }

    auto contact_distances = [](const std::array<double, kTipPositionFeatureDimension>& feature) {
        std::array<double, 2> result{};
        for (std::size_t finger = 0; finger < 2; ++finger) {
            double squared = 0.0;
            const std::size_t other = (finger + 1) * 3;
            for (std::size_t axis = 0; axis < 3; ++axis) {
                const double delta = feature[axis] - feature[other + axis];
                squared += delta * delta;
            }
            result[finger] = std::sqrt(std::max(0.0, squared));
        }
        return result;
    };
    const auto point = contact_distances(current);
    const auto start = contact_distances(index_touch);
    const auto end = contact_distances(middle_touch);
    for (double value : {point[0], point[1], start[0], start[1], end[0], end[1]}) {
        if (!Finite(value)) return debug;
    }
    const double dx = end[0] - start[0];
    const double dy = end[1] - start[1];
    const double length_squared = dx * dx + dy * dy;
    if (length_squared < 1e-9) return debug;

    debug.segment_ratio = Clamp01(
        ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared);
    const double closest_x = start[0] + debug.segment_ratio * dx;
    const double closest_y = start[1] + debug.segment_ratio * dy;
    debug.distance_to_segment = std::hypot(point[0] - closest_x, point[1] - closest_y);
    const double proximity = Clamp01(1.0 - debug.distance_to_segment / support_radius);
    debug.weight = influence * proximity * proximity * (3.0 - 2.0 * proximity);

    auto fuse_channel = [&](std::size_t channel, uint16_t rotation_target) {
        const double position_target =
            static_cast<double>(index_targets[channel]) +
            debug.segment_ratio *
                (static_cast<double>(middle_targets[channel]) - index_targets[channel]);
        return static_cast<uint16_t>(std::lround(
            (1.0 - debug.weight) * rotation_target + debug.weight * position_target));
    };
    debug.pitch_target = fuse_channel(0, rotation_pitch_target);
    debug.yaw_target = fuse_channel(1, rotation_yaw_target);
    debug.valid = true;
    return debug;
}

void Mapper::Reset() {
    calibration_ = Calibration{};
    configured_ = false;
    wrist_rotation_initialized_ = false;
    previous_wrist_rotation_vector_deg_ = {{0.0, 0.0, 0.0}};
    wrist_pose_quaternion_initialized_ = false;
    previous_wrist_pose_quaternion_ = Quaternion{};
}

bool Mapper::Configure(const Calibration& input, std::string* error) {
    Reset();
    auto fail = [&](const std::string& message) {
        if (error != nullptr) *error = message;
        return false;
    };

    if (input.schema_version != 2) return fail("unsupported schema_version");
    if (input.hand != "left" && input.hand != "right") return fail("hand must be left or right");
    if (!ValidOrder(input.euler_order)) return fail("euler_order must contain x, y and z exactly once");
    if (input.calibration_id.empty()) return fail("calibration_id is empty");
    if (!(input.open_deadzone >= 0.0 && input.close_deadzone >= 0.0 &&
          input.open_deadzone + input.close_deadzone < 0.95)) {
        return fail("invalid endpoint deadzones");
    }
    if (input.open_target > 2000 || input.close_target > 2000 || input.open_target == input.close_target) {
        return fail("output targets must be distinct values in 0..2000");
    }
    for (std::size_t index = 0; index < kSelectedJointCount; ++index) {
        if (input.joint_indices[index] < 0 || input.joint_indices[index] >= 21) return fail("joint index out of range");
        const Quaternion& q = input.reference_quaternions[index];
        const double qnorm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        if (!Finite(qnorm) || qnorm < 0.5) return fail("invalid reference quaternion");
    }
    if (input.wrist_compensation_enabled &&
        std::find(input.joint_indices.begin(), input.joint_indices.end(), input.wrist_joint_index) ==
            input.joint_indices.end()) {
        return fail("wrist compensation joint is not present in feature_joint_indices");
    }
    if (!Finite(input.wrist_influence_coefficient) ||
        input.wrist_influence_coefficient < 0.0 ||
        input.wrist_influence_coefficient > 5.0) {
        return fail("wrist influence coefficient must be in 0..5");
    }
    for (double value : input.wrist_influence_terms) {
        if (!Finite(value)) return fail("non-finite wrist influence term");
    }
    for (double value : input.wrist_pose_reference) {
        if (!Finite(value) || std::abs(value) > 1.0) return fail("invalid wrist pose reference");
    }
    for (double value : input.wrist_pitch_influence_terms) {
        if (!Finite(value)) return fail("non-finite wrist pitch influence term");
    }
    if (!Finite(input.yaw_open_score) || !Finite(input.yaw_close_score) ||
        std::abs(input.yaw_close_score - input.yaw_open_score) < 1e-6) {
        return fail("invalid yaw score range");
    }
    if (!Finite(input.pitch_open_score) || !Finite(input.pitch_close_score) ||
        std::abs(input.pitch_close_score - input.pitch_open_score) < 1e-6) {
        return fail("invalid pitch score range");
    }

    calibration_ = input;
    for (Quaternion& reference : calibration_.reference_quaternions) reference = NormalizeQuaternion(reference);

    auto normalize_axis = [&](std::array<double, kFeatureDimension>& axis, const char* name) {
        for (double value : axis) if (!Finite(value)) return fail(std::string("non-finite ") + name);
        const double norm = Norm(axis);
        if (norm < 1e-8) return fail(std::string(name) + " is degenerate");
        for (double& value : axis) value /= norm;
        return true;
    };
    if (!normalize_axis(calibration_.yaw_axis, "yaw_axis")) return false;
    if (!normalize_axis(calibration_.pitch_axis, "pitch_axis")) return false;
    for (const auto& row : calibration_.demix) {
        for (double value : row) if (!Finite(value)) return fail("non-finite demix matrix");
    }

    configured_ = true;
    if (error != nullptr) error->clear();
    return true;
}

MappingDebug Mapper::Map(const std::vector<EulerDeg>& rotations) {
    MappingDebug debug;
    debug.configured = configured_;
    debug.calibration_id = configured_ ? calibration_.calibration_id : std::string();
    if (!configured_) {
        debug.error = "thumb calibration is not configured";
        return debug;
    }

    std::array<double, kFeatureDimension> feature{};
    std::array<double, 3> wrist_pose_vector{{0.0, 0.0, 0.0}};
    for (std::size_t selected = 0; selected < kSelectedJointCount; ++selected) {
        const int joint_index = calibration_.joint_indices[selected];
        if (joint_index < 0 || static_cast<std::size_t>(joint_index) >= rotations.size()) {
            debug.error = "rotation array is shorter than configured joint index";
            return debug;
        }
        const EulerDeg& euler = rotations[static_cast<std::size_t>(joint_index)];
        if (!Finite(euler.x) || !Finite(euler.y) || !Finite(euler.z)) {
            debug.error = "rotation array contains non-finite values";
            return debug;
        }
        const Quaternion current = EulerToQuaternion(euler, calibration_.euler_order);
        const Quaternion relative = NormalizeQuaternion(Multiply(
            Conjugate(calibration_.reference_quaternions[selected]), current));
        auto log = QuaternionLogDegrees(relative);
        if (calibration_.wrist_compensation_enabled &&
            joint_index == calibration_.wrist_joint_index) {
            Quaternion continuous_pose = relative;
            if (wrist_pose_quaternion_initialized_) {
                const double continuity_dot =
                    continuous_pose.w * previous_wrist_pose_quaternion_.w +
                    continuous_pose.x * previous_wrist_pose_quaternion_.x +
                    continuous_pose.y * previous_wrist_pose_quaternion_.y +
                    continuous_pose.z * previous_wrist_pose_quaternion_.z;
                if (continuity_dot < 0.0) {
                    continuous_pose.w = -continuous_pose.w;
                    continuous_pose.x = -continuous_pose.x;
                    continuous_pose.y = -continuous_pose.y;
                    continuous_pose.z = -continuous_pose.z;
                }
            }
            previous_wrist_pose_quaternion_ = continuous_pose;
            wrist_pose_quaternion_initialized_ = true;
            wrist_pose_vector = {{continuous_pose.x, continuous_pose.y, continuous_pose.z}};
            if (wrist_rotation_initialized_) {
                log = UnwrapRotationVectorDegrees(
                    log, previous_wrist_rotation_vector_deg_, &debug.wrist_wrap_adjusted);
            }
            previous_wrist_rotation_vector_deg_ = log;
            wrist_rotation_initialized_ = true;
        }
        for (std::size_t axis = 0; axis < 3; ++axis) feature[selected * 3 + axis] = log[axis];
    }
    const std::array<double, kFeatureDimension>& residual = feature;
    debug.feature_norm = Norm(feature);
    debug.residual_norm = Norm(residual);
    debug.yaw_projected = Dot(residual, calibration_.yaw_axis);
    debug.pitch_projected = Dot(residual, calibration_.pitch_axis);
    debug.yaw_score_uncompensated =
        calibration_.demix[0][0] * debug.yaw_projected +
        calibration_.demix[0][1] * debug.pitch_projected;
    debug.wrist_compensation_enabled = calibration_.wrist_compensation_enabled;
    debug.wrist_influence_coefficient = calibration_.wrist_influence_coefficient;
    debug.yaw_wrist_component = 0.0;
    if (calibration_.wrist_compensation_enabled) {
        for (std::size_t selected = 0; selected < kSelectedJointCount; ++selected) {
            if (calibration_.joint_indices[selected] != calibration_.wrist_joint_index) continue;
            double wrist_yaw_projection = 0.0;
            double wrist_pitch_projection = 0.0;
            double wrist_norm_squared = 0.0;
            for (std::size_t axis = 0; axis < 3; ++axis) {
                const std::size_t feature_index = selected * 3 + axis;
                const double wrist_value = residual[feature_index];
                wrist_yaw_projection += wrist_value * calibration_.yaw_axis[feature_index];
                wrist_pitch_projection += wrist_value * calibration_.pitch_axis[feature_index];
                wrist_norm_squared += wrist_value * wrist_value;
            }
            debug.wrist_rotation_deg = std::sqrt(wrist_norm_squared);
            debug.yaw_wrist_direct_component =
                calibration_.demix[0][0] * wrist_yaw_projection +
                calibration_.demix[0][1] * wrist_pitch_projection;
            debug.pitch_wrist_direct_component =
                calibration_.demix[1][0] * wrist_yaw_projection +
                calibration_.demix[1][1] * wrist_pitch_projection;
            const double x = wrist_pose_vector[0] - calibration_.wrist_pose_reference[0];
            const double y = wrist_pose_vector[1] - calibration_.wrist_pose_reference[1];
            const double z = wrist_pose_vector[2] - calibration_.wrist_pose_reference[2];
            const std::array<double, kWristInfluenceTermCount> pose_terms{{
                x, y, z, x * x, y * y, z * z, x * y, x * z, y * z,
            }};
            for (std::size_t term = 0; term < pose_terms.size(); ++term) {
                debug.yaw_wrist_pose_component +=
                    pose_terms[term] * calibration_.wrist_influence_terms[term];
                debug.pitch_wrist_pose_component +=
                    pose_terms[term] * calibration_.wrist_pitch_influence_terms[term];
            }
            debug.yaw_wrist_component =
                debug.yaw_wrist_direct_component + debug.yaw_wrist_pose_component;
            break;
        }
    }
    debug.yaw_score = debug.yaw_score_uncompensated -
        calibration_.wrist_influence_coefficient * debug.yaw_wrist_component;
    debug.pitch_score =
        calibration_.demix[1][0] * debug.yaw_projected +
        calibration_.demix[1][1] * debug.pitch_projected -
        calibration_.wrist_influence_coefficient *
            (debug.pitch_wrist_direct_component + debug.pitch_wrist_pose_component);
    debug.yaw_ratio = MapScalarRatio(
        debug.yaw_score, calibration_.yaw_open_score, calibration_.yaw_close_score,
        calibration_.open_deadzone, calibration_.close_deadzone);
    debug.pitch_ratio = MapScalarRatio(
        debug.pitch_score, calibration_.pitch_open_score, calibration_.pitch_close_score,
        calibration_.open_deadzone, calibration_.close_deadzone);
    debug.yaw_target = Mapper::RatioToTarget(
        debug.yaw_ratio, calibration_.open_target, calibration_.close_target);
    debug.pitch_target = Mapper::RatioToTarget(
        debug.pitch_ratio, calibration_.open_target, calibration_.close_target);
    debug.valid = true;
    return debug;
}

}  // namespace glove_thumb

