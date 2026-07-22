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

uint16_t RatioToTarget(double ratio) {
    const double target = 2000.0 * (1.0 - Clamp01(ratio));
    return static_cast<uint16_t>(std::lround(std::max(0.0, std::min(2000.0, target))));
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

double Mapper::MapRatio(
    double score,
    double open_score,
    double close_score,
    double open_deadzone,
    double close_deadzone) {
    double ratio = Clamp01((score - open_score) / (close_score - open_score));
    if (ratio <= open_deadzone) return 0.0;
    if (ratio >= 1.0 - close_deadzone) return 1.0;
    const double middle = Clamp01((ratio - open_deadzone) / (1.0 - open_deadzone - close_deadzone));
    return middle * middle * (3.0 - 2.0 * middle);
}

void Mapper::Reset() {
    calibration_ = Calibration{};
    configured_ = false;
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
    for (std::size_t index = 0; index < kSelectedJointCount; ++index) {
        if (input.joint_indices[index] < 0 || input.joint_indices[index] >= 21) return fail("joint index out of range");
        const Quaternion& q = input.reference_quaternions[index];
        const double qnorm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        if (!Finite(qnorm) || qnorm < 0.5) return fail("invalid reference quaternion");
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

MappingDebug Mapper::Map(const std::vector<EulerDeg>& rotations) const {
    MappingDebug debug;
    debug.configured = configured_;
    debug.calibration_id = configured_ ? calibration_.calibration_id : std::string();
    if (!configured_) {
        debug.error = "thumb calibration is not configured";
        return debug;
    }

    std::array<double, kFeatureDimension> feature{};
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
        const auto log = QuaternionLogDegrees(relative);
        for (std::size_t axis = 0; axis < 3; ++axis) feature[selected * 3 + axis] = log[axis];
    }
    const std::array<double, kFeatureDimension>& residual = feature;
    debug.feature_norm = Norm(feature);
    debug.residual_norm = Norm(residual);
    debug.yaw_projected = Dot(residual, calibration_.yaw_axis);
    debug.pitch_projected = Dot(residual, calibration_.pitch_axis);
    debug.yaw_score = calibration_.demix[0][0] * debug.yaw_projected + calibration_.demix[0][1] * debug.pitch_projected;
    debug.pitch_score = calibration_.demix[1][0] * debug.yaw_projected + calibration_.demix[1][1] * debug.pitch_projected;
    debug.yaw_ratio = MapRatio(
        debug.yaw_score, calibration_.yaw_open_score, calibration_.yaw_close_score,
        calibration_.open_deadzone, calibration_.close_deadzone);
    debug.pitch_ratio = MapRatio(
        debug.pitch_score, calibration_.pitch_open_score, calibration_.pitch_close_score,
        calibration_.open_deadzone, calibration_.close_deadzone);
    debug.yaw_target = RatioToTarget(debug.yaw_ratio);
    debug.pitch_target = RatioToTarget(debug.pitch_ratio);
    debug.valid = true;
    return debug;
}

}  // namespace glove_thumb
