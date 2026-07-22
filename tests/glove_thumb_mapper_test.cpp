#include "glove_thumb_mapper.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

using glove_thumb::Calibration;
using glove_thumb::EulerDeg;
using glove_thumb::Mapper;

void Require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << std::endl;
        std::exit(1);
    }
}

Calibration BaseCalibration() {
    Calibration value;
    value.calibration_id = "unit-test";
    value.hand = "right";
    value.euler_order = "xyz";
    value.yaw_axis[1] = 1.0;    // relative thumb Y
    value.pitch_axis[8] = 1.0;  // relative thumb IP Z
    value.yaw_open_score = 0.0;
    value.yaw_close_score = 30.0;
    value.pitch_open_score = 0.0;
    value.pitch_close_score = 30.0;
    value.open_deadzone = 0.0;
    value.close_deadzone = 0.0;
    return value;
}

std::vector<EulerDeg> NeutralRotations() {
    return std::vector<EulerDeg>(21);
}

void TestEulerWrapIsContinuous() {
    Calibration calibration = BaseCalibration();
    calibration.yaw_axis.fill(0.0);
    calibration.yaw_axis[0] = 1.0;
    calibration.yaw_open_score = 0.0;
    calibration.yaw_close_score = 180.0;
    Mapper mapper;
    std::string error;
    Require(mapper.Configure(calibration, &error), "wrap calibration: " + error);

    auto first = NeutralRotations();
    auto second = NeutralRotations();
    first[1].x = 179.0;
    second[1].x = -181.0;
    const auto a = mapper.Map(first);
    const auto b = mapper.Map(second);
    Require(a.valid && b.valid, "wrap samples must be valid");
    Require(std::abs(a.yaw_score - b.yaw_score) < 1e-6, "equivalent Euler wraps must map identically");
    Require(a.yaw_target == b.yaw_target, "equivalent Euler wraps must have identical targets");
}

void TestNeutralPoseIsOpen() {
    Mapper mapper;
    std::string error;
    Require(mapper.Configure(BaseCalibration(), &error), "neutral calibration: " + error);
    const auto result = mapper.Map(NeutralRotations());
    Require(result.valid, "neutral sample must be valid");
    Require(result.residual_norm < 1e-6, "neutral sample must have zero relative rotation");
    Require(result.yaw_target == 2000 && result.pitch_target == 2000, "neutral thumb must stay open");
}

void TestYawAndPitchAreIndependent() {
    Mapper mapper;
    std::string error;
    Require(mapper.Configure(BaseCalibration(), &error), "independence calibration: " + error);

    auto yaw = NeutralRotations();
    yaw[1].y = 30.0;
    const auto yaw_result = mapper.Map(yaw);
    Require(yaw_result.yaw_target == 0, "yaw close endpoint must reach zero");
    Require(yaw_result.pitch_target == 2000, "yaw must not move pitch");

    auto pitch = NeutralRotations();
    pitch[3].z = 30.0;
    const auto pitch_result = mapper.Map(pitch);
    Require(pitch_result.pitch_target == 0, "pitch close endpoint must reach zero");
    Require(pitch_result.yaw_target == 2000, "pitch must not move yaw");
}

void TestInvalidCalibrationFailsSafe() {
    Calibration calibration = BaseCalibration();
    calibration.hand = "unknown";
    Mapper mapper;
    std::string error;
    Require(!mapper.Configure(calibration, &error), "invalid hand must be rejected");
    const auto result = mapper.Map(NeutralRotations());
    Require(!result.valid, "unconfigured mapper must be invalid");
    Require(result.yaw_target == 2000 && result.pitch_target == 2000, "unconfigured mapper must fail safe open");
}

}  // namespace

int main() {
    TestEulerWrapIsContinuous();
    TestNeutralPoseIsOpen();
    TestYawAndPitchAreIndependent();
    TestInvalidCalibrationFailsSafe();
    std::cout << "glove_thumb_mapper_test: PASS" << std::endl;
    return 0;
}
