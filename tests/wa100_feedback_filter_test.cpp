#include "wa100_feedback_filter.h"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void Require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << std::endl;
        std::exit(1);
    }
}

void TestNormalPositionPassesThrough() {
    const auto result = wa100_feedback::NormalizePosition(37, 35, true);
    Require(result.valid && result.position == 37, "normal position must pass through");
    Require(!result.zero_wrap_corrected, "normal position must not be marked corrected");
}

void TestZeroUnderflowIsCorrected() {
    for (std::uint16_t raw : {std::uint16_t(65000), std::uint16_t(65534), std::uint16_t(65535)}) {
        const auto result = wa100_feedback::NormalizePosition(raw, 2, true);
        Require(result.valid && result.position == 0, "zero underflow must normalize to zero");
        Require(result.zero_wrap_corrected, "zero underflow must be observable in telemetry");
    }
}

void TestLargeValueAwayFromConfirmedZeroStaysInvalid() {
    const auto history = wa100_feedback::NormalizePosition(65535, 21, true);
    Require(!history.valid && history.position == 21, "nonzero history must not hide invalid feedback");
    const auto corrupt = wa100_feedback::NormalizePosition(12000, 0, true);
    Require(!corrupt.valid, "large non-wrap value must remain invalid");
}

void TestInvalidFrameNeverBecomesValid() {
    const auto result = wa100_feedback::NormalizePosition(0, 4, false);
    Require(!result.valid && result.position == 4, "invalid frame must retain last valid position");
}

}  // namespace

int main() {
    TestNormalPositionPassesThrough();
    TestZeroUnderflowIsCorrected();
    TestLargeValueAwayFromConfirmedZeroStaysInvalid();
    TestInvalidFrameNeverBecomesValid();
    std::cout << "wa100_feedback_filter_test: PASS" << std::endl;
    return 0;
}
