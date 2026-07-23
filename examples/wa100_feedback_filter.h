#pragma once

#include <cstdint>

namespace wa100_feedback {

constexpr std::uint16_t kNormalPositionMaximum = 10000;
constexpr std::uint16_t kZeroWrapMinimum = 65000;
constexpr std::uint16_t kZeroHistoryMaximum = 20;

struct PositionResult {
    std::uint16_t position = 0;
    bool valid = false;
    bool zero_wrap_corrected = false;
};

inline PositionResult NormalizePosition(
    std::uint16_t raw_position,
    std::uint16_t last_valid_position,
    bool feedback_frame_valid) {
    if (!feedback_frame_valid) {
        return {last_valid_position, false, false};
    }
    if (raw_position <= kNormalPositionMaximum) {
        return {raw_position, true, false};
    }
    // The controller occasionally reports a small negative encoder value near
    // the mechanical zero as uint16 (65535, 65534, ...). Only unwrap it when
    // recent measured history confirms that this joint is physically at zero.
    if (raw_position >= kZeroWrapMinimum &&
        last_valid_position <= kZeroHistoryMaximum) {
        return {0, true, true};
    }
    return {last_valid_position, false, false};
}

}  // namespace wa100_feedback
