#pragma once

#include <cstddef>
#include <cstdint>

namespace wa100_channel_layout {

// Canonical WA100 hardware order. Array index N always addresses hardware
// motor ID N+1; semantic producers must pack their targets in this order.
constexpr std::size_t kThumbPitchIndex = 0;  // ID1: thumb flexion/extension
constexpr std::size_t kThumbYawIndex = 1;    // ID2: thumb opposition/abduction
constexpr std::size_t kIndexFingerIndex = 2;
constexpr std::size_t kMiddleFingerIndex = 3;
constexpr std::size_t kRingFingerIndex = 4;
constexpr std::size_t kPinkyFingerIndex = 5;
constexpr std::size_t kChannelCount = 6;

// Tuned hardware closing engagement targets.
// A logical target of 2000 remains fully open. The first closing command
// enters below the corresponding hardware boundary, then the remaining
// logical range is scaled linearly down to 0.
constexpr std::uint16_t kFullyOpenTarget = 2000;
constexpr std::uint16_t kFingerCloseEngageTargets[4] = {
    1905,  // ID3 index:  tuned from 1875
    1896,  // ID4 middle: tuned from 1866
    1856,  // ID5 ring:   tuned from 1826
    1655,  // ID6 pinky:  measured boundary 1665
};

inline void SetThumbTargets(
    std::uint16_t pitch_target,
    std::uint16_t yaw_target,
    std::uint16_t out_targets[kChannelCount]) {
    out_targets[kThumbPitchIndex] = pitch_target;
    out_targets[kThumbYawIndex] = yaw_target;
}

inline std::uint16_t CompensateFingerClosingTarget(
    std::size_t channel_index,
    std::uint16_t logical_target) {
    if (channel_index < kIndexFingerIndex ||
        channel_index > kPinkyFingerIndex ||
        logical_target >= kFullyOpenTarget) {
        return logical_target;
    }

    const std::uint32_t engage_target =
        kFingerCloseEngageTargets[channel_index - kIndexFingerIndex];
    constexpr std::uint32_t kMaxClosingLogicalTarget = kFullyOpenTarget - 1;
    const std::uint32_t scaled =
        static_cast<std::uint32_t>(logical_target) * engage_target;
    return static_cast<std::uint16_t>(
        (scaled + kMaxClosingLogicalTarget / 2) / kMaxClosingLogicalTarget);
}

inline void ApplyFourFingerClosingCompensation(
    std::uint16_t targets[kChannelCount]) {
    for (std::size_t channel_index = kIndexFingerIndex;
         channel_index <= kPinkyFingerIndex;
         ++channel_index) {
        targets[channel_index] =
            CompensateFingerClosingTarget(channel_index, targets[channel_index]);
    }
}

}  // namespace wa100_channel_layout
