#include "wa100_channel_layout.h"

#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace {

void Require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << std::endl;
        std::exit(1);
    }
}

}  // namespace

int main() {
    Require(wa100_channel_layout::kThumbPitchIndex == 0, "hardware ID1 must be thumb pitch");
    Require(wa100_channel_layout::kThumbYawIndex == 1, "hardware ID2 must be thumb yaw");
    Require(wa100_channel_layout::kFingerCloseEngageTargets[0] == 1905, "ID3 compensation must include +30 tuning");
    Require(wa100_channel_layout::kFingerCloseEngageTargets[1] == 1896, "ID4 compensation must include +30 tuning");
    Require(wa100_channel_layout::kFingerCloseEngageTargets[2] == 1856, "ID5 compensation must include +30 tuning");
    Require(wa100_channel_layout::kFingerCloseEngageTargets[3] == 1655, "ID6 compensation must be 1665 - 10");

    std::uint16_t targets[wa100_channel_layout::kChannelCount] = {
        2000, 2000, 2000, 2000, 2000, 2000,
    };
    wa100_channel_layout::SetThumbTargets(321, 654, targets);
    Require(targets[0] == 321, "pitch target must be packed into ID1");
    Require(targets[1] == 654, "yaw target must be packed into ID2");
    Require(targets[2] == 2000, "packing thumb targets must not modify finger channels");

    std::uint16_t compensated[wa100_channel_layout::kChannelCount] = {
        321, 654, 2000, 2000, 2000, 2000,
    };
    wa100_channel_layout::ApplyFourFingerClosingCompensation(compensated);
    Require(compensated[0] == 321, "finger compensation must not modify ID1");
    Require(compensated[1] == 654, "finger compensation must not modify ID2");
    Require(compensated[2] == 2000, "fully open ID3 must remain 2000");
    Require(compensated[3] == 2000, "fully open ID4 must remain 2000");
    Require(compensated[4] == 2000, "fully open ID5 must remain 2000");
    Require(compensated[5] == 2000, "fully open ID6 must remain 2000");

    std::uint16_t first_close[wa100_channel_layout::kChannelCount] = {
        2000, 2000, 1999, 1999, 1999, 1999,
    };
    wa100_channel_layout::ApplyFourFingerClosingCompensation(first_close);
    Require(first_close[2] == 1905, "ID3 first closing command must include +30 tuning");
    Require(first_close[3] == 1896, "ID4 first closing command must include +30 tuning");
    Require(first_close[4] == 1856, "ID5 first closing command must include +30 tuning");
    Require(first_close[5] == 1655, "ID6 first closing command must be 1665 - 10");

    std::uint16_t fully_closed[wa100_channel_layout::kChannelCount] = {
        2000, 2000, 0, 0, 0, 0,
    };
    wa100_channel_layout::ApplyFourFingerClosingCompensation(fully_closed);
    Require(fully_closed[2] == 0, "fully closed ID3 must remain 0");
    Require(fully_closed[3] == 0, "fully closed ID4 must remain 0");
    Require(fully_closed[4] == 0, "fully closed ID5 must remain 0");
    Require(fully_closed[5] == 0, "fully closed ID6 must remain 0");

    std::cout << "wa100_channel_layout_test: PASS" << std::endl;
    return 0;
}
