#include "vr_anchor_stability.h"
#include <iostream>
#include <stdexcept>
static void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
int main() {
    VrAnchorStability state;
    state.update(0.05, .9, .8, 3, 0);
    for (int i = 1; i <= 500; ++i) {
        state.update(.05 + .04 * std::sin(i), .9, i < 10 ? .8 : 1.1 + .15 * std::sin(i), 3, i * .02);
        if (i >= 5) require(state.active == 0, "anchor chatter during 10s hold");
    }
    const double previous = state.ratio;
    state.update(.9, 0, 4, .2, 10.0);
    require(state.ratio == previous && state.active == 0, "duplicate advanced state");
    for (int i = 501; i <= 506; ++i) state.update(.95, .9, 3, .5, i * .02);
    require(state.active == 1, "deliberate anchor switch failed");
    for (int i = 507; i <= 510; ++i) state.update(.5, 0, 3, 3, i * .02);
    require(state.active == -1, "release exceeds 150ms");
    state.update(.3, .2, 3, 3, 11);
    require(state.active == -1 && state.ratio == .3, "tracking gap dragged old state");
    std::cout << "VR anchor hold, switch, release, duplicate and recovery passed\n";
}
