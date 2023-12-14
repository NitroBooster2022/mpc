#include "optimizer.hpp"

int main() {
    Optimizer controller;
    // controller.x_current << controller.state_refs(0, 0), controller.state_refs(0, 1), controller.state_refs(0, 2);
    // controller.run();
    controller.x_current << 0, 0, 0;
    controller.park();
    controller.computeStats(0);
    return 0;
}
