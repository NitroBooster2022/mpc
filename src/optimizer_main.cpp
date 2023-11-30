#include "optimizer.hpp"

int main() {
    Optimizer controller;
    controller.run();
    controller.park();
    controller.computeStats(0);
    return 0;
}
