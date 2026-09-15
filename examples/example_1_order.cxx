///
/// @file example_1_order.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief PID position control of a first-order lag plant.
/// @version 0.2
/// @date 2026-09-16
/// SPDX-License-Identifier: MIT
/// @copyright Copyright (c) 2025
///

#include <chrono>
#include <cmath>
#include <iostream>

#include "pid_controller.hxx"
#include "plant_models.hxx"
#include "utility.hxx"

using namespace ba7lya::pid;

int main(int argc, char* argv[]) {
    // Argument count check
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <Kp> <Ki> <Kd> (e.g. 2.0 0.5 0.0)\n";
        return 1;
    }

    // Argument parsing
    auto args = parse_args(argc, argv);
    double Kp = args[0];
    double Ki = args[1];
    double Kd = args[2];

    // Argument range check
    if (Kp < 0 || Kp > 100 || Ki < 0 || Ki > 100 || Kd < 0 || Kd > 100) {
        std::cerr << "error: Kp, Ki, Kd out of range\n";
        return 1;
    }

    // Fixed simulation step; the explicit-dt overload makes the loop
    // deterministic (the wall-clock overload is meant for real-time use).
    constexpr double step_time = 0.01;
    const std::chrono::duration<double> dt { step_time };

    // Create the PID controller. The command is limited to +-20 (plant
    // steady-state value is 5), which keeps the loop out of deep saturation.
    pid_controller<double> pid(Kp, Ki, Kd);
    pid.set_output_limits(-20.0, 20.0);

    // Create the plant
    first_order_system system(/*gain=*/2, /*time_const=*/0.5);
    const double target = 10.0; // Target set point
    const int steps = 3000;     // Simulation steps

    std::cout << "pid controller simulation:\n";
    std::cout << "target: " << target << "\n";

    for (int i = 0; i < steps; ++i) {
        double current = system.state();
        double control = pid.compute(target, current, dt);
        double new_state = system.update(control, step_time);

        if (i % 20 == 0) {
            std::cout << "step," << i << ",current," << new_state << ",control," << control << "\n";
        }

        // Finish early once the target is close enough
        if (std::abs(target - new_state) < 0.001) {
            std::cout << "target arrived\n";
            break;
        }
    }

    return 0;
}
