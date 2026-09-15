///
/// @file example_n_order.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief PID control of a 4th order plant (cascade of four first-order
/// lags) from plant_models.hxx.
/// @version 0.2
/// @date 2026-09-16
/// SPDX-License-Identifier: MIT
/// @copyright Copyright (c) 2025
///

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>

#include "pid_controller.hxx"
#include "plant_models.hxx"
#include "utility.hxx"

int main(int argc, char* argv[]) {
    using namespace std::chrono;

    // Argument count check
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <Kp> <Ki> <Kd> (e.g. 2.0 0.5 0.1)\n";
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

    // 4-th order plant: four lags of 0.1 s each
    constexpr std::size_t order = 4;
    constexpr double step_time = 0.001;
    const duration<double> dt { step_time };

    cascade_system<order> system(/*time_const=*/0.1);

    // Conservative gains for a plant with four cascaded phase lags; a high
    // integral gain destabilizes it, so clamp both the command and the
    // integral.
    ba7lya::pid::pid_controller<double> pid(Kp, Ki, Kd);
    pid.set_output_limits(-50.0, 50.0);
    pid.set_integral_limits(-100.0, 100.0);

    const double target = 10.0;
    constexpr int steps = 10000; // 10 s

    std::cout << "time,output,control\n";
    for (int i = 0; i < steps; ++i) {
        double current = system.state();
        double control = pid.compute(target, current, dt);
        double out = system.update(control, step_time);

        if (i % 100 == 0) { std::cout << i * step_time << "," << out << "," << control << "\n"; }

        // Finish early once the target is close enough
        if (std::abs(target - out) < 0.01) {
            std::cout << "# target arrived at " << i * step_time << " s\n";
            break;
        }
    }

    return 0;
}
