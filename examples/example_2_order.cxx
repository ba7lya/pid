///
/// @file example_2_order.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief PID position control of a mass-spring-damper plant, demonstrating
/// output saturation and integral anti-windup.
/// @version 0.2
/// @date 2026-09-16
/// SPDX-License-Identifier: MIT
/// @copyright Copyright (c) 2025
///

#include <chrono>
#include <iostream>

#include "pid_controller.hxx"
#include "plant_models.hxx"

int main() {
    using namespace std::chrono;

    constexpr double step_time = 0.001;
    const duration<double> dt { step_time };

    // The actuator can only deliver +-2 N; the integral window is wide enough
    // to cover the steady-state force (k*target = 1 N) yet still bounds the
    // windup charge.
    ba7lya::pid::pid_controller<double> pid(30.0, 2.0, 5.0);
    pid.set_output_limits(-2.0, 2.0);
    pid.set_integral_limits(-1.5, 1.5);

    second_order_system system(/*mass=*/1.0, /*damping=*/0.1, /*stiffness=*/1.0);

    double target = 1.0;
    constexpr int steps = 15000; // 15 s

    std::cout << "time,pos,vel,control\n";
    for (int phase = 0; phase < 2; ++phase) {
        // Mid-run set point reversal exercises the anti-windup path after a
        // long saturated acceleration phase.
        if (phase == 1) {
            target = -1.0;
            std::cout << "# set point reversed to " << target << "\n";
        }
        for (int i = 0; i < steps; ++i) {
            auto s = system.get_state();
            double control = pid.compute(target, s.pos, dt);
            s = system.update(control, step_time);

            if (i % 100 == 0) {
                std::cout << (phase * steps + i) * step_time << "," << s.pos << "," << s.vel << ","
                          << control << "\n";
            }
        }
    }

    return 0;
}
