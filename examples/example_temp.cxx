///
/// @file example_temp.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief PID control of the thermal plant from plant_models.hxx,
/// demonstrating heater power saturation and a mid-run set point change.
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

    constexpr double step_time = 0.1;
    const duration<double> dt { step_time };

    // Heater command is a power percentage: only the upper half makes sense,
    // and the integral needs its own limit to recover quickly when the set
    // point is lowered.
    ba7lya::pid::pid_controller<double> pid(2.0, 0.5, 1.0);
    pid.set_output_limits(0.0, 100.0);
    pid.set_integral_limits(0.0, 200.0);

    temperature_system system;

    double target = 80.0; // Target temperature (deg C)
    constexpr int steps = 2000;

    std::cout << "time,temp,control\n";
    for (int phase = 0; phase < 2; ++phase) {
        if (phase == 1) {
            // Drop the set point: with a wound-up integral the controller
            // would keep heating for a long time; clamping prevents this.
            target = 60.0;
            std::cout << "# target changed to " << target << " deg C\n";
        }
        for (int i = 0; i < steps; ++i) {
            const double t = (phase * steps + i) * step_time;
            double current_temp = system.get_temperature();
            double control = pid.compute(target, current_temp, dt);
            system.update(control, step_time);

            if (i % 20 == 0) { std::cout << t << "," << current_temp << "," << control << "\n"; }
        }
    }

    return 0;
}
