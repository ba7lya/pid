///
/// @file example_motor.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Brushed DC motor demos built on the motor plant model: (1) a speed
/// loop contrasting integral windup with anti-windup under a saturated
/// voltage command, and (2) a classical three-loop cascade (position ->
/// velocity -> current) servo chasing a position step.
/// @version 0.2
/// @date 2026-09-16
/// SPDX-License-Identifier: MIT
/// @copyright Copyright (c) 2025
///

#include <chrono>
#include <cmath>
#include <iostream>

#include "motor.hxx"
#include "pid_controller.hxx"

using namespace ba7lya::pid;

namespace {

/// Motor used by both demos: a small armature machine whose electrical time
/// constant L/R is 2 ms and whose mechanical time constant is tens of ms.
dc_motor::params motor_params() {
    dc_motor::params p;
    p.resistance = 1.0;
    p.inductance = 0.002;
    p.torque_constant = 0.02;
    p.back_emf_constant = 0.02;
    p.inertia = 5e-5;
    p.friction = 1e-5;
    p.supply_voltage_max = 24.0;
    return p;
}

///
/// @brief Speed-loop reversal test: how the integral anti-windup choice
/// shapes recovery when the actuator saturates.
///
void speed_loop_windup() {
    using namespace std::chrono;

    constexpr double step_time = 1e-4;
    const duration<double> dt { step_time };

    const auto demo = [&](const char* name, const anti_windup mode)
    {
        dc_motor motor(motor_params());

        // Speed-loop gains for the 24 V, ~1200 rad/s machine.
        pid_controller<double> pid(200.0, 2000.0, 0.5);
        pid.set_output_limits(-24.0, 24.0);
        pid.set_anti_windup(mode);

        double target = 300.0; // rad/s
        constexpr int steps = 8000;
        double peak = 0.0;
        double settle = -1.0;

        for (int phase = 0; phase < 2; ++phase) {
            // Phase 1 accelerates hard into saturation; phase 2 reverses the
            // set point. A wound-up integral must unwind before braking,
            // producing a large overshoot and delayed settling.
            if (phase == 1) {
                target = -300.0;
                peak = 0.0;
                settle = -1.0;
            }
            for (int i = 0; i < steps; ++i) {
                const double u = pid.compute(target, motor.speed(), dt);
                const double omega = motor.update(u, step_time);
                if (phase == 1) {
                    peak = std::min(peak, omega); // most negative excursion
                    if (settle < 0.0 && std::abs(omega - target) < 15.0) { settle = i * step_time; }
                }
            }
        }

        std::cout << name << "overshoot to " << peak << " rad/s, settled " << settle
                  << " s after reversal\n";
    };

    std::cout << "[1] speed loop, step to +300 then -300 rad/s, rail 24 V:\n";
    demo("    anti_windup::none     ", anti_windup::none);
    demo("    anti_windup::clamping ", anti_windup::clamping);
}

///
/// @brief Three-loop servo: position loop commands a velocity, velocity loop
/// commands a current, current loop commands a voltage.
///
void position_cascade() {
    constexpr double step_time = 1e-4; // electrical/inner loop rate
    const auto dt = std::chrono::duration<double> { step_time };

    const auto demo = [&](const char* name, const anti_windup outer_mode)
    {
        dc_motor motor(motor_params());

        // Loop bandwidths separate outward-to-inward as in a real drive:
        // current (fastest) -> velocity -> position (slowest). The velocity
        // command of the outer loop is limited to 2 rad/s, so a +1 rad step
        // saturates it for most of the move - where windup would happen.
        pid_controller<double> position_pid(8.0, 0.5, 1.0);
        position_pid.set_output_limits(-2.0, 2.0);
        position_pid.set_anti_windup(outer_mode);
        pid_controller<double> velocity_pid(0.05, 1.0, 0.0);
        velocity_pid.set_output_limits(-12.0, 12.0); // max armature current (A)
        pid_controller<double> current_pid(25.0, 2000.0, 0.0);
        current_pid.set_output_limits(-48.0, 48.0); // PWM bus equivalent (V)

        const double target = 1.0; // rad
        constexpr int steps = static_cast<int>(4.0 / step_time);

        for (int i = 0; i < steps; ++i) {
            const double velocity_cmd = position_pid.compute(target, motor.angle(), dt);
            const double current_cmd = velocity_pid.compute(velocity_cmd, motor.speed(), dt);
            const double voltage = current_pid.compute(current_cmd, motor.current(), dt);
            motor.update(voltage, step_time);
        }

        std::cout << name << "final position " << motor.angle() << " rad (target " << target
                  << "), speed " << motor.speed() << " rad/s\n";
    };

    std::cout << "\n[2] position/velocity/current cascade, step to +1 rad:\n";
    demo("    position loop, anti_windup::none     ", anti_windup::none);
    demo("    position loop, anti_windup::clamping ", anti_windup::clamping);
}

} // namespace

int main() {
    speed_loop_windup();
    position_cascade();
    return 0;
}
