///
/// @file test_plant_closed_loop.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Closed-loop PID tests against the physical plant in
/// hinge_plant.hxx: a motorized hinge joint with viscous damping and an
/// optional constant load torque, whose angle (or angular rate) is driven by
/// the controller. Assertions are property based (convergence, overshoot
/// bounds, disturbance rejection) rather than golden waveforms, so they
/// tolerate solver and gain changes.
/// @version 0.2
/// @date 2026-09-16
/// SPDX-License-Identifier: MIT
/// @copyright Copyright (c) 2025
///

#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

#include "hinge_plant.hxx"
#include "pid_controller.hxx"

using namespace ba7lya::pid;
using namespace std::chrono;
using test_plant::hinge_plant;

namespace {

/// Simulation step used by every closed-loop test (seconds).
constexpr double kStepTime = 1e-3;

/// Angular viscous damping of the joint, chosen low enough that a pure PD
/// loop leaves a measurable steady-state offset under load.
constexpr double kDamping = 0.01;

/// Control sample interval matching the simulation step.
auto step_interval() { return duration<double> { kStepTime }; }

/// Convert a step index to elapsed simulated time (seconds).
double time_at(const int step) { return step * kStepTime; }

///
/// @brief Step-response measurements collected while chasing a set point.
///
struct step_response {
    double final_error = 0.0; ///< |pv - target| at the last step
    double overshoot = 0.0;   ///< peak excursion past the target (fraction of target)
    double settle_time = std::numeric_limits<double>::infinity(); ///< time of the
    ///< last exit from the +-tol band (s); infinity if still outside it at the end
    double max_abs_control = 0.0; ///< largest |u| seen
};

///
/// @brief Angle step response on a fresh plant.
/// @param pid Active controller (limits/modes configured by the caller).
/// @param target Set point (rad).
/// @param seconds Simulated duration.
/// @param tol Settle band (rad).
///
step_response track_angle(
    pid_controller<double>& pid,
    const double target,
    const double seconds,
    const double tol = 0.02
) {
    hinge_plant plant(0.0, kDamping);
    step_response r;
    const int total = static_cast<int>(seconds / kStepTime);
    bool in_band = false;

    for (int i = 0; i < total; ++i) {
        const double u = pid.compute(target, plant.angle(), step_interval());
        plant.step(u, kStepTime);
        const double x = plant.angle();

        r.final_error = std::abs(x - target);
        r.max_abs_control = std::max(r.max_abs_control, std::abs(u));
        r.overshoot = std::max(r.overshoot, (x - target) / std::abs(target));

        // Settling time = the last moment spent outside the band; a run that
        // is still outside it at the end never settled.
        in_band = std::abs(x - target) <= tol;
        if (!in_band) { r.settle_time = time_at(i); }
    }
    if (!in_band) { r.settle_time = std::numeric_limits<double>::infinity(); }
    return r;
}

} // namespace

TEST(plant_closed_loop_test, plant_inertia_matches_box_formula) {
    hinge_plant plant;
    // Box 1 x 0.1 x 0.1, mass 1: Izz = m*(lx^2 + ly^2)/12 = (1 + 0.01)/12.
    EXPECT_NEAR(plant.inertia_zz(), (1.0 + 0.01) / 12.0, 1e-4);
}

TEST(plant_closed_loop_test, hinge_angle_step_tracking) {
    // Plant inertia ~0.084 kg*m^2, torque limit +-2 N*m -> |accel| <= ~24
    // rad/s^2. Gains place a well-damped ~20 rad/s pole:
    // kp = J*wn^2 ~ 34, kd = 2*J*zeta*wn ~ 3.4, with a light ki.
    pid_controller<double> pid(34.0, 10.0, 2.0);
    pid.set_output_limits(-2.0, 2.0);

    const auto r = track_angle(pid, /*target=*/1.0, /*seconds=*/5.0, /*tol=*/0.02);
    EXPECT_TRUE(std::isfinite(r.settle_time)) << "never reached the settle band";
    EXPECT_LT(r.settle_time, 2.5);
    EXPECT_LT(r.overshoot, 0.15) << "overshoot exceeds 15%";
    EXPECT_LT(r.final_error, 0.02);
    EXPECT_LE(r.max_abs_control, 2.0 + 1e-9); // command honored its saturation
}

TEST(plant_closed_loop_test, integral_action_removes_steady_error) {
    constexpr double target = 1.0;
    constexpr double load = 0.2; // N*m persistent load

    // Pure PD against a constant load: the proportional term must hold the
    // load, so the loop settles with a visible offset e_ss ~ load/kp.
    pid_controller<double> pd(34.0, 0.0, 3.4);
    pd.set_output_limits(-2.0, 2.0);
    {
        hinge_plant plant(load, kDamping);
        for (int i = 0; i < 8000; ++i) {
            plant.step(pd.compute(target, plant.angle(), step_interval()), kStepTime);
        }
        EXPECT_NEAR(std::abs(plant.angle() - target), load / 34.0, 0.002); // ~0.0059
    }

    // Adding the integral term drives the residual to zero. The loop needs a
    // few extra seconds of integration to bleed off the load error fully.
    pid_controller<double> pid(34.0, 10.0, 2.0);
    pid.set_output_limits(-2.0, 2.0);
    {
        hinge_plant plant(load, kDamping);
        for (int i = 0; i < 20000; ++i) {
            plant.step(pid.compute(target, plant.angle(), step_interval()), kStepTime);
        }
        EXPECT_NEAR(plant.angle(), target, 1e-3);
    }
}

TEST(plant_closed_loop_test, disturbance_rejection_constant_load) {
    // A heavier load than the one above: the integral absorbs it and the
    // angle still converges onto the set point.
    pid_controller<double> pid(34.0, 10.0, 2.0);
    pid.set_output_limits(-2.0, 2.0);

    hinge_plant plant(/*load_torque=*/0.5, kDamping);
    for (int i = 0; i < 8000; ++i) {
        plant.step(pid.compute(1.0, plant.angle(), step_interval()), kStepTime);
    }

    EXPECT_NEAR(plant.angle(), 1.0, 0.02);
    EXPECT_TRUE(std::isfinite(pid.integral()));
    EXPECT_TRUE(std::isfinite(pid.output()));
}

TEST(plant_closed_loop_test, saturated_actuator_reversal_no_windup) {
    // Chase +-2 rad with the command limited to +-0.5 N*m (deep saturation),
    // then flip the set point at 6 s. A wound-up integral keeps driving the
    // wrong way after the flip and delays (or prevents) recovery; clamping
    // prevents that.
    const auto settle_after_reversal = [](const anti_windup mode)
    {
        pid_controller<double> pid(30.0, 8.0, 3.0);
        pid.set_output_limits(-0.5, 0.5);
        pid.set_anti_windup(mode);

        hinge_plant plant(0.0, kDamping);
        const int flip = static_cast<int>(6.0 / kStepTime);
        const int total = static_cast<int>(20.0 / kStepTime);
        const double tol = 0.05;
        double last_exit = std::numeric_limits<double>::infinity();
        bool in_band = false;
        for (int i = 0; i < total; ++i) {
            const double target = (i < flip) ? 2.0 : -2.0;
            plant.step(pid.compute(target, plant.angle(), step_interval()), kStepTime);
            if (i > flip) {
                const bool b = std::abs(plant.angle() - target) <= tol;
                if (!b) { last_exit = time_at(i) - time_at(flip); }
                in_band = b;
            }
        }
        return in_band ? last_exit : std::numeric_limits<double>::infinity();
    };

    const double clamped = settle_after_reversal(anti_windup::clamping);
    const double wound = settle_after_reversal(anti_windup::none);

    EXPECT_TRUE(std::isfinite(clamped)) << "clamped run never settled after reversal";
    EXPECT_LT(clamped, 10.0);
    // The wound-up controller recovers no faster than the anti-windup one.
    EXPECT_GE(wound, clamped);
}

TEST(plant_closed_loop_test, velocity_control_derivative_on_measurement) {
    constexpr double target_rate = 3.0;

    // Un-saturated comparison of the derivative input: with
    // derivative-on-measurement the first sample of a rate step sees only the
    // P (+I) effort; with derivative-on-error the same step multiplies the
    // set-point jump by kd/dt and kicks.
    pid_controller<double> on_measurement(0.5, 2.0, 0.05);
    on_measurement.set_derivative_source(derivative_source::measurement);
    hinge_plant plant(0.0, kDamping);
    const double u_first_measurement
        = on_measurement.compute(target_rate, plant.rate(), step_interval());

    pid_controller<double> on_error(0.5, 2.0, 0.05);
    hinge_plant plant2(0.0, kDamping);
    const double u_first_error = on_error.compute(target_rate, plant2.rate(), step_interval());

    EXPECT_LT(std::abs(u_first_measurement), 2.0) << "derivative kick on measurement";
    EXPECT_GT(std::abs(u_first_error), 100.0) << "expected a large set-point kick";

    // The measurement-based loop still regulates speed to the target.
    double max_torque = std::abs(u_first_measurement);
    for (int i = 1; i < 4000; ++i) {
        const double u = on_measurement.compute(target_rate, plant.rate(), step_interval());
        max_torque = std::max(max_torque, std::abs(u));
        plant.step(u, kStepTime);
    }
    EXPECT_NEAR(plant.rate(), target_rate, 0.05);
}
