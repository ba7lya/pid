///
/// @file test_pid_core.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Unit tests for the core pid_controller algorithm: each term, dt
/// clamping, saturation, anti-windup strategies, derivative-on-measurement,
/// state access and template instantiation. All tests use the explicit-dt
/// compute() overload and are fully deterministic (the wall-clock tests use a
/// fake clock).
/// @version 0.2
/// @date 2026-09-16
/// SPDX-License-Identifier: MIT
/// @copyright Copyright (c) 2025
///

#include <chrono>
#include <gtest/gtest.h>
#include <type_traits>

#include "pid_controller.hxx"

using namespace ba7lya::pid;
using namespace std::chrono;

namespace {

///
/// @brief Build a sample interval from a number of seconds.
/// @param seconds Interval length.
/// @return The interval as the controller's duration type (duration<double>).
///
auto seconds_interval(double seconds) { return duration<double> { seconds }; }

///
/// @brief Trivial clock whose now() is controlled by the test, so the
/// wall-clock compute() overload can be exercised deterministically.
///
struct fake_clock {
    using duration = duration<double>;
    using rep = double;
    using period = std::ratio<1>;
    using time_point = std::chrono::time_point<fake_clock, duration>;
    static inline time_point current_time { duration { 0.0 } };

    static time_point now() { return current_time; }

    static constexpr bool is_steady = true;

    /// @brief Move the clock forward by the given number of seconds.
    static void advance(double seconds) { current_time += duration { seconds }; }

    /// @brief Return the clock to the epoch.
    static void rewind() { current_time = time_point { duration { 0.0 } }; }
};

} // namespace

// ---------------------------------------------------------------------------
// Construction and template machinery
// ---------------------------------------------------------------------------

TEST(pid_controller_test, ctor_defaults) {
    pid_controller<double> pid(1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(pid.proportional_gain(), 1.0);
    EXPECT_DOUBLE_EQ(pid.integral_gain(), 2.0);
    EXPECT_DOUBLE_EQ(pid.derivative_gain(), 3.0);
    EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
    EXPECT_DOUBLE_EQ(pid.output(), 0.0);
    EXPECT_DOUBLE_EQ(pid.last_error(), 0.0);
    EXPECT_EQ(pid.anti_windup_mode(), anti_windup::clamping);
    EXPECT_EQ(pid.derivative_mode(), derivative_source::error);
}

TEST(pid_controller_test, ctad_and_default_template_arguments) {
    static_assert(
        std::is_same_v<decltype(pid_controller { 1.0, 2.0, 3.0 }), pid_controller<double>>
    );
    static_assert(
        std::is_same_v<decltype(pid_controller { 1.0f, 2.0f, 3.0f }), pid_controller<float>>
    );
    static_assert(std::is_same_v<pid_controller<>, pid_controller<float>>);
    static_assert(
        std::
            is_same_v<typename pid_controller<double>::duration_type, std::chrono::duration<double>>
    );
    SUCCEED();
}

TEST(pid_controller_test, floating_point_types_instantiate) {
    pid_controller<float> pf(2.0f, 0.0f, 0.0f);
    EXPECT_NEAR(pf.compute(10.0f, 5.0f, duration<float> { 0.1f }), 10.0f, 1e-6f);

    pid_controller<double> pd(2.0, 0.0, 0.0);
    EXPECT_NEAR(pd.compute(10.0, 5.0, seconds_interval(0.1)), 10.0, 1e-12);

    pid_controller<long double> pl(2.0L, 0.0L, 0.0L);
    EXPECT_NEAR(pl.compute(10.0L, 5.0L, duration<long double> { 0.1L }), 10.0L, 1e-12L);
}

// ---------------------------------------------------------------------------
// Individual terms
// ---------------------------------------------------------------------------

TEST(pid_controller_test, proportional_only) {
    pid_controller<double> pid(2.5, 0.0, 0.0);
    const auto dt = seconds_interval(0.1);
    EXPECT_DOUBLE_EQ(pid.compute(10.0, 4.0, dt), 15.0);
    EXPECT_DOUBLE_EQ(pid.compute(10.0, 8.0, dt), 5.0);
    EXPECT_DOUBLE_EQ(pid.compute(10.0, 10.0, dt), 0.0);
}

TEST(pid_controller_test, integral_accumulation) {
    // Pure I controller: ten steps of e = 1 at dt = 0.1 with ki = 1
    // integrate to 1.0.
    pid_controller<double> pid(0.0, 1.0, 0.0);
    const auto dt = seconds_interval(0.1);
    double u = 0.0;
    for (int i = 0; i < 10; ++i) { u = pid.compute(1.0, 0.0, dt); }
    EXPECT_NEAR(pid.integral(), 1.0, 1e-9);
    EXPECT_NEAR(u, 1.0, 1e-9);
}

TEST(pid_controller_test, derivative_on_error) {
    // Pure D controller: the error ramps 0 -> 1 over dt = 0.5, kd = 2 -> u = 4.
    pid_controller<double> pid(0.0, 0.0, 2.0);
    pid.compute(1.0, 1.0, seconds_interval(0.5));                  // e = 0
    const double u = pid.compute(2.0, 1.0, seconds_interval(0.5)); // de/dt = 2
    EXPECT_NEAR(u, 4.0, 1e-9);
}

TEST(pid_controller_test, derivative_on_measurement_no_setpoint_kick) {
    pid_controller<double> pid(0.0, 0.0, 2.0);
    pid.set_derivative_source(derivative_source::measurement);
    const auto dt = seconds_interval(0.5);

    pid.compute(1.0, 1.0, dt); // primes the measurement
    // The set point steps by +10 while the measurement is unchanged: with
    // derivative-on-measurement the D term must stay zero (no kick).
    EXPECT_DOUBLE_EQ(pid.compute(11.0, 1.0, dt), 0.0);

    // The measurement then steps by +2: D = -kd * d(measurement)/dt = -8.
    EXPECT_NEAR(pid.compute(11.0, 3.0, dt), -8.0, 1e-9);
}

TEST(pid_controller_test, first_sample_primes_measurement_without_kick) {
    pid_controller<double> pid(0.0, 0.0, 1.0);
    pid.set_derivative_source(derivative_source::measurement);
    // The very first sample primes last_measurement_, so the D term is zero
    // no matter how large the measurement is.
    EXPECT_DOUBLE_EQ(pid.compute(0.0, 100.0, seconds_interval(0.1)), 0.0);
}

TEST(pid_controller_test, combined_pid_hand_computation) {
    // Full three-term check against the documented positional form.
    pid_controller<double> pid(1.0, 2.0, 4.0);
    const auto dt = seconds_interval(0.5);

    pid.compute(1.0, 1.0, dt); // e0 = 0
    const double u = pid.compute(2.0, 1.0, dt);
    // e = 1; integral = 0 + 1*0.5 = 0.5 -> I = 2*0.5 = 1; de/dt = 2 -> D = 8
    EXPECT_NEAR(u, 1.0 + 1.0 + 8.0, 1e-9);
    EXPECT_NEAR(pid.integral(), 0.5, 1e-9);
    EXPECT_DOUBLE_EQ(pid.output(), u);
    EXPECT_DOUBLE_EQ(pid.last_error(), 1.0);
}

// ---------------------------------------------------------------------------
// dt clamping
// ---------------------------------------------------------------------------

TEST(pid_controller_test, dt_clamped_to_minimum) {
    pid_controller<double> pid(0.0, 1.0, 0.0);
    pid.compute(1.0, 0.0, seconds_interval(0.0));  // clamped up to 1e-4
    pid.compute(1.0, 0.0, seconds_interval(-1.0)); // likewise
    EXPECT_NEAR(pid.integral(), 2e-4, 1e-12);
}

TEST(pid_controller_test, dt_clamped_to_maximum) {
    // A 10 s gap must be treated as 1 s: one huge integration step is the
    // failure mode this clamp exists to prevent.
    pid_controller<double> pid(0.0, 1.0, 0.0);
    pid.compute(1.0, 0.0, seconds_interval(10.0));
    EXPECT_NEAR(pid.integral(), 1.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Saturation and anti-windup
// ---------------------------------------------------------------------------

TEST(pid_controller_test, output_saturation) {
    pid_controller<double> pid(100.0, 0.0, 0.0);
    pid.set_output_limits(-5.0, 5.0);
    EXPECT_DOUBLE_EQ(pid.compute(1.0, 0.0, seconds_interval(0.1)), 5.0);
    EXPECT_DOUBLE_EQ(pid.compute(-1.0, 0.0, seconds_interval(0.1)), -5.0);
}

TEST(pid_controller_test, anti_windup_clamping_freezes_integral) {
    pid_controller<double> pid(1.0, 10.0, 0.0);
    pid.set_output_limits(-1.0, 1.0);
    const auto dt = seconds_interval(0.1);

    // e = 2 every sample: u_unsat = kp*e + ki*(integral + e*dt) is already
    // above the +1 limit on the very first sample, so the integral is never
    // committed while the error keeps pushing into the limit.
    for (int i = 0; i < 50; ++i) { EXPECT_DOUBLE_EQ(pid.compute(2.0, 0.0, dt), 1.0); }
    EXPECT_DOUBLE_EQ(pid.integral(), 0.0);

    // The error sign flips: with the frozen (zero) integral the output
    // responds immediately instead of staying pinned at +1 while a wound-up
    // integral unwinds.
    EXPECT_DOUBLE_EQ(pid.compute(-2.0, 0.0, dt), -1.0);
}

TEST(pid_controller_test, anti_windup_none_winds_up) {
    // Regression baseline: with anti_windup::none the integral grows
    // without bound while the output is saturated (pre-0.2 behaviour).
    pid_controller<double> pid(1.0, 10.0, 0.0);
    pid.set_output_limits(-1.0, 1.0);
    pid.set_anti_windup(anti_windup::none);
    const auto dt = seconds_interval(0.1);

    for (int i = 0; i < 50; ++i) { pid.compute(2.0, 0.0, dt); }
    EXPECT_NEAR(pid.integral(), 50 * 2 * 0.1, 1e-6); // 10.0

    // After winding up, the negative error needs ~integral/(e*dt) = 10/0.2 =
    // 50 samples before the output can leave the lower limit.
    int steps_to_unwind = -1;
    for (int i = 0; i < 300; ++i) {
        const double u = pid.compute(-2.0, 0.0, dt);
        if (std::abs(u) < 1.0) { // no longer saturated
            steps_to_unwind = i;
            break;
        }
    }
    EXPECT_GT(steps_to_unwind, 40); // the cost of the accumulated windup
}

TEST(pid_controller_test, anti_windup_back_calculation_bleeds_integral) {
    pid_controller<double> pid(1.0, 10.0, 0.0);
    pid.set_output_limits(-1.0, 1.0);
    pid.set_anti_windup(anti_windup::back_calculation);
    pid.set_tracking_time_constant(5.0);
    const auto dt = seconds_interval(0.1);

    for (int i = 0; i < 50; ++i) { pid.compute(2.0, 0.0, dt); }
    // Fixed point of the bleed: I* = e/kb + (limit - kp*e)/ki - e*dt = 0.1.
    // Without back-calculation the integral would sit near 10.
    EXPECT_NEAR(pid.integral(), 2.0 / 5.0 + (1.0 - 2.0) / 10.0 - 2.0 * 0.1, 0.02);

    // Recovery is immediate because almost nothing accumulated.
    EXPECT_DOUBLE_EQ(pid.compute(-2.0, 0.0, dt), -1.0);
    for (int i = 0; i < 50; ++i) { pid.compute(-2.0, 0.0, dt); }
    EXPECT_NEAR(pid.integral(), -0.1, 0.02); // symmetric in the other direction
}

TEST(pid_controller_test, integral_limits_hard_clamp) {
    pid_controller<double> pid(0.0, 1.0, 0.0);
    pid.set_integral_limits(-0.5, 0.5);
    const auto dt = seconds_interval(0.1);

    for (int i = 0; i < 100; ++i) { pid.compute(1.0, 0.0, dt); }
    EXPECT_DOUBLE_EQ(pid.integral(), 0.5);
    for (int i = 0; i < 100; ++i) { pid.compute(-1.0, 0.0, dt); }
    EXPECT_DOUBLE_EQ(pid.integral(), -0.5);
}

// ---------------------------------------------------------------------------
// State management
// ---------------------------------------------------------------------------

TEST(pid_controller_test, set_gain_keeps_raw_integral) {
    pid_controller<double> pid(0.0, 1.0, 0.0);
    const auto dt = seconds_interval(0.1);
    pid.compute(1.0, 0.0, dt);
    EXPECT_NEAR(pid.integral(), 0.1, 1e-12);

    // Changing ki must not resurrect stale control effort: the stored integral
    // is the raw error integral, ki is applied on use.
    pid.set_gain(0.0, 5.0, 0.0);
    EXPECT_NEAR(pid.compute(1.0, 0.0, dt), 5.0 * 0.2, 1e-12); // ki * (0.1 + 0.1)
    EXPECT_DOUBLE_EQ(pid.proportional_gain(), 0.0);
    EXPECT_DOUBLE_EQ(pid.integral_gain(), 5.0);
}

TEST(pid_controller_test, reset_equivalent_to_fresh_controller) {
    auto drive = [](pid_controller<double>& pid)
    {
        const auto dt = seconds_interval(0.1);
        pid.compute(5.0, 2.0, dt);
        pid.compute(5.0, 3.0, dt);
        return pid.compute(1.0, 2.0, dt);
    };

    pid_controller<double> used(2.0, 1.0, 1.0);
    pid_controller<double> fresh(2.0, 1.0, 1.0);
    drive(used);
    used.reset();
    EXPECT_DOUBLE_EQ(used.integral(), 0.0);
    EXPECT_DOUBLE_EQ(used.output(), 0.0);
    EXPECT_DOUBLE_EQ(used.last_error(), 0.0);
    EXPECT_NEAR(drive(used), drive(fresh), 1e-12);
}

TEST(pid_controller_test, reset_integral_clears_only_integral) {
    pid_controller<double> pid(1.0, 1.0, 0.0);
    const auto dt = seconds_interval(0.1);
    pid.compute(2.0, 0.0, dt);
    EXPECT_DOUBLE_EQ(pid.integral(), 0.2);
    EXPECT_DOUBLE_EQ(pid.last_error(), 2.0);

    pid.reset_integral();
    EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
    EXPECT_DOUBLE_EQ(pid.last_error(), 2.0);
    // P and I are recomputed from a zeroed integral; the retained error state
    // is untouched (ki = 1 here, e = 2 -> u = 2 + (0 + 0.2)).
    EXPECT_DOUBLE_EQ(pid.compute(2.0, 0.0, dt), 2.0 + 0.2);
}

TEST(pid_controller_test, initial_integral_and_error_are_seeded) {
    pid_controller<double> pid(1.0, 1.0, 1.0, /*initial_integral=*/0.25, /*initial_error=*/1.0);
    EXPECT_DOUBLE_EQ(pid.integral(), 0.25);
    EXPECT_DOUBLE_EQ(pid.last_error(), 1.0);

    // e = 1, previous e = 1 -> D = 0; I = ki * (0.25 + 1*0.1)
    const double u = pid.compute(1.0, 0.0, seconds_interval(0.1));
    EXPECT_NEAR(u, 1.0 + 0.35 + 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Wall-clock overload
// ---------------------------------------------------------------------------

TEST(pid_controller_test, clock_overload_matches_explicit_dt) {
    fake_clock::rewind(); // construction anchors last_time_ to the epoch
    pid_controller<double, fake_clock> clocked(1.0, 1.0, 0.0);
    pid_controller<double> explicit_pid(1.0, 1.0, 0.0);

    for (double t : { 0.1, 0.5, 0.02 }) {
        fake_clock::advance(t);
        const double a = clocked.compute(2.0, 0.5);
        const double b = explicit_pid.compute(2.0, 0.5, seconds_interval(t));
        EXPECT_DOUBLE_EQ(a, b);
    }

    // reset() re-anchors the timestamp, so the next sample interval is ~0
    // (clamped to the minimum), matching a freshly constructed controller.
    clocked.reset();
    pid_controller<double, fake_clock> fresh(1.0, 1.0, 0.0);
    fake_clock::advance(0.3);
    EXPECT_DOUBLE_EQ(clocked.compute(2.0, 0.5), fresh.compute(2.0, 0.5, seconds_interval(0.3)));
}

TEST(pid_controller_test, clock_overload_handles_long_pause) {
    fake_clock::rewind();
    pid_controller<double, fake_clock> pid(0.0, 1.0, 0.0);
    fake_clock::advance(3600.0); // the controller idled for an hour
    pid.compute(1.0, 0.0);
    // The pause may contribute at most the max-step clamp (1 s).
    EXPECT_NEAR(pid.integral(), 1.0, 1e-9);
}
