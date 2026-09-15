///
/// @file pid_controller.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief A template-based discrete PID controller with output saturation,
/// integral anti-windup and optional derivative-on-measurement.
/// @version 0.2
/// @date 2026-09-16
/// SPDX-License-Identifier: MIT
/// @copyright Copyright (c) 2025
///

#pragma once

#include <algorithm>
#include <cassert>
#include <chrono>
#include <limits>
#include <type_traits>

namespace ba7lya::pid {

///
/// @brief Integral anti-windup strategy applied while the output is saturated.
///
enum class anti_windup {
    /// Always integrate. The integral may wind up while the output is
    /// saturated (the behaviour of the pre-0.2 controller).
    none,
    /// Conditional integration: freeze the integral while a saturated output
    /// is still pushed further into the limit; unwinding is never blocked.
    clamping,
    /// Back-calculation: bleed the integral away in proportion to the part of
    /// the output that was clipped (gain `kb`, see
    /// pid_controller::set_tracking_time_constant).
    back_calculation,
};

///
/// @brief Signal the derivative term differentiates.
///
enum class derivative_source {
    /// Differentiate the error (textbook form). A step change of the set point
    /// produces a one-shot derivative kick.
    error,
    /// Differentiate the process variable only (negated). The usual choice
    /// when the set point may change abruptly.
    measurement,
};

///
/// @brief Discrete positional PID controller.
///
/// Control law (positional form):
/// @code
/// u(t) = kp * e(t) + ki * integral(e) + kd * d(e)/dt,  e = set_point - process_variable
/// @endcode
///
/// Features:
/// - Sample interval expressed as std::chrono::duration: an explicit-dt
///   overload for deterministic/simulation use, and a wall-clock overload
///   sampled from the template parameter `Clock`.
/// - Configurable output and integral limits (open range by default, i.e. no
///   saturation).
/// - Selectable anti-windup strategy (see ba7lya::pid::anti_windup).
/// - Optional derivative-on-measurement to avoid set-point kicks (see
///   ba7lya::pid::derivative_source).
/// - dt is clamped to [1e-4 s, 1 s]: a stopped or stalled loop can neither
///   divide by (almost) zero nor dump a huge integral on the first sample
///   after a long pause.
///
/// The stored integral is the raw error integral (ki is applied on use), so
/// changing ki with set_gain() never resurrects stale control effort.
///
/// @tparam T     Floating-point type used for gains and states (float, double,
///               long double).
/// @tparam Clock Clock sampled by the two-argument compute() overload.
///
template<typename T = float, typename Clock = std::chrono::high_resolution_clock>
class pid_controller {
public:
    static_assert(std::is_floating_point_v<T>, "pid_controller requires a floating point type");

    using value_type = T;                           ///< Scalar type of gains and states.
    using clock_type = Clock;                       ///< Clock sampled by compute().
    using duration_type = std::chrono::duration<T>; ///< Sample interval type (seconds).

    ///
    /// @brief Construct a controller with the given gains and zeroed state.
    /// @param kp               Proportional gain.
    /// @param ki               Integral gain.
    /// @param kd               Derivative gain.
    /// @param initial_integral Initial value of the raw error integral
    ///                         (NOT multiplied by ki).
    /// @param initial_error    Error value assumed for the previous step, used
    ///                         by the derivative term on the first sample.
    ///
    /// The internal timestamp is anchored to Clock::now(), so the first
    /// two-argument compute() measures elapsed time from construction.
    ///
    pid_controller(
        const T kp,
        const T ki,
        const T kd,
        const T initial_integral = T { 0 },
        const T initial_error = T { 0 }
    )
        : kp_(kp)
        , ki_(ki)
        , kd_(kd)
        , integral_(initial_integral)
        , last_error_value_(initial_error)
        , last_time_(Clock::now()) {}

    ///
    /// @brief Replace the P/I/D gains. Running integral and error state are kept.
    /// @param kp New proportional gain.
    /// @param ki New integral gain.
    /// @param kd New derivative gain.
    ///
    void set_gain(const T kp, const T ki, const T kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    ///
    /// @brief Set the output saturation window.
    /// @param lower Minimum allowed control output.
    /// @param upper Maximum allowed control output.
    /// @note Requires lower <= upper. Defaults to the full range of T.
    ///
    void set_output_limits(const T lower, const T upper) {
        assert(lower <= upper && "output limits must be ordered");
        output_min_ = lower;
        output_max_ = upper;
    }

    ///
    /// @brief Set hard limits on the raw error integral.
    /// @param lower Minimum allowed integral value.
    /// @param upper Maximum allowed integral value.
    /// @note Requires lower <= upper. Defaults to the full range of T.
    ///
    void set_integral_limits(const T lower, const T upper) {
        assert(lower <= upper && "integral limits must be ordered");
        integral_min_ = lower;
        integral_max_ = upper;
    }

    ///
    /// @brief Select the anti-windup strategy (default: clamping).
    /// @param mode Strategy applied when the saturated output differs from the
    ///             unsaturated control effort.
    ///
    void set_anti_windup(const anti_windup mode) { windup_ = mode; }

    ///
    /// @brief Select which signal the derivative term differentiates.
    /// @param source Either the error or the process variable (default: error).
    ///
    void set_derivative_source(const derivative_source source) { d_source_ = source; }

    ///
    /// @brief Set the back-calculation tracking gain kb.
    /// @param kb Discharge rate applied to the clipped output portion in
    ///           anti_windup::back_calculation mode (default: 1).
    /// @note Ignored by the other anti-windup modes.
    ///
    void set_tracking_time_constant(const T kb) { tracking_kb_ = kb; }

    ///
    /// @brief Advance the control law by one step with an explicit sample interval.
    /// @param set_point     Desired value.
    /// @param process_variable Measured value.
    /// @param dt            Time elapsed since the previous sample; clamped to
    ///                      [1e-4 s, 1 s].
    /// @return The (saturated) control output for this step.
    ///
    T compute(const T set_point, const T process_variable, duration_type dt) {
        // Guard against pathological sample intervals.
        dt = std::clamp(dt, min_step_, max_step_);
        const T dt_seconds = dt.count();

        // Current error.
        const T error = set_point - process_variable;

        // First sample: prime the stored measurement so that
        // derivative-on-measurement starts without a kick.
        if (!primed_) {
            last_measurement_ = process_variable;
            primed_ = true;
        }

        // Derivative term.
        const T d_term = (d_source_ == derivative_source::error)
                           ? kd_ * (error - last_error_value_) / dt_seconds
                           : -kd_ * (process_variable - last_measurement_) / dt_seconds;

        // Integrate the error; keep the integral inside its hard limits.
        const T integral_candidate
            = std::clamp(integral_ + (error * dt_seconds), integral_min_, integral_max_);

        // Weighted sum of the three terms, then output saturation.
        const T u_unsaturated = (kp_ * error) + (ki_ * integral_candidate) + d_term;
        const T u = std::clamp(u_unsaturated, output_min_, output_max_);

        // Commit the integral according to the anti-windup policy.
        switch (windup_) {
        case anti_windup::none: integral_ = integral_candidate; break;

        case anti_windup::clamping:
        {
            // Freeze integration only while a saturated output is still being
            // pushed further into the limit; unwinding is always allowed.
            const bool pushed_deeper
                = (u != u_unsaturated)
               && ((error > T { 0 } && u == output_max_) || (error < T { 0 } && u == output_min_));
            if (!pushed_deeper) { integral_ = integral_candidate; }
            break;
        }

        case anti_windup::back_calculation:
        {
            // Bleed off the clipped part of the output.
            T integral_back = integral_candidate;
            if (ki_ != T { 0 }) {
                integral_back += tracking_kb_ * (u - u_unsaturated) * dt_seconds / ki_;
            }
            integral_ = std::clamp(integral_back, integral_min_, integral_max_);
            break;
        }
        }

        // Remember the state for the next sample.
        last_error_value_ = error;
        last_measurement_ = process_variable;
        output_ = u;
        return u;
    }

    ///
    /// @brief Advance the control law by one step, sampling the wall clock.
    /// @param set_point Desired value.
    /// @param process_variable Measured value.
    /// @return The (saturated) control output for this step.
    ///
    /// @note dt = Clock::now() - <previous call or construction>, then the
    /// timestamp is re-anchored; delegates to the explicit-dt overload.
    ///
    T compute(const T set_point, const T process_variable) {
        const auto now_time = Clock::now();
        const duration_type dt = now_time - last_time_;
        last_time_ = now_time;
        return compute(set_point, process_variable, dt);
    }

    ///
    /// @brief Clear dynamic state; gains and limits are kept.
    ///
    /// Resets the integral, the last error/measurement, the stored output and
    /// the derivative priming flag, and re-anchors the timestamp to now().
    ///
    void reset() {
        integral_ = T { 0 };
        last_error_value_ = T { 0 };
        last_measurement_ = T { 0 };
        output_ = T { 0 };
        primed_ = false;
        last_time_ = Clock::now();
    }

    ///
    /// @brief Clear only the integral term (e.g. after a set point schedule change).
    ///
    void reset_integral() { integral_ = T { 0 }; }

    /// @brief Current proportional gain.
    [[nodiscard]]
    constexpr T proportional_gain() const {
        return kp_;
    }

    /// @brief Current integral gain.
    [[nodiscard]]
    constexpr T integral_gain() const {
        return ki_;
    }

    /// @brief Current derivative gain.
    [[nodiscard]]
    constexpr T derivative_gain() const {
        return kd_;
    }

    /// @brief Raw error integral currently held (input to the I term).
    [[nodiscard]]
    constexpr T integral() const {
        return integral_;
    }

    /// @brief Control output produced by the most recent compute() call.
    [[nodiscard]]
    constexpr T output() const {
        return output_;
    }

    /// @brief Error used by the most recent compute() call.
    [[nodiscard]]
    constexpr T last_error() const {
        return last_error_value_;
    }

    /// @brief Process variable of the most recent compute() call.
    [[nodiscard]]
    constexpr T last_measurement() const {
        return last_measurement_;
    }

    /// @brief Active anti-windup strategy.
    [[nodiscard]]
    constexpr anti_windup anti_windup_mode() const {
        return windup_;
    }

    /// @brief Active derivative input selection.
    [[nodiscard]]
    constexpr derivative_source derivative_mode() const {
        return d_source_;
    }

private:
    // Shortest sample interval honoured by compute().
    static constexpr duration_type min_step_ { T(0.0001) };
    // Longest sample interval honoured by compute().
    static constexpr duration_type max_step_ { T(1) };

    // P/I/D gains
    T kp_, ki_, kd_;

    // Raw error integral since controller start (the I term is ki_ * integral_)
    T integral_ = T { 0 };

    // Error value of the previous step
    T last_error_value_ = T { 0 };

    // Process variable of the previous step (derivative-on-measurement)
    T last_measurement_ = T { 0 };

    // Last saturated control output
    T output_ = T { 0 };

    // Output saturation window
    T output_min_ = std::numeric_limits<T>::lowest();
    T output_max_ = std::numeric_limits<T>::max();

    // Hard clamp window for the integral
    T integral_min_ = std::numeric_limits<T>::lowest();
    T integral_max_ = std::numeric_limits<T>::max();

    anti_windup windup_ = anti_windup::clamping;
    derivative_source d_source_ = derivative_source::error;
    T tracking_kb_ = T { 1 };

    // True once compute() has sampled at least one process variable
    bool primed_ = false;

    // Timestamp of the last compute() (or of construction), for the wall-clock overload
    Clock::time_point last_time_;
};

///
/// @brief Deduction guide for the common three-gain construction,
/// e.g. `pid_controller p(1.0f, 0.1f, 0.0f);` -> `pid_controller<float>`.
///
template<typename T>
pid_controller(T, T, T) -> pid_controller<T>;

} // namespace ba7lya::pid
