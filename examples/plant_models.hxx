///
/// @file plant_models.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Abstract (lumped-parameter) plant models used by the example
/// programs: first order lag, mass-spring-damper, N-stage cascade and a
/// simple thermal plant. All models are plain stdlib and integrate at an
/// explicit sample interval, so simulated loops stay deterministic.
/// @version 0.2
/// @date 2026-09-16
/// SPDX-License-Identifier: MIT
/// @copyright Copyright (c) 2025
///

#pragma once

#include <array>
#include <cstddef>

///
/// @brief First-order lag system: T * dy/dt + y = K * u.
///
/// Discretized with the backward Euler method.
///
class first_order_system {
public:
    ///
    /// @param gain Static gain K of the system.
    /// @param time_const Time constant T (seconds).
    ///
    first_order_system(const double gain, const double time_const)
        : K_(gain)
        , T_(time_const) {}

    ///
    /// @brief Advance the plant by one step.
    /// @param input Current control input u.
    /// @param dt Sample interval (seconds).
    /// @return The new output y.
    ///
    double update(const double input, const double dt) {
        // Backward Euler discretization of T*dy/dt + y = K*u:
        // y[n] = (dt*K*u + T*y[n-1]) / (T + dt)
        y_ = (K_ * dt * input + T_ * y_) / (T_ + dt);
        return y_;
    }

    /// @brief Current output (state) of the plant.
    double state() const { return y_; }

    /// @brief Return the plant to its rest state.
    void reset() { y_ = 0.0; }

private:
    double K_;     // System gain
    double T_;     // Time constant
    double y_ = 0; // Output
};

///
/// @brief Second-order mass-spring-damper plant: m*x'' + c*x' + k*x = u.
///
/// Integrated with the semi-implicit Euler method.
///
class second_order_system {
public:
    struct state {
        double pos; // Position
        double vel; // Velocity
    };

    ///
    /// @param mass Mass m (kg).
    /// @param damping Damping coefficient c (N*s/m).
    /// @param stiffness Spring constant k (N/m).
    ///
    second_order_system(const double mass, const double damping, const double stiffness)
        : mass_(mass)
        , damping_(damping)
        , stiffness_(stiffness) {}

    ///
    /// @brief Advance the plant by one step.
    /// @param control_input Force u (N).
    /// @param dt Sample interval (seconds).
    /// @return The new plant state.
    ///
    state update(const double control_input, const double dt) {
        // Acceleration from the second-order differential equation
        const double acc
            = (control_input - damping_ * current_.vel - stiffness_ * current_.pos) / mass_;

        // Semi-implicit Euler integration
        current_.vel += acc * dt;
        current_.pos += current_.vel * dt;

        return current_;
    }

    /// @brief Current plant state.
    const state& get_state() const { return current_; }

    /// @brief Return the plant to its rest state.
    void reset() { current_ = { 0, 0 }; }

private:
    state current_ { 0, 0 };
    double mass_;      // Mass
    double damping_;   // Damping coefficient
    double stiffness_; // Stiffness coefficient
};

///
/// @brief Plant made of N identical first-order lags in series.
///
/// Each stage obeys tau * dx_i/dt = x_{i-1} - x_i (with x_{-1} = u, the
/// control input), discretized with backward Euler. The cascade replaces the
/// state-space matrix formulation: the structure is banded by construction,
/// so a plain loop suffices.
///
template<std::size_t N>
class cascade_system {
public:
    /// @param time_const Time constant tau shared by all stages (seconds).
    explicit cascade_system(const double time_const)
        : tau_(time_const) {}

    ///
    /// @brief Advance all stages by one step.
    /// @param control_input Cascade input u.
    /// @param dt Sample interval (seconds).
    /// @return The output of the last stage.
    ///
    double update(const double control_input, const double dt) {
        double input = control_input;
        for (auto& stage : state_) {
            // Backward Euler: stage = (dt*input + tau*stage) / (tau + dt)
            stage = (dt * input + tau_ * stage) / (tau_ + dt);
            input = stage;
        }
        return state_.back();
    }

    /// @brief Output of the last stage.
    double state() const { return state_.back(); }

    /// @brief Return the plant to its rest state.
    void reset() { state_.fill(0.0); }

private:
    std::array<double, N> state_ {}; // Stage outputs
    double tau_;                     // Stage time constant
};

///
/// @brief First-order thermal plant: a heater with limited power and linear
/// losses to the ambient temperature (Newton cooling).
///
class temperature_system {
public:
    /// @param initial_temp Starting temperature (deg C).
    explicit temperature_system(const double initial_temp = 25.0)
        : current_temp_(initial_temp) {}

    ///
    /// @brief Advance the plant by one step.
    /// @param control_signal Heater power command (0..100 %).
    /// @param dt Sample interval (seconds).
    /// @return The new temperature.
    ///
    double update(const double control_signal, const double dt) {
        const double heat_input = control_signal * 0.1;             // heater efficiency factor
        const double heat_loss = (current_temp_ - ambient_) * 0.02; // Newton cooling
        current_temp_ += (heat_input - heat_loss) * dt;
        return current_temp_;
    }

    /// @brief Current temperature (deg C).
    double get_temperature() const { return current_temp_; }

private:
    double current_temp_;
    static constexpr double ambient_ = 25.0; // Ambient temperature (deg C)
};
