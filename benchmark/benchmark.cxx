///
/// @file benchmark.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Google benchmarks for the pid_controller hot path: per-term
/// configuration variants, wall-clock sampling overhead, anti-windup modes
/// and closed-loop throughput against an inline first-order plant.
/// @version 0.2
/// @date 2026-09-16
/// SPDX-License-Identifier: MIT
/// @copyright Copyright (c) 2025
///

#include <benchmark/benchmark.h>
#include <chrono>

#include "pid_controller.hxx"

using namespace ba7lya::pid;

namespace {

template<typename T>
auto seconds_as(T s) {
    return std::chrono::duration<T> { s };
}

} // namespace

/// Baseline: float, explicit dt, no saturation.
static void bm_compute_float(benchmark::State& state) {
    pid_controller<float> pid(2.0f, 0.5f, 1.0f);
    const auto dt = seconds_as<float>(0.01f);
    for (auto _ : state) { benchmark::DoNotOptimize(pid.compute(10.0f, 7.5f, dt)); }
}

BENCHMARK(bm_compute_float);

/// Same workload in double precision.
static void bm_compute_double(benchmark::State& state) {
    pid_controller<double> pid(2.0, 0.5, 1.0);
    const auto dt = seconds_as<double>(0.01);
    for (auto _ : state) { benchmark::DoNotOptimize(pid.compute(10.0, 7.5, dt)); }
}

BENCHMARK(bm_compute_double);

/// Wall-clock overload: quantifies Clock::now() overhead per sample.
static void bm_compute_clock_sampled(benchmark::State& state) {
    pid_controller<double> pid(2.0, 0.5, 1.0);
    for (auto _ : state) { benchmark::DoNotOptimize(pid.compute(10.0, 7.5)); }
}

BENCHMARK(bm_compute_clock_sampled);

/// Saturated output with clamping anti-windup (steady windup state).
static void bm_compute_saturated_clamping(benchmark::State& state) {
    pid_controller<double> pid(100.0, 10.0, 1.0);
    pid.set_output_limits(-1.0, 1.0);
    const auto dt = seconds_as<double>(0.01);
    for (auto _ : state) { benchmark::DoNotOptimize(pid.compute(2.0, 0.0, dt)); }
}

BENCHMARK(bm_compute_saturated_clamping);

/// Back-calculation branch of the anti-windup switch.
static void bm_compute_back_calculation(benchmark::State& state) {
    pid_controller<double> pid(100.0, 10.0, 1.0);
    pid.set_output_limits(-1.0, 1.0);
    pid.set_anti_windup(anti_windup::back_calculation);
    const auto dt = seconds_as<double>(0.01);
    for (auto _ : state) { benchmark::DoNotOptimize(pid.compute(2.0, 0.0, dt)); }
}

BENCHMARK(bm_compute_back_calculation);

/// Derivative-on-measurement source.
static void bm_compute_derivative_on_measurement(benchmark::State& state) {
    pid_controller<double> pid(2.0, 0.5, 1.0);
    pid.set_derivative_source(derivative_source::measurement);
    const auto dt = seconds_as<double>(0.01);
    for (auto _ : state) { benchmark::DoNotOptimize(pid.compute(10.0, 7.5, dt)); }
}

BENCHMARK(bm_compute_derivative_on_measurement);

/// State management path: reconfiguration every sample.
static void bm_reset_and_set_gain(benchmark::State& state) {
    pid_controller<double> pid(2.0, 0.5, 1.0);
    const auto dt = seconds_as<double>(0.01);
    for (auto _ : state) {
        pid.compute(10.0, 7.5, dt);
        pid.set_gain(2.1, 0.5, 1.0);
        pid.reset();
        benchmark::ClobberMemory();
    }
}

BENCHMARK(bm_reset_and_set_gain);

/// Throughput of a full closed loop: 10k controller steps against an
/// inline first-order lag plant (simulated seconds per second of work).
static void bm_closed_loop_10k_steps(benchmark::State& state) {
    constexpr int steps = 10000;
    constexpr double dt_seconds = 1e-3;
    for (auto _ : state) {
        pid_controller<double> pid(30.0, 5.0, 2.0);
        pid.set_output_limits(-50.0, 50.0);
        const auto dt = seconds_as<double>(dt_seconds);
        double y = 0.0;
        for (int i = 0; i < steps; ++i) {
            const double u = pid.compute(10.0, y, dt);
            y += (2.0 * u - y) * dt_seconds / 0.5; // first-order lag, K=2, T=0.5
        }
        benchmark::DoNotOptimize(y);
    }
    state.SetItemsProcessed(state.iterations() * steps);
}

BENCHMARK(bm_closed_loop_10k_steps);
