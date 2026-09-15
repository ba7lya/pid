# PIDController

<!-- SPDX-License-Identifier: MIT -->

A modern C++23 header-only PID controller: templated on scalar type and clock,
with std::chrono-based sampling, output saturation, selectable integral
anti-windup (clamping / back-calculation) and optional derivative-on-
measurement.

![CI](https://github.com/BA7LYA/pid/actions/workflows/ci.yml/badge.svg)

## Quick start

```cpp
#include "ba7lya/pid/pid_controller.hxx"

using namespace ba7lya::pid;
using namespace std::chrono;

pid_controller<double> pid(/*kp=*/2.0, /*ki=*/0.5, /*kd=*/0.1);
pid.set_output_limits(-100.0, 100.0);           // actuator saturation
pid.set_anti_windup(anti_windup::clamping);     // default

double u1 = pid.compute(set_point, measurement);            // wall clock sampled
double u2 = pid.compute(set_point, measurement, duration<double>{0.01}); // fixed dt
```

The control law is the positional form
`u = kp*e + ki*integral(e) + kd*de/dt` with `e = set_point - measurement`;
the stored integral is the raw error integral, so changing `ki` with
`set_gain()` never resurrects stale control effort. `dt` is clamped to
[1e-4 s, 1 s] to guard against stalled or restarted loops.

## Layout and include paths

- In this repository, public headers sit flat in [include/](include/) and are
  included by bare filename (`#include "pid_controller.hxx"`).
- When installed as a dependency, headers land under
  `include/<author>/<package>/`, i.e. `#include "ba7lya/pid/pid_controller.hxx"`.
- Example/test helper headers (plant models, argument parsing) stay in their
  own directories and are not installed.

## Building

Requires CMake >= 3.21, a C++23 compiler (MSVC 2019+/GCC 11+/Clang 14+), and
the [vcpkg](https://github.com/microsoft/vcpkg) submodule for GTest,
Google Benchmark and ODE (ODE is used by the tests and the motor example
only; the library itself has no dependencies).

```bash
git submodule update --init --recursive
cmake --preset MSVC-2022-x64-Ninja        # or GCC-Linux-Ninja / Clang-MacOS-Ninja
cmake --build --preset MSVC-2022-x64-Ninja-Release
ctest --preset MSVC-2022-x64-Ninja-Test -C Release
```

Build options:

| Option | Default | Meaning |
| ------ | ------- | ------- |
| `ba7lya.pid_BUILD_EXAMPLES` | ON | Build the example programs |
| `ba7lya.pid_BUILD_TEST` | OFF | Build unit tests (GTest + ODE plants) |
| `ba7lya.pid_BUILD_BENCHMARK` | OFF | Build Google benchmarks |

Install the headers with `cmake --install build/<preset> --prefix <dir>`.

## Examples

- [examples/example_1_order.cxx](examples/example_1_order.cxx) — first-order
  lag plant.
- [examples/example_2_order.cxx](examples/example_2_order.cxx) — mass-spring-
  damper with actuator saturation and a mid-run set point reversal.
- [examples/example_n_order.cxx](examples/example_n_order.cxx) — 4-stage
  cascade plant.
- [examples/example_temp.cxx](examples/example_temp.cxx) — thermal plant with
  power saturation.
- [examples/example_motor.cxx](examples/example_motor.cxx) — brushed DC motor
  (ODE-simulated rotor, fully encapsulated in
  [examples/motor.hxx](examples/motor.hxx)): speed-loop windup comparison and
  a three-loop position/velocity/current servo.

## Reference

[PID controller @ Wikipedia](https://en.wikipedia.org/wiki/PID_controller)

## License

MIT License - see [LICENSE](LICENSE).
