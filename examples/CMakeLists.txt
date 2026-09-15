# SPDX-License-Identifier: MIT
# --- CMake Version ---
cmake_minimum_required(VERSION 3.21)

# --- plain examples (stdlib + the header-only library only) ---
# Every demo program is a single .cxx file that only needs the header-only
# library; keep them in one list to avoid repeated boilerplate.
set(_examples
	example_1_order
	example_2_order
	example_n_order
	example_temp
)

foreach(_example IN LISTS _examples)
	add_executable(${PROJECT_NAME}_${_example})
	target_sources(${PROJECT_NAME}_${_example} PRIVATE ${_example}.cxx)
	target_link_libraries(${PROJECT_NAME}_${_example} PRIVATE ${PROJECT_NAME})
endforeach()

# --- ODE-based example ---
# Physical DC motor demos: the rotor is simulated with the ODE rigid body
# library (wrapped in examples/ode_plants.hxx), so this target depends on ODE.
find_package(ode CONFIG REQUIRED)
add_executable(${PROJECT_NAME}_example_motor)
target_sources(${PROJECT_NAME}_example_motor PRIVATE example_motor.cxx)
target_link_libraries(${PROJECT_NAME}_example_motor PRIVATE ${PROJECT_NAME} ODE::ODE)
