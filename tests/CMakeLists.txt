# SPDX-License-Identifier: MIT
# --- CMake Version ---
cmake_minimum_required(VERSION 3.21)

find_package(GTest CONFIG REQUIRED)
find_package(ode CONFIG REQUIRED)

include(GoogleTest)

# --- Taget ---
add_executable(${PROJECT_NAME}_test)

# --- Sources ---
target_sources(
	${PROJECT_NAME}_test
	PRIVATE
	test_pid_core.cxx
	test_plant_closed_loop.cxx
)

# --- Linakge ---
# The library itself is dependency-free; ODE is only used to build the
# physical plant the closed-loop tests control.
target_link_libraries(
	${PROJECT_NAME}_test
	PRIVATE
	${PROJECT_NAME}
	ODE::ODE
	GTest::gtest
	GTest::gtest_main
)

gtest_discover_tests(${PROJECT_NAME}_test DISCOVERY_MODE PRE_TEST)
