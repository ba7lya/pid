# SPDX-License-Identifier: MIT
# --- CMake Version ---
cmake_minimum_required(VERSION 3.21)

find_package(benchmark CONFIG REQUIRED)

# --- Taget ---
add_executable(${PROJECT_NAME}_benchmark)

# --- Sources ---
target_sources(
	${PROJECT_NAME}_benchmark
	PRIVATE
    ${PROJECT_SOURCE_DIR}/benchmark/benchmark.cxx
)

# --- Linakge ---
target_link_libraries(
	${PROJECT_NAME}_benchmark
	PRIVATE
	${PROJECT_NAME}
	benchmark::benchmark
	benchmark::benchmark_main
)
