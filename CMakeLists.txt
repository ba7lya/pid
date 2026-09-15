# SPDX-License-Identifier: MIT
# --- cmake version ---
cmake_minimum_required(VERSION 3.21)

# --- project ---
project(ba7lya.pid LANGUAGES CXX VERSION 0.2)

# --- includes ---
include(CTest)

# --- build options ---
option(${PROJECT_NAME}_BUILD_BENCHMARK "Build the benchmark" OFF)
option(${PROJECT_NAME}_BUILD_EXAMPLES  "Build demo programs" ON )
option(${PROJECT_NAME}_BUILD_TEST      "Build Test"          ON )

# --- target ---
# Header-only: the public header includes nothing but the standard library,
# so no third-party dependency is propagated to consumers.
add_library(${PROJECT_NAME} INTERFACE)

# --- include directories ---
target_include_directories(
	${PROJECT_NAME}
	INTERFACE
	${PROJECT_SOURCE_DIR}/include/
)

# --- compile features ---
target_compile_features(
	${PROJECT_NAME}
	INTERFACE
	cxx_std_23
)

# --- benchmarks ---
if(${PROJECT_NAME}_BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif()

# --- demo ---
if(${PROJECT_NAME}_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# --- unit tests ---
if(${PROJECT_NAME}_BUILD_TEST)
    add_subdirectory(tests)
endif()

# --- install ---
# In-repo public API headers sit directly in include/. When installed as a
# dependency they are laid out under include/<author>/<package>/, so consumers
# write: #include "ba7lya/pid/pid_controller.hxx"
install(
	DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION include/ba7lya/pid
	FILES_MATCHING PATTERN "*.hxx" PATTERN "*.hpp" PATTERN "*.h"
)
