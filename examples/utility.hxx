///
/// @file utility.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Small helpers shared by the example programs.
/// @version 0.2
/// @date 2026-09-16
/// SPDX-License-Identifier: MIT
/// @copyright Copyright (c) 2025
///

#pragma once

#include <array>
#include <iostream>
#include <stdexcept>
#include <string>

///
/// @brief Parse up to three numeric command line arguments.
/// @param argc Argument count from main().
/// @param argv Argument vector from main().
/// @return The three parsed values (missing arguments are left uninitialized).
/// @throw std::runtime_error If an argument is not a pure number.
///
inline std::array<double, 3> parse_args(int argc, char* argv[]) {
    std::array<double, 3> args {};
    for (int i = 1; i < argc; ++i) { // argv[0] is the program name, skip it
        try {
            size_t pos;
            double value = std::stod(argv[i], &pos);

            // The whole string must have been consumed by the conversion
            if (argv[i][pos] != '\0') {
                throw std::invalid_argument("trailing characters after number");
            }
            args[i - 1] = value;
        }
        catch (const std::exception& e) {
            std::cerr << "error parsing argument " << i << " (" << argv[i] << "): " << e.what()
                      << "\n";
            throw std::runtime_error("error parsing argument");
        }
    }
    return args;
}
