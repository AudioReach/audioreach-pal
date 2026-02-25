/**
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <cstdint>
#include <string>

/**
 * @brief Represents the outcome of an API operation.
 *
 * This enum provides a generic status indicator for API calls.
 * - Maintain `OK` as `0` for compatibility with existing code.
 * - Add additional statuses as needed, using negative values for errors.
 * - Ensure corresponding string representations are added in `toString()`.
 */
enum class Status : int8_t {
    // Future statuses can be added here in the beginning, e.g., TIMEOUT = -4, INVALID_ARG = -3
    NO_INIT = -2,
    FAILED = -1,  ///< Operation failed due to an error.
    OK = 0,       ///< Operation completed successfully
};

/**
 * @brief Converts a Status enum value to its string representation.
 *
 * @param status The Status value to convert.
 * @return A human-readable string describing the status.
 *
 * This function should be updated whenever new Status values are introduced.
 */
std::string toString(const Status& status) noexcept;
