/**
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "Status.h"

std::string toString(const Status& status) noexcept {
    switch (status) {
        case Status::NO_INIT:
            return "NO_INIT";
        case Status::FAILED:
            return "FAILED";
        case Status::OK:
            return "OK";
        default:
            return "UNKNOWN";  // Handles any unrecognized status values.
    }
}
