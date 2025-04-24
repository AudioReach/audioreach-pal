/**
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 **/

#pragma once

#include <thread>
#include <string>

#ifdef _POSIX_THREADS
#pragma message("supports POSIX THREADS")
#include <pthread.h>
#endif


void setThreadName(const std::thread& thread, const std::string& name) {
#ifdef _POSIX_THREADS
    auto& pthread = const_cast<std::thread&>(thread);
    if (int ret = ::pthread_setname_np(pthread.native_handle(), name.substr(0, 15).c_str()); ret) {
        return;
    }
#endif
}