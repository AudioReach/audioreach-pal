/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_NDEBUG 0

#include "ar_osal_mem_op.h"
#include <stdlib.h>
#include <chrono>
#include <string.h>
#include <stdio.h>
#ifdef PAL_USE_SYSLOG
#include <syslog.h>
#define ALOGE(fmt, arg...) syslog (LOG_ERR, fmt, ##arg)
#define ALOGI(fmt, arg...) syslog (LOG_INFO, fmt, ##arg)
#define ALOGD(fmt, arg...) syslog (LOG_DEBUG, fmt, ##arg)
#define ALOGV(fmt, arg...) syslog (LOG_NOTICE, fmt, ##arg)
#else
#include <log/log.h>
#endif

#define PAL_LOG_ERR             (0x1) /**< error message, represents code bugs that should be debugged and fixed.*/
#define PAL_LOG_INFO            (0x2) /**< info message, additional info to support debug */
#define PAL_LOG_DBG             (0x4) /**< debug message, required at minimum for debug.*/
#define PAL_LOG_VERBOSE         (0x8)/**< verbose message, useful primarily to help developers debug low-level code */

extern uint32_t pal_log_lvl;

#define PAL_FATAL(log_tag, arg,...)                                          \
    if (pal_log_lvl & PAL_LOG_ERR) {                              \
        ALOGE("%s: %d: "  arg, __func__, __LINE__, ##__VA_ARGS__);\
    }
#define PAL_ERR(log_tag, arg,...)                                          \
    if (pal_log_lvl & PAL_LOG_ERR) {                              \
        ALOGE("%s: %d: "  arg, __func__, __LINE__, ##__VA_ARGS__);\
    }
#define PAL_DBG(log_tag,arg,...)                                           \
    if (pal_log_lvl & PAL_LOG_DBG) {                               \
        ALOGD("%s: %d: "  arg, __func__, __LINE__, ##__VA_ARGS__); \
    }
#define PAL_INFO(log_tag,arg,...)                                         \
    if (pal_log_lvl & PAL_LOG_INFO) {                             \
        ALOGI("%s: %d: "  arg, __func__, __LINE__, ##__VA_ARGS__);\
    }
#define PAL_VERBOSE(log_tag,arg,...)                                      \
    if (pal_log_lvl & PAL_LOG_VERBOSE) {                          \
        ALOGV("%s: %d: "  arg, __func__, __LINE__, ##__VA_ARGS__);\
    }

/**
 * @brief CHECK macro for runtime assertions with fatal logging.
 *
 * This macro is intended to improve debugging by validating assumptions
 * during development or critical runtime checks. If the condition fails,
 * it logs a fatal error message and typically terminates the program.
 *
 * **Important Notes:**
 * - CHECK should NOT be used for business logic or control flow decisions.
 * - It is strictly for debugging and validation purposes.
 * Release builds may disable this macro.
 *
 * Usage:
 *     CHECK(ptr != nullptr); // Ensures pointer is not null
 *
 * @param expected_condition The condition to check. If false, triggers fatal log.
 */
#define CHECK_MSG(expected_condition, crash_message) \
    LOG_FATAL_IF(!(expected_condition),              \
                 "💥 at line:%d " #expected_condition " failed! " #crash_message, __LINE__)

#define CHECK(expected_condition) CHECK_MSG(expected_condition, "")

/**
 * TIME_LOG(expr)
 * ----------------
 * Measures the execution time of any expression.
 * Usage:
 * ------
 * auto ret = TIME_LOG(a->callOperation(x, y, z));
 *
 * This will:
 * - Execute a->callOperation(x, y, z)
 * - Log: "<function>: <line>: Time taken for a->callOperation(x, y, z): <duration> ms"
 * - Store the return value in 'ret'
 *
 */

#define TIME_LOG(expr)                                                                      \
    ([&]() {                                                                                \
        auto start = std::chrono::steady_clock::now();                                      \
        auto result = (expr);                                                               \
        auto end = std::chrono::steady_clock::now();                                        \
        auto duration =                                                                     \
                std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); \
        ALOGI("%s: %d: Time taken for %s: %lld ms", __func__, __LINE__, #expr,              \
              static_cast<long long>(duration));                                            \
        return result;                                                                      \
    }())
