/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

/**
 * The message queue flags used to synchronize reads and writes from
 * message queues used by PAL
 */
@VintfStability
@Backing(type="int")
enum PalMessageQueueFlagBits {
    NOT_EMPTY = 1 << 0,
    NOT_FULL = 1 << 1,
}
