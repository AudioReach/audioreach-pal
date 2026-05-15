/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

/**
 * Commands that can be executed on the driver read/write done thread.
 */
@VintfStability
@Backing(type="int")
enum PalReadWriteDoneCommand {
    WRITE_READY,
    DRAIN_READY,
    PARTIAL_DRAIN_READY,
    READ_DONE,
    ERROR,
}
