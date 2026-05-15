/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

@Backing(type="int") @VintfStability
enum Status {
    UNKNOWN = -1,
    SUCCESS = 0,
    IO_ERROR,
    BUSY,
    NO_SPACE,
    INVALID_FD,
    ADVERTISE_ERROR,
    PROTOCOL_NOT_AVAILABLE,
    NOT_SUPPORTED,
    DOWN_WITH_SSR,
    NOW_INPROGRESS,
    ALREADY_INPROGRESS,
    CANCELLED,
    NOT_RECOVERABLE,
}
