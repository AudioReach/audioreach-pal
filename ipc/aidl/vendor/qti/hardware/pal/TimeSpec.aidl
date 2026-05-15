/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

/**
 * A substitute for POSIX timespec.
 */
@VintfStability
parcelable TimeSpec {
    long tvSec;
    long tvNSec;
}
