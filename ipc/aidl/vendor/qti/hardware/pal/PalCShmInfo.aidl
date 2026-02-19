/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

package vendor.qti.hardware.pal;

import vendor.qti.hardware.pal.PalCShmType ;

@VintfStability
parcelable PalCShmInfo {
    long memID;
    PalCShmType type;
    android.hardware.common.NativeHandle fdMemory;
    int flags;
}
