/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

@VintfStability
parcelable PalTimeus {
    int valLsw;
    /**
     * Lower 32 bits of 64 bit time value in microseconds
     */
    int valMsw;
}
