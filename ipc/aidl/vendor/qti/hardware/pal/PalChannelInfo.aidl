/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

/**
 * Audio channel info data structure
 */
@VintfStability
parcelable PalChannelInfo {
    char channels;
    byte[64] chMap;
}
