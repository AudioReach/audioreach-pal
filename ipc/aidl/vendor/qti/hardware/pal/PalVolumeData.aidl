/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

import vendor.qti.hardware.pal.PalChannelVolKv;

/**
 * Volume data structure defintion used as argument for volume command
 */
@VintfStability
parcelable PalVolumeData {
    PalChannelVolKv[] volPair;
}
