/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

import vendor.qti.hardware.pal.PalAudioFmt;
import vendor.qti.hardware.pal.PalChannelInfo;

/**
 * Media configuraiton
 */
@VintfStability
parcelable PalMediaConfig {
    int sampleRate;
    int bitwidth;
    PalChannelInfo chInfo;
    PalAudioFmt audioFormatId;
}
