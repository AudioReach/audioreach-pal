/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

/**
 * Audio stream direction enumeration
 */
@VintfStability
@Backing(type="int")
enum PalStreamDirection {
    PAL_AUDIO_OUTPUT = 0x1,
    PAL_AUDIO_INPUT = 0x2,
    PAL_AUDIO_INPUT_OUTPUT = 0x3,
    PAL_AUDIO_INVALID = 0x4,
}
