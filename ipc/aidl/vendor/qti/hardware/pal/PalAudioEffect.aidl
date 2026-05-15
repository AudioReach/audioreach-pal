/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

@VintfStability
@Backing(type="int")
enum PalAudioEffect {
    PAL_AUDIO_EFFECT_NONE = 0x0,
    /**
     * < No audio effect ie., EC_OFF_NS_OFF
     */
    PAL_AUDIO_EFFECT_EC = 0x1,
    /**
     * < Echo Cancellation
     */
    PAL_AUDIO_EFFECT_NS = 0x2,
    /**
     * < Noise Suppression
     */
    PAL_AUDIO_EFFECT_ECNS = 0x3,
}
