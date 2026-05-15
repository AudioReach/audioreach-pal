/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

@VintfStability
@Backing(type="int")
enum PalDrainType {
    /**
     * request notification when all accumlated data has be
     *  drained.
     */
    PAL_DRAIN,
    /**
     * request notification when drain completes shortly before all
     *  accumlated data of the current track has been played out
     */
    PAL_DRAIN_PARTIAL,
}
