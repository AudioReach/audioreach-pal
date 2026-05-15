/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

/**
 * Mmap buffer read/write position returned by GetMmapPosition.
 * note\ Used by streams opened in mmap mode.
 */
@VintfStability
parcelable PalMmapPosition {
    long timeNanoseconds;
    /**
     * < timestamp in ns, CLOCK_MONOTONIC
     */
    int positionFrames;
}
