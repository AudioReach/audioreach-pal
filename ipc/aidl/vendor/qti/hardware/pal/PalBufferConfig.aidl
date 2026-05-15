/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

@VintfStability
parcelable PalBufferConfig {
    int bufCount;
    /**
     * < number of buffers
     */
    int bufSize;
    /**
     * < This would be the size of each buffer
     */
    int maxMetadataSize;
}
