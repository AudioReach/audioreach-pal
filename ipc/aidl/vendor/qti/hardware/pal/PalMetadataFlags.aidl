/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

/**
 * Metadata flags
 */
@VintfStability
@Backing(type="int")
enum PalMetadataFlags {
    PAL_META_DATA_FLAGS_NONE = 0,
    PAL_META_DATA_VALID_TS,
    PAL_META_DATA_FLAGS_MAX,
}
