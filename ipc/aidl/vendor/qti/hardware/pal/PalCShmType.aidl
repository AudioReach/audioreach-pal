/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

package vendor.qti.hardware.pal;

@Backing(type="int") @VintfStability
enum PalCShmType {
    CACHED = 1,
    UNCACHED = 2,
}
