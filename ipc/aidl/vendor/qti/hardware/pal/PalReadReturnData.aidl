/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

import vendor.qti.hardware.pal.PalBuffer;

@VintfStability
parcelable PalReadReturnData {
    int ret;
    PalBuffer[] buffer;
}
