/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

import vendor.qti.hardware.pal.PalCallbackBufferInfo;
import vendor.qti.hardware.pal.TimeSpec;

@VintfStability
parcelable PalCallbackBuffer {
    byte[] buffer;
    int size;
    TimeSpec timeStamp;
    int status;
    PalCallbackBufferInfo cbBufInfo;
}
