/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

import android.os.ParcelFileDescriptor;

@VintfStability
parcelable PalParamPayloadShmem {
    long payloadSize;
    ParcelFileDescriptor fd;
}
