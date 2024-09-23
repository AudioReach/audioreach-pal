/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

package vendor.qti.hardware.pal;

import vendor.qti.hardware.pal.PalStreamType;
import vendor.qti.hardware.pal.PalDeviceId;

/**
 * PAL buffer structure used for reading/writing buffers from/to the stream
 */
@VintfStability
parcelable PalCustomPayloadInfo {
    PalStreamType streamType;
    PalDeviceId deviceId;
    int sample_rate;
    int instanceId;
    boolean streamless;
}
