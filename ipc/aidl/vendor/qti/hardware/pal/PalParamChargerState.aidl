/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

package vendor.qti.hardware.pal;

/**
 * Payload For ID: PAL_PARAM_ID_CHARGER_STATE
 *  Description   : Charger State
 */
@VintfStability
parcelable PalParamChargerState {
    boolean online;
    /**
     * < status of charger
     */
    boolean concurrentBoostEnable;
}
