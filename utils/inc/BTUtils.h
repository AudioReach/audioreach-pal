/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef BT_UTILS_H
#define BT_UTILS_H

#include "Stream.h"
#include <stdio.h>

void BTUtilsInit();
int32_t BTUtilsDeviceNotReadyToDummy(Stream* s, bool& a2dpSuspend);
int32_t BTUtilsDeviceNotReady(Stream* s, bool& a2dpSuspend);
void handleA2dpBleConcurrency(std::shared_ptr<Device> *inDev,
            struct pal_device *inDevAttr, struct pal_device &dummyDevAttr,
            std::vector <std::tuple<Stream *, uint32_t>> &streamDevDisconnect,
            std::vector <std::tuple<Stream *, struct pal_device *>> &streamDevConnect);
int handleBTDeviceConnectionChange(pal_param_device_connection_t connection_state,
                                    std::vector <pal_device_id_t> &avail_devices_);
int32_t a2dpSuspend(pal_device_id_t dev_id);
int32_t a2dpSuspendToDummy(pal_device_id_t dev_id);
int32_t a2dpResume(pal_device_id_t dev_id);
int32_t a2dpResumeFromDummy(pal_device_id_t dev_id);
int32_t a2dpCaptureSuspend(pal_device_id_t dev_id);
int32_t a2dpCaptureSuspendToDummy(pal_device_id_t dev_id);
int32_t a2dpCaptureResume(pal_device_id_t dev_id);
int32_t a2dpCaptureResumeFromDummy(pal_device_id_t dev_id);
int32_t a2dpReconfig();
void processBTCodecInfo(const XML_Char **attr, const int attr_count);
void updateBtCodecMap(std::pair<uint32_t, std::string> key, std::string value);
std::string getBtCodecLib(uint32_t codecFormat, std::string codecType);
void updateBtSlimClockSrcMap(uint32_t key, uint32_t value);
uint32_t getBtSlimClockSrc(uint32_t codecFormat);
void reconfigureScoStreams();
int setBTParameter(uint32_t param_id, void *param_payload,
                     size_t payload_size);
int getBTParameter(uint32_t param_id, void **param_payload,
                     size_t *payload_size, void *query __unused);
#endif