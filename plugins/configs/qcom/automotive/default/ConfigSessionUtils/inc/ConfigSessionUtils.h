/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef CONFIG_SESSION_UTILS_H
#define CONFIG_SESSION_UTILS_H

#include "Stream.h"

int configureMFC(const std::shared_ptr<ResourceManager>& rm, struct pal_stream_attributes &sAttr,
            struct pal_device &dAttr, const std::vector<int> &pcmDevIds, const char* intf,
            PayloadBuilder* builder);
int setSlotMask(const std::shared_ptr<ResourceManager>& rm, struct pal_stream_attributes &sAttr,
                    struct pal_device &dAttr, const std::vector<int> &pcmDevIds);
int32_t reconfigureInCallMusicStream(struct pal_media_config config, PayloadBuilder* builder);
int reconfigCommon(Stream* streamHandle, void* pluginPayload);
int handleDeviceRotation(const std::shared_ptr<ResourceManager>& rm,
                    Stream *s, pal_speaker_rotation_type rotation_type,
                    int device, struct mixer *mixer, PayloadBuilder* builder,
                    std::vector<std::pair<int32_t, std::string>> rxAifBackEnds);
int32_t pluginConfigSetParam(Stream* s, void* payload);
#endif