/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STREAMSENSORRENDERER_H_
#define STREAMSENSORRENDERER_H_

#include "StreamCommon.h"
#include "ResourceManager.h"
#include "Device.h"
#include "Session.h"

extern "C" Stream* CreateSensorRendererStream(const struct pal_stream_attributes *sattr, struct pal_device *dattr,
                               const uint32_t no_of_devices, const struct modifier_kv *modifiers,
                               const uint32_t no_of_modifiers, const std::shared_ptr<ResourceManager> rm);

class StreamSensorRenderer : public StreamCommon
{
public:
    StreamSensorRenderer(const struct pal_stream_attributes *sattr, struct pal_device *dattr,
                     const uint32_t no_of_devices, const struct modifier_kv *modifiers,
                     const uint32_t no_of_modifiers, const std::shared_ptr<ResourceManager> rm);
    ~StreamSensorRenderer();
};

#endif//STREAMSENSORRENDERER_H_
