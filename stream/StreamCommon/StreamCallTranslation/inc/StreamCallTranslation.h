/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef StreamCallTranslation_H_
#define StreamCallTranslation_H_

#include "StreamCommon.h"

class ResourceManager;
class Device;
class Session;

extern "C" Stream* CreateCallTranslationStream(const struct pal_stream_attributes *sattr, struct pal_device *dattr,
                                  const uint32_t no_of_devices, const struct modifier_kv *modifiers,
                                  const uint32_t no_of_modifiers, const std::shared_ptr<ResourceManager> rm);

class StreamCallTranslation : public StreamCommon
{
public:
    StreamCallTranslation(const struct pal_stream_attributes *sattr, struct pal_device *dattr,
              const uint32_t no_of_devices,
              const struct modifier_kv *modifiers, const uint32_t no_of_modifiers,
              const std::shared_ptr<ResourceManager> rm);
    ~StreamCallTranslation();
};

#endif//StreamCallTranslation_H_