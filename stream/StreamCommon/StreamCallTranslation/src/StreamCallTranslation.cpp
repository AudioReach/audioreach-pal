/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "StreamCallTranslation"

#include "StreamCallTranslation.h"
#include "Session.h"
#include "ResourceManager.h"
#include <unistd.h>
#include "MemLogBuilder.h"


extern "C" Stream* CreateCallTranslationStream(const struct pal_stream_attributes *sattr, struct pal_device *dattr,
                                  const uint32_t no_of_devices, const struct modifier_kv *modifiers,
                                  const uint32_t no_of_modifiers, const std::shared_ptr<ResourceManager> rm) {
       return new StreamCallTranslation(sattr, dattr, no_of_devices, modifiers, no_of_modifiers, rm);
}

StreamCallTranslation::StreamCallTranslation(const struct pal_stream_attributes *sattr __unused,
                       struct pal_device *dattr __unused, const uint32_t no_of_devices __unused,
                       const struct modifier_kv *modifiers __unused, const uint32_t no_of_modifiers __unused,
                       const std::shared_ptr<ResourceManager> rm) :
                   StreamCommon(sattr,dattr,no_of_devices,modifiers,no_of_modifiers,rm)
{
    rm->registerStream(this);
}

StreamCallTranslation::~StreamCallTranslation() {
    rm->deregisterStream(this);
}