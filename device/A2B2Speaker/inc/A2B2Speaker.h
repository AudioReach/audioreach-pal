/*
 *Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef A2B2SPEAKER_H
#define A2B2SPEAKER_H

#include "Device.h"
#include "PalAudioRoute.h"

class A2B2Speaker : public Device
{
protected:
    static std::shared_ptr<Device> obj;
    A2B2Speaker(struct pal_device *device, std::shared_ptr<ResourceManager> Rm);
public:
    static std::shared_ptr<Device> getInstance(struct pal_device *device,
                                               std::shared_ptr<ResourceManager> Rm);
    int32_t isSampleRateSupported(uint32_t sampleRate);
    int32_t isChannelSupported(uint32_t numChannels);
    int32_t isBitWidthSupported(uint32_t bitWidth);
    static std::shared_ptr<Device> getObject();
    int stop();
    A2B2Speaker();
    virtual ~A2B2Speaker();
};

#endif // A2B2SPEAKER_H
