/*
 *Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "PAL: A2BSpeaker"
#include "A2BSpeaker.h"
#include "ResourceManager.h"
#include "Device.h"
#include "kvh2xml.h"

extern "C" void Createa2bspeakerDevice(struct pal_device *device,
                                       const std::shared_ptr<ResourceManager> rm,
                                       pal_device_id_t id, bool createDevice,
                                       std::shared_ptr<Device> *dev) {
    if (createDevice)
        *dev = A2BSpeaker::getInstance(device, rm);
    else
        *dev = A2BSpeaker::getObject();

}

std::shared_ptr<Device> A2BSpeaker::obj = nullptr;

std::shared_ptr<Device> A2BSpeaker::getObject()
{
    return obj;
}

std::shared_ptr<Device> A2BSpeaker::getInstance(struct pal_device *device,
                                                std::shared_ptr<ResourceManager> Rm)
{
    if (!obj) {
        std::shared_ptr<Device> sp(new A2BSpeaker(device, Rm));
        obj = sp;
    }
    return obj;
}

A2BSpeaker::A2BSpeaker(struct pal_device *device, std::shared_ptr<ResourceManager> Rm) :
Device(device, Rm)
{
}

A2BSpeaker::A2BSpeaker()
{

}

A2BSpeaker::~A2BSpeaker()
{

}

int32_t A2BSpeaker::isSampleRateSupported(uint32_t sampleRate)
{
    int32_t rc = 0;
    PAL_DBG(LOG_TAG, "sampleRate %u", sampleRate);
    switch (sampleRate) {
        case SAMPLINGRATE_48K:
        case SAMPLINGRATE_96K:
            break;
        default:
            rc = -EINVAL;
            PAL_ERR(LOG_TAG, "sample rate not supported rc %d", rc);
            break;
    }
    return rc;
}

int32_t A2BSpeaker::isChannelSupported(uint32_t numChannels)
{
    int32_t rc = 0;
    PAL_DBG(LOG_TAG, "numChannels %u", numChannels);
    switch (numChannels) {
        case CHANNELS_1:
        case CHANNELS_2:
        case CHANNELS_3:
        case CHANNELS_4:
        case CHANNELS_8:
            break;
        default:
            rc = -EINVAL;
            PAL_ERR(LOG_TAG, "channels not supported rc %d", rc);
            break;
    }
    return rc;
}

int32_t A2BSpeaker::isBitWidthSupported(uint32_t bitWidth)
{
    int32_t rc = 0;
    PAL_DBG(LOG_TAG, "bitWidth %u", bitWidth);
    switch (bitWidth) {
        case BITWIDTH_16:
        case BITWIDTH_24:
        case BITWIDTH_32:
            break;
        default:
            rc = -EINVAL;
            PAL_ERR(LOG_TAG, "bit width not supported rc %d", rc);
            break;
    }
    return rc;
}

int A2BSpeaker::stop()
{
    int status = 0;

    status = Device::stop();
    if (status == 0 && deviceCount == 0) {
        std::shared_ptr<ResourceManager> Rm = nullptr;
        Rm = ResourceManager::getInstance();
        if (Rm && Rm->IsChargeConcurrencyEnabled() && Rm->getChargerOnlineState() &&
            Rm->getConcurrentBoostState()) {
            status = Rm->chargerListenerSetBoostState(false, CONCURRENCY_PB_STOPS);
            if (0 != status)
                PAL_ERR(LOG_TAG, "Failed to notify PMIC: %d", status);
        } else {
            PAL_DBG(LOG_TAG, "Concurrent State unchanged, ignore notifying");
        }
    }
    return status;
}
