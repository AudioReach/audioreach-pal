 /*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "PAL: HapticsDev"
#include "HapticsDev.h"
#include "ResourceManager.h"
#include "Device.h"
#include "kvh2xml.h"
#include "HapticsDevProtection.h"

extern "C" void CreateHapticsDevice(struct pal_device *device,
                                    const std::shared_ptr<ResourceManager> rm,
                                    pal_device_id_t id, bool createDevice,
                                    std::shared_ptr<Device> *dev) {
    if (createDevice)
        *dev = HapticsDev::getInstance(device, rm);
    else
        *dev = HapticsDev::getObject();

}

std::shared_ptr<Device> HapticsDev::obj = nullptr;
std::mutex HapticsDev::InstMutex;

std::shared_ptr<Device> HapticsDev::getObject()
{
    std::lock_guard<std::mutex> guard(InstMutex);
    return obj;
}

std::shared_ptr<Device> HapticsDev::getInstance(struct pal_device *device,
                                             std::shared_ptr<ResourceManager> Rm)
{
    std::lock_guard<std::mutex> guard(InstMutex);
    if (!obj) {
        if (ResourceManager::IsHapticsProtectionEnabled() &&
            ResourceManager::IsHapticsThroughWSA()) {
            std::shared_ptr<Device> sp(new HapticsDevProtection(device, Rm));
            obj = sp;
        } else {
            std::shared_ptr<Device> sp(new HapticsDev(device, Rm));
            obj = sp;
        }
    }
    return obj;
}


HapticsDev::HapticsDev(struct pal_device *device, std::shared_ptr<ResourceManager> Rm) :
Device(device, Rm)
{

}

HapticsDev::~HapticsDev()
{

}

int32_t HapticsDev::isSampleRateSupported(uint32_t sampleRate)
{
    int32_t rc = 0;
    PAL_DBG(LOG_TAG, "sampleRate %u", sampleRate);
    switch (sampleRate) {
        case SAMPLINGRATE_48K:
            break;
        default:
            rc = -EINVAL;
            PAL_ERR(LOG_TAG, "sample rate not supported rc %d", rc);
            break;
    }
    return rc;
}

int32_t HapticsDev::isChannelSupported(uint32_t numChannels)
{
    int32_t rc = 0;
    PAL_DBG(LOG_TAG, "numChannels %u", numChannels);
    switch (numChannels) {
        case CHANNELS_1:
            break;
        default:
            rc = -EINVAL;
            PAL_ERR(LOG_TAG, "channels not supported rc %d", rc);
            break;
    }
    return rc;
}

int32_t HapticsDev::isBitWidthSupported(uint32_t bitWidth)
{
    int32_t rc = 0;
    PAL_DBG(LOG_TAG, "bitWidth %u", bitWidth);
    switch (bitWidth) {
        case BITWIDTH_16:
            break;
        default:
            rc = -EINVAL;
            PAL_ERR(LOG_TAG, "bit width not supported rc %d", rc);
            break;
    }
    return rc;
}

int32_t HapticsDev::getDeviceConfig(struct pal_device *deviceattr,
                                    struct pal_stream_attributes *sAttr) {
    /* For PAL_DEVICE_OUT_HAPTICS_DEVICE, copy all config from stream attributes */
    if (!sAttr) {
        PAL_ERR(LOG_TAG, "Invalid parameter.");
        return -EINVAL;
    }
    if (ResourceManager::IsHapticsThroughWSA())
        return 0;

    deviceattr->config.ch_info = sAttr->out_media_config.ch_info;
    deviceattr->config.bit_width = sAttr->out_media_config.bit_width;
    deviceattr->config.aud_fmt_id = sAttr->out_media_config.aud_fmt_id;
    PAL_DBG(LOG_TAG, "device %d sample rate %d bitwidth %d",
            deviceattr->id,deviceattr->config.sample_rate,
            deviceattr->config.bit_width);
    return 0;
}
