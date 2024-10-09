/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
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
 */

#define LOG_TAG "PAL: FMDevice"
#include "ResourceManager.h"
#include "FMDevice.h"
#include "Device.h"
#include "kvh2xml.h"

extern "C" void CreateFmDevice(struct pal_device *device,
                                const std::shared_ptr<ResourceManager> rm,
                                pal_device_id_t id, bool createDevice,
                                std::shared_ptr<Device> *dev) {
    *dev = FMDevice::getInstance(device, rm);
}

std::shared_ptr<Device> FMDevice::obj = nullptr;

std::shared_ptr<Device> FMDevice::getInstance(struct pal_device *device,
                                             std::shared_ptr<ResourceManager> Rm)
{
    if (!obj){
        std::shared_ptr<Device> sp(new FMDevice(device, Rm));
        obj = sp;
    }
    return obj;
}

int32_t FMDevice::isSampleRateSupported(uint32_t sampleRate)
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

int32_t FMDevice::isChannelSupported(uint32_t numChannels)
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

int32_t FMDevice::isBitWidthSupported(uint32_t bitWidth)
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

int32_t FMDevice::getDeviceConfig(struct pal_device *deviceattr,
                                  struct pal_stream_attributes *sAttr) {
    int32_t status = 0;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    if (deviceattr->id == PAL_DEVICE_IN_FM_TUNER) {
        /* For PAL_DEVICE_IN_FM_TUNER, copy all config from stream attributes */
        if (!sAttr) {
            PAL_ERR(LOG_TAG, "Invalid parameter.");
            return -EINVAL;
        }

        struct pal_media_config *candidateConfig = &sAttr->in_media_config;
        PAL_DBG(LOG_TAG, "sattr chn=0x%x fmt id=0x%x rate = 0x%x width=0x%x",
            sAttr->in_media_config.ch_info.channels,
            sAttr->in_media_config.aud_fmt_id,
            sAttr->in_media_config.sample_rate,
            sAttr->in_media_config.bit_width);

        if (!rm->ifVoiceorVoipCall(sAttr->type) && rm->isDeviceAvailable(PAL_DEVICE_OUT_PROXY)) {
            PAL_DBG(LOG_TAG, "This is NOT voice call. out proxy is available");
            std::shared_ptr<Device> devOut = nullptr;
            struct pal_device proxyOut_dattr;
            proxyOut_dattr.id = PAL_DEVICE_OUT_PROXY;
            devOut = Device::getInstance(&proxyOut_dattr, rm);
            if (devOut) {
                status = devOut->getDeviceAttributes(&proxyOut_dattr);
                if (status) {
                    PAL_ERR(LOG_TAG, "getDeviceAttributes for OUT_PROXY failed %d", status);
                    return status;
                }

                if (proxyOut_dattr.config.ch_info.channels &&
                        proxyOut_dattr.config.sample_rate) {
                    PAL_INFO(LOG_TAG, "proxy out attr is used");
                    candidateConfig = &proxyOut_dattr.config;
                }
            }
        }
        deviceattr->config.ch_info = candidateConfig->ch_info;
        if (isPalPCMFormat(candidateConfig->aud_fmt_id))
            deviceattr->config.bit_width =
                      rm->palFormatToBitwidthLookup(candidateConfig->aud_fmt_id);
        else
            deviceattr->config.bit_width = candidateConfig->bit_width;

        deviceattr->config.aud_fmt_id = candidateConfig->aud_fmt_id;
        deviceattr->config.sample_rate = candidateConfig->sample_rate;

        PAL_INFO(LOG_TAG, "in proxy chn=0x%x fmt id=0x%x rate = 0x%x width=0x%x",
                    deviceattr->config.ch_info.channels,
                    deviceattr->config.aud_fmt_id,
                    deviceattr->config.sample_rate,
                    deviceattr->config.bit_width);
    }
    return status;
}
