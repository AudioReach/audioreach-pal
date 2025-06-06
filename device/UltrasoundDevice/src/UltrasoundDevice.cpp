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
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "PAL: UltrasoundDevice"
#include "UltrasoundDevice.h"
#include "ResourceManager.h"
#include "Device.h"
#include "kvh2xml.h"

extern "C" void CreateUltrasoundDevice(struct pal_device *device,
                                        const std::shared_ptr<ResourceManager> rm,
                                        std::shared_ptr<Device> *dev) {
    *dev = UltrasoundDevice::getInstance(device, rm);

}
std::shared_ptr<Device> UltrasoundDevice::objRx = nullptr;
std::shared_ptr<Device> UltrasoundDevice::objTx = nullptr;

std::shared_ptr<Device> UltrasoundDevice::getInstance(struct pal_device *device,
                                             std::shared_ptr<ResourceManager> Rm)
{
    if (device->id == PAL_DEVICE_OUT_ULTRASOUND ||
        device->id == PAL_DEVICE_OUT_ULTRASOUND_DEDICATED) {
        if (!objRx) {
            std::lock_guard<std::mutex> lock(Device::mInstMutex);
            if (!objRx) {
                PAL_INFO(LOG_TAG, "%s creating instance for  %d\n", __func__, device->id);
                std::shared_ptr<Device> sp(new UltrasoundDevice(device, Rm));
                objRx = sp;
            }
        }
        return objRx;
    } else {
        if (!objTx) {
            std::lock_guard<std::mutex> lock(Device::mInstMutex);
            if (!objTx) {
                PAL_INFO(LOG_TAG, "%s creating instance for  %d\n", __func__, device->id);
                std::shared_ptr<Device> sp(new UltrasoundDevice(device, Rm));
                objTx = sp;
            }
        }
        return objTx;
    }
}

int32_t UltrasoundDevice::getDeviceConfig(struct pal_device *deviceattr,
                                            struct pal_stream_attributes *sAttr) {

    int32_t status = 0;
    /* copy all config from stream attributes for sensor renderer */
    if (!sAttr) {
        PAL_ERR(LOG_TAG, "Invalid parameter.");
        return -EINVAL;
    }
    if (sAttr->type != PAL_STREAM_SENSOR_PCM_RENDERER)
        return status;

    deviceattr->config.sample_rate = sAttr->out_media_config.sample_rate;
    deviceattr->config.ch_info = sAttr->out_media_config.ch_info;
    deviceattr->config.bit_width = sAttr->out_media_config.bit_width;
    deviceattr->config.aud_fmt_id = bitWidthToFormat.at(sAttr->out_media_config.bit_width);
    PAL_DBG(LOG_TAG, "devcie %d sample rate %d bitwidth %d",
            deviceattr->id, deviceattr->config.sample_rate,
            deviceattr->config.bit_width);
    return status;
}

UltrasoundDevice::UltrasoundDevice(struct pal_device *device, std::shared_ptr<ResourceManager> Rm) :
Device(device, Rm)
{

}

UltrasoundDevice::~UltrasoundDevice()
{

}

