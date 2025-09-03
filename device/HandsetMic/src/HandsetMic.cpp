/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "PAL: HandsetMic"

#include "HandsetMic.h"
#include <tinyalsa/asoundlib.h>
#include "PalAudioRoute.h"
#include "ResourceManager.h"
#include "Device.h"
#include "kvh2xml.h"

extern "C" void CreateHandsetMicDevice(struct pal_device *device,
                                        const std::shared_ptr<ResourceManager> rm,
                                        std::shared_ptr<Device> *dev) {
    *dev = HandsetMic::getInstance(device, rm);

}

std::shared_ptr<Device> HandsetMic::obj = nullptr;

std::shared_ptr<Device> HandsetMic::getInstance(struct pal_device *device,
                                                std::shared_ptr<ResourceManager> Rm)
{
    if (!obj) {
        std::lock_guard<std::mutex> lock(Device::mInstMutex);
        if (!obj) {
            std::shared_ptr<Device> sp(new HandsetMic(device, Rm));
            obj = sp;
        }
    }
    return obj;
}


HandsetMic::HandsetMic(struct pal_device *device, std::shared_ptr<ResourceManager> Rm) :
Device(device, Rm)
{

}

HandsetMic::~HandsetMic()
{

}
