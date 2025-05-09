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

 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef STREAMDUMMY_H_
#define STREAMDUMMY_H_

#include "Stream.h"

class ResourceManager;
class Device;
class Session;

extern "C" Stream* CreateDummyStream(const struct pal_stream_attributes *sattr, struct pal_device *dattr,
                               uint32_t instance_id, const std::shared_ptr<ResourceManager> rm);

class StreamDummy : public Stream
{
public:
   StreamDummy(const struct pal_stream_attributes *sattr, struct pal_device *dattr,
             uint32_t instance_id, const std::shared_ptr<ResourceManager> rm);
   ~StreamDummy();
   int32_t open() override {return 0;};
   int32_t close() override {return 0;};
   int32_t start() override {return 0;};
   int32_t stop() override {return 0;};
   int32_t prepare() override {return 0;};
   int32_t setVolume( struct pal_volume_data *volume) override {return 0;};
   int32_t mute(bool state) override {return 0;};
   int32_t mute_l(bool state) override {return 0;};
   int32_t pause() override {return 0;};
   int32_t pause_l() override {return 0;};
   int32_t resume() override {return 0;};
   int32_t resume_l() override {return 0;};
   int32_t flush() override {return 0;};
   int32_t drain(pal_drain_type_t type __unused) override { return 0; };
   int32_t suspend() override { return 0; };
   int32_t addRemoveEffect(pal_audio_effect_t effect, bool enable) override {return 0;};
   int32_t read(struct pal_buffer *buf) override {return 0;};
   int32_t write(struct pal_buffer *buf) override {return 0;};
   int32_t registerCallBack(pal_stream_callback cb, uint64_t cookie) override {return 0;};
   int32_t getCallBack(pal_stream_callback *cb) override {return 0;};
   int32_t getParameters(uint32_t param_id, void **payload) override {return 0;};
   int32_t setParameters(uint32_t param_id, void *payload) override {return 0;};
   int32_t setECRef(std::shared_ptr<Device> dev, bool is_enable) override {return 0;};
   int32_t setECRef_l(std::shared_ptr<Device> dev, bool is_enable) override {return 0;};
   int32_t ssrDownHandler() override {return 0;};
   int32_t ssrUpHandler() override {return 0;};
   int32_t createMmapBuffer(int32_t min_size_frames,
                                   struct pal_mmap_buffer *info) override {return 0;};
   int32_t GetMmapPosition(struct pal_mmap_position *position) override {return 0;};
   int32_t isSampleRateSupported(uint32_t sampleRate __unused) override {return 0;}
   int32_t isChannelSupported(uint32_t numChannels __unused) override {return 0;}
   int32_t isBitWidthSupported(uint32_t bitWidth __unused) override {return 0;}
};

#endif//STREAMDUMMY_H_
