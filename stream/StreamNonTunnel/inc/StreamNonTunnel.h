/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#ifndef STREAMNONTUNNEL_H_
#define STREAMNONTUNNEL_H_

#include "Stream.h"


extern "C" Stream* CreateNonTunnelStream(const struct pal_stream_attributes *sattr, struct pal_device *dattr,
                               const uint32_t no_of_devices, const struct modifier_kv *modifiers,
                               const uint32_t no_of_modifiers, const std::shared_ptr<ResourceManager> rm);

class StreamNonTunnel : public Stream
{
public:
   StreamNonTunnel(const struct pal_stream_attributes *sattr, struct pal_device *dattr,
             const uint32_t no_of_devices,
             const struct modifier_kv *modifiers, const uint32_t no_of_modifiers,
             const std::shared_ptr<ResourceManager> rm);
   virtual ~StreamNonTunnel();
   int32_t open() override;
   int32_t close() override;
   int32_t start() override;
   int32_t stop() override;
   int32_t prepare() override {return 0;};
   int32_t setStreamAttributes( struct pal_stream_attributes *sattr __unused) {return 0;};
   int32_t setVolume( struct pal_volume_data *volume __unused) {return 0;};
   int32_t mute(bool state __unused) {return 0;};
   int32_t mute_l(bool state __unused) {return 0;};
   int32_t pause() {return 0;};
   int32_t pause_l() {return 0;};
   int32_t resume() {return 0;};
   int32_t resume_l() {return 0;};
   int32_t drain(pal_drain_type_t type) override;
   int32_t flush();
   int32_t suspend() override;
   int32_t setBufInfo(size_t *in_buf_size, size_t in_buf_count,
                       size_t *out_buf_size, size_t out_buf_count);

   int32_t addRemoveEffect(pal_audio_effect_t effect __unused, bool enable __unused) {return 0;};
   int32_t read(struct pal_buffer *buf) override;
   int32_t write(struct pal_buffer *buf) override;
   int32_t registerCallBack(pal_stream_callback cb, uint64_t cookie) override;
   int32_t getCallBack(pal_stream_callback *cb) override;
   int32_t getParameters(uint32_t param_id, void **payload) override;
   int32_t setParameters(uint32_t param_id, void *payload) override;
   int32_t isSampleRateSupported(uint32_t sampleRate) override;
   int32_t isChannelSupported(uint32_t numChannels) override;
   int32_t isBitWidthSupported(uint32_t bitWidth) override;
   bool isStreamSupported() override;
   int32_t setECRef(std::shared_ptr<Device> dev __unused, bool is_enable __unused) {return 0;};
   int32_t setECRef_l(std::shared_ptr<Device> dev __unused, bool is_enable __unused) {return 0;};
   int32_t ssrDownHandler() override;
   int32_t ssrUpHandler() override;
private:
   /*This notifies that the system went through/is in a ssr*/
   bool ssrInNTMode;
};

#endif // STREAMNONTUNNEL_H_
