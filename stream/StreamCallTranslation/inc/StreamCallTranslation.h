/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef StreamCallTranslation_H_
#define StreamCallTranslation_H_

#include "Stream.h"
#include "CallTranslationNMTEngine.h"
#include "nmt_module_calibration_api.h"

#define GET_DIR_STR(X) (X == PAL_AUDIO_OUTPUT)? "RX": (X == PAL_AUDIO_INPUT)? "TX": "TX_RX"

struct eventPayload {
    eventType type;
    uint32_t payloadSize;
    void *payload;
};
class ResourceManager;
class Device;
class Session;
class CallTranslationNMTEngine;

extern "C" Stream* CreateCallTranslationStream(const struct pal_stream_attributes *sattr, struct pal_device *dattr,
                                  const uint32_t no_of_devices, const struct modifier_kv *modifiers,
                                  const uint32_t no_of_modifiers, const std::shared_ptr<ResourceManager> rm);

class StreamCallTranslation : public Stream
{
public:
    StreamCallTranslation(const struct pal_stream_attributes *sattr, struct pal_device *dattr,
              const uint32_t no_of_devices,
              const struct modifier_kv *modifiers, const uint32_t no_of_modifiers,
              const std::shared_ptr<ResourceManager> rm);
    ~StreamCallTranslation();
    uint64_t cookie_;
    int32_t open() override;
    int32_t close() override;
    int32_t start() override;
    int32_t stop() override;
    int32_t start_device();
    int32_t startSession();
    int32_t prepare() override { return 0; };
    int32_t setVolume( struct pal_volume_data *volume) override { return 0; };
    int32_t mute(bool state) override { return 0; };
    int32_t mute_l(bool state) override { return 0; };
    int32_t getDeviceMute(pal_stream_direction_t dir __unused, bool *state __unused) override {return 0; };
    int32_t setDeviceMute(pal_stream_direction_t dir __unused, bool state __unused) override {return 0; };
    int32_t pause() override { return 0; };
    int32_t pause_l() override { return 0; };
    int32_t resume() override { return 0; };
    int32_t resume_l() override { return 0; };
    int32_t flush() override { return 0; };
    int32_t drain(pal_drain_type_t type __unused) override { return 0; };
    int32_t suspend() override { return 0; };
    int32_t addRemoveEffect(pal_audio_effect_t effect, bool enable) override { return 0; };
    int32_t read(struct pal_buffer *buf) override { return 0; };
    int32_t write(struct pal_buffer *buf) override { return 0; };
    int32_t registerCallBack(pal_stream_callback cb, uint64_t cookie) override { return 0; };
    int32_t getCallBack(pal_stream_callback *cb) override { return 0; };
    int32_t getParameters(uint32_t param_id, void **payload) override;
    int32_t setParameters(uint32_t param_id, void *payload) override;
    int32_t isSampleRateSupported(uint32_t sampleRate) override { return 0; };
    int32_t isChannelSupported(uint32_t numChannels) override { return 0; };
    int32_t isBitWidthSupported(uint32_t bitWidth) override { return 0; };
    int32_t setECRef(std::shared_ptr<Device> dev, bool is_enable) override { return 0; };
    int32_t setECRef_l(std::shared_ptr<Device> dev, bool is_enable) override { return 0; };
    int32_t ssrDownHandler() override { return 0; };
    int32_t ssrUpHandler() override { return 0; };
    Session* GetSession() {return session; };
    pal_nmt_config GetNmtConfig() { return nmtConfig; };
    param_id_nmt_output_config_t* GetOutputConfig(pal_stream_direction_t dir) { return (dir == PAL_AUDIO_INPUT) ? outputTxConfig : outputRxConfig; }
    bool compareConfig(struct call_translation_config *oldConfig, struct call_translation_config *newConfig);
    int32_t setCallTranslationConfig(struct call_translation_config *payload);
    int32_t setNMTConfig(struct pal_nmt_config *nmt_payload);
    struct call_translation_config *palCallTranslationConfig;
    void HandleEventData(struct eventPayload event);
private:
    bool exitThread;
    Session *session;
    Stream *streamHandle;
    pal_nmt_config nmtConfig;
    param_id_nmt_output_config_t *outputTxConfig;
    param_id_nmt_output_config_t *outputRxConfig;
    std::shared_ptr<CallTranslationNMTEngine> NMTEngine;
};

#endif//StreamCallTranslation_H_
