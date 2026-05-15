/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef CONFIG_SESSION_ALSA_VOICE_H
#define CONFIG_SESSION_ALSA_VOICE_H

extern "C" int voicePluginConfig(Stream* stream, plugin_config_name_t config,
                 void *pluginPayload, size_t ppldSize);

int populateRatPayload(Stream* s, SessionAlsaVoice *session, PayloadBuilder* builder);
int setPopSuppressorMute(Stream* s);
int getDeviceData(Stream* s, struct sessionToPayloadParam* deviceData);
int build_rx_mfc_payload(Stream* s, PayloadBuilder* builder);
int populate_rx_mfc_payload(Stream* s, uint32_t rx_mfc_tag, PayloadBuilder* builder);
int populate_rx_mfc_coeff_payload(std::shared_ptr<Device> CrsDevice, SessionAlsaVoice* session,
                                    PayloadBuilder* builder, std::vector<int>& pcmDevRxIds,
                                    std::shared_ptr<ResourceManager> rm);
int setTaggedSlotMask(Stream* s, std::shared_ptr<ResourceManager> rm, std::vector<int> pcmDevRxIds);
int populate_vsid_payload(Stream* s, PayloadBuilder* builder, uint32_t vsid);
int populate_ch_info_payload(Stream* s, PayloadBuilder* builder, uint32_t vsid);
int configVSID(Stream *s, SessionAlsaVoice *session, configType type __unused,
                uint32_t vsid, int dir, PayloadBuilder* builder);
int payloadSetVSID(Stream* s, PayloadBuilder* builder, uint32_t vsid);
int populateVSIDLoopbackPayload(Stream* s, PayloadBuilder* builder, uint32_t vsid);
int reconfigureSession(Stream* s, PayloadBuilder* builder,
                        uint32_t vsid, pal_stream_direction_t dir);
int voiceSilenceDetectionConfig(uint8_t config, pal_device *dAttr, void * pluginPayload);
int32_t voicePluginConfigSetConfigStart(Stream* s, void* pluginPayload);
int32_t voicePluginConfigSetConfigStop(Stream* s, void * pluginPayload);
int32_t voicePreCommonReconfig(Stream* stream);
int32_t voicePostCommonReconfig(Stream* s, void* pluginPayload);
int32_t voicePostReconfig(Stream* s, void* pluginPayload);
int32_t voicePluginConfigSetConfigPostStart(Stream* s, void* pluginPayload);
int32_t voicePluginReconfig(Stream* s, void* pluginPayload);
int32_t voicePluginPreReconfig(Stream* s, void* pluginPayload);
int32_t rxMFCCoeffConfig(Stream* s, void* pluginPayload);

#endif
