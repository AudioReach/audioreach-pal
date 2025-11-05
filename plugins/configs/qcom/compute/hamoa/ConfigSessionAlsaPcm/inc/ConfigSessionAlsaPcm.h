/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#ifndef CONFIG_SESSION_ALSA_PCM_H
#define CONFIG_SESSION_ALSA_PCM_H

extern "C" int pcmPluginConfig(Stream* stream, plugin_config_name_t config,
                 void *pluginPayload, size_t ppldSize);

void handleSilenceDetectionCb(uint64_t hdl __unused,
                uint32_t event_id, void *event_data, uint32_t event_size);
int dump_silence_event_status(char *out_file, uint32_t channel_group, uint32_t status_ch_mask);
int dump_registers(char *in_file_path, char *regdump_out_file);
int dump_kernel_log(char *kmsg_out_file);
int pcmSilenceDetectionConfig(uint8_t config, pal_device *dAttr,  void * pluginPayload);
int32_t pcmPluginConfigSetConfigStart(Stream* s, void* pluginPayload);
int32_t pcmPluginConfigSetConfigStop(Stream* s, void* pluginPayload);
int32_t pcmPluginPreReconfig(Stream* s, void* pluginPayload);
int register_asps_event(uint32_t reg, SessionAlsaPcm* session, struct mixer* mxr);
int32_t configureCallTranslationModules(Stream* s, PayloadBuilder* builder, struct mixer *mxr, SessionAlsaPcm* session, std::shared_ptr<ResourceManager> rm);
int32_t configureCallTranslationRxDeviceMFC(PayloadBuilder* builder, struct mixer *mxr, SessionAlsaPcm* session, std::shared_ptr<ResourceManager> rm);
int32_t configureInCallRxMFC(SessionAlsaPcm* session, std::shared_ptr<ResourceManager> rm,
                                                                PayloadBuilder* builder);
int reconfigureModule(SessionAlsaPcm* session, PayloadBuilder* builder, uint32_t tagID,
                        const char* BE, struct sessionToPayloadParam *data);
int32_t reconfigureInCallMusicStream(struct pal_media_config config, PayloadBuilder* builder);

#endif
