/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef CONFIG_SESSION_ALSA_PCM_H
#define CONFIG_SESSION_ALSA_PCM_H

void handleSilenceDetectionCb(uint64_t hdl __unused,
                uint32_t event_id, void *event_data, uint32_t event_size);
int dump_silence_event_status(char *out_file, uint32_t channel_group, uint32_t status_ch_mask);
int dump_registers(char *in_file_path, char *regdump_out_file);
int dump_kernel_log(char *kmsg_out_file);
int32_t pcmPluginConfigSetConfigStart(Stream* s, void* pluginPayload);
int32_t pcmPluginConfigSetConfigStop(Stream* s, void* pluginPayload);
int register_asps_event(uint32_t reg, SessionAlsaPcm* session, struct mixer* mxr);
int32_t configureInCallRxMFC(SessionAlsaPcm* session, std::shared_ptr<ResourceManager> rm,
                                                                PayloadBuilder* builder);
int reconfigureModule(SessionAlsaPcm* session, PayloadBuilder* builder, uint32_t tagID,
                        const char* BE, struct sessionToPayloadParam *data);
int32_t reconfigureInCallMusicStream(struct pal_media_config config, PayloadBuilder* builder);

#endif