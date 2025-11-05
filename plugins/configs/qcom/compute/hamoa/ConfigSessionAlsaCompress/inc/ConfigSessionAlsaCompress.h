/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#ifndef CONFIG_SESSION_ALSA_COMPRESS_H
#define CONFIG_SESSION_ALSA_COMPRESS_H

extern "C" int compressPluginConfig(Stream* stream, plugin_config_name_t config,
                 void *pluginPayload, size_t pluginPayloadSize);

int32_t compressPluginConfigSetConfigStart(Stream* s, void* pluginConfigPayload);
int32_t compressPluginConfigSetConfigPostStart(Stream* s, void* pluginConfigPayload);
int32_t compressPluginPreReconfig(Stream* s, void* pluginPayload);
int32_t compressPluginConfigSetConfigStop(Stream* s);
int32_t compressPluginConfigSetParam(Stream* s, void* pluginConfigPayload, size_t pluginConfigPayloadSize);
int configureEarlyEOSDelay(PayloadBuilder* builder, struct mixer* mxr,
                std::vector<std::pair<int32_t, std::string>>& rxAifBackEnds,
                        std::vector<int>& compressDevIds);
int setCustomFormatParam(pal_audio_fmt_t audio_fmt, PayloadBuilder* builder, SessionAlsaCompress* session, struct mixer* mxr,
                        std::vector<std::pair<int32_t, std::string>>& rxAifBackEnds, std::vector<int>& compressDevIds);
int compressSilenceDetectionConfig(uint8_t config, pal_device *dAttr,  void * pluginPayload);

#endif
