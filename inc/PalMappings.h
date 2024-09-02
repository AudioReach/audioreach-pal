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
 *
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/** \file pal_defs.h
 *  \brief struture, enum constant defintions of the
 *         PAL(Platform Audio Layer).
 *
 *  This file contains macros, constants, or global variables
 *  exposed to the client of PAL(Platform Audio Layer).
 */

#ifndef PAL_MAPPINGS_H
#define PAL_MAPPINGS_H

#include "PalDefs.h"

#ifdef __cplusplus
#include <iomanip>
#include <map>
#include <sstream>
#include <string>

static const std::map<std::string, pal_audio_fmt_t> PalAudioFormatMap
{
    { "PCM",  PAL_AUDIO_FMT_PCM_S16_LE},
    { "PCM_S8",  PAL_AUDIO_FMT_PCM_S8},
    { "PCM_S16_LE",  PAL_AUDIO_FMT_PCM_S16_LE},
    { "PCM_S24_3LE",  PAL_AUDIO_FMT_PCM_S24_3LE},
    { "PCM_S24_LE",  PAL_AUDIO_FMT_PCM_S24_LE},
    { "PCM_S32_LE",  PAL_AUDIO_FMT_PCM_S32_LE},
    { "MP3",  PAL_AUDIO_FMT_MP3},
    { "AAC",  PAL_AUDIO_FMT_AAC},
    { "AAC_ADTS",  PAL_AUDIO_FMT_AAC_ADTS},
    { "AAC_ADIF",  PAL_AUDIO_FMT_AAC_ADIF},
    { "AAC_LATM",  PAL_AUDIO_FMT_AAC_LATM},
    { "WMA_STD",  PAL_AUDIO_FMT_WMA_STD},
    { "ALAC", PAL_AUDIO_FMT_ALAC},
    { "APE", PAL_AUDIO_FMT_APE},
    { "WMA_PRO", PAL_AUDIO_FMT_WMA_PRO},
    { "FLAC", PAL_AUDIO_FMT_FLAC},
    { "FLAC_OGG", PAL_AUDIO_FMT_FLAC_OGG},
    { "VORBIS", PAL_AUDIO_FMT_VORBIS},
    { "AMR_NB", PAL_AUDIO_FMT_AMR_NB},
    { "AMR_WB", PAL_AUDIO_FMT_AMR_WB},
    { "AMR_WB_PLUS", PAL_AUDIO_FMT_AMR_WB_PLUS},
    { "EVRC", PAL_AUDIO_FMT_EVRC},
    { "G711", PAL_AUDIO_FMT_G711},
    { "QCELP", PAL_AUDIO_FMT_QCELP},
    { "OPUS", PAL_AUDIO_FMT_OPUS}

};

static const std::map<std::string, pal_device_id_t> deviceIdLUT {
    {std::string{ "PAL_DEVICE_OUT_MIN" },                  PAL_DEVICE_OUT_MIN},
    {std::string{ "PAL_DEVICE_NONE" },                     PAL_DEVICE_NONE},
    {std::string{ "PAL_DEVICE_OUT_HANDSET" },              PAL_DEVICE_OUT_HANDSET},
    {std::string{ "PAL_DEVICE_OUT_SPEAKER" },              PAL_DEVICE_OUT_SPEAKER},
    {std::string{ "PAL_DEVICE_OUT_WIRED_HEADSET" },        PAL_DEVICE_OUT_WIRED_HEADSET},
    {std::string{ "PAL_DEVICE_OUT_WIRED_HEADPHONE" },      PAL_DEVICE_OUT_WIRED_HEADPHONE},
    {std::string{ "PAL_DEVICE_OUT_LINE" },                 PAL_DEVICE_OUT_LINE},
    {std::string{ "PAL_DEVICE_OUT_BLUETOOTH_SCO" },        PAL_DEVICE_OUT_BLUETOOTH_SCO},
    {std::string{ "PAL_DEVICE_OUT_BLUETOOTH_A2DP" },       PAL_DEVICE_OUT_BLUETOOTH_A2DP},
    {std::string{ "PAL_DEVICE_OUT_BLUETOOTH_BLE" },        PAL_DEVICE_OUT_BLUETOOTH_BLE},
    {std::string{ "PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST" }, PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST},
    {std::string{ "PAL_DEVICE_OUT_AUX_DIGITAL" },          PAL_DEVICE_OUT_AUX_DIGITAL},
    {std::string{ "PAL_DEVICE_OUT_HDMI" },                 PAL_DEVICE_OUT_HDMI},
    {std::string{ "PAL_DEVICE_OUT_USB_DEVICE" },           PAL_DEVICE_OUT_USB_DEVICE},
    {std::string{ "PAL_DEVICE_OUT_USB_HEADSET" },          PAL_DEVICE_OUT_USB_HEADSET},
    {std::string{ "PAL_DEVICE_OUT_SPDIF" },                PAL_DEVICE_OUT_SPDIF},
    {std::string{ "PAL_DEVICE_OUT_FM" },                   PAL_DEVICE_OUT_FM},
    {std::string{ "PAL_DEVICE_OUT_AUX_LINE" },             PAL_DEVICE_OUT_AUX_LINE},
    {std::string{ "PAL_DEVICE_OUT_PROXY" },                PAL_DEVICE_OUT_PROXY},
    {std::string{ "PAL_DEVICE_OUT_RECORD_PROXY" },         PAL_DEVICE_OUT_RECORD_PROXY},
    {std::string{ "PAL_DEVICE_OUT_AUX_DIGITAL_1" },        PAL_DEVICE_OUT_AUX_DIGITAL_1},
    {std::string{ "PAL_DEVICE_OUT_HEARING_AID" },          PAL_DEVICE_OUT_HEARING_AID},
    {std::string{ "PAL_DEVICE_OUT_HAPTICS_DEVICE" },       PAL_DEVICE_OUT_HAPTICS_DEVICE},
    {std::string{ "PAL_DEVICE_OUT_ULTRASOUND" },           PAL_DEVICE_OUT_ULTRASOUND},
    {std::string{ "PAL_DEVICE_OUT_ULTRASOUND_DEDICATED" }, PAL_DEVICE_OUT_ULTRASOUND_DEDICATED},
    {std::string{ "PAL_DEVICE_OUT_DUMMY" },                PAL_DEVICE_OUT_DUMMY},
    {std::string{ "PAL_DEVICE_OUT_SOUND_DOSE" },           PAL_DEVICE_OUT_SOUND_DOSE},
    {std::string{ "PAL_DEVICE_OUT_BLUETOOTH_HFP" },        PAL_DEVICE_OUT_BLUETOOTH_HFP},
    {std::string{ "PAL_DEVICE_OUT_SPEAKER2" },             PAL_DEVICE_OUT_SPEAKER2},
    {std::string{ "PAL_DEVICE_OUT_SPEAKER3" },             PAL_DEVICE_OUT_SPEAKER3},
    {std::string{ "PAL_DEVICE_OUT_MAX" },                  PAL_DEVICE_OUT_MAX},
    {std::string{ "PAL_DEVICE_IN_HANDSET_MIC" },           PAL_DEVICE_IN_HANDSET_MIC},
    {std::string{ "PAL_DEVICE_IN_SPEAKER_MIC" },           PAL_DEVICE_IN_SPEAKER_MIC},
    {std::string{ "PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET" }, PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET},
    {std::string{ "PAL_DEVICE_IN_WIRED_HEADSET" },         PAL_DEVICE_IN_WIRED_HEADSET},
    {std::string{ "PAL_DEVICE_IN_AUX_DIGITAL" },           PAL_DEVICE_IN_AUX_DIGITAL},
    {std::string{ "PAL_DEVICE_IN_HDMI" },                  PAL_DEVICE_IN_HDMI},
    {std::string{ "PAL_DEVICE_IN_USB_ACCESSORY" },         PAL_DEVICE_IN_USB_ACCESSORY},
    {std::string{ "PAL_DEVICE_IN_USB_DEVICE" },            PAL_DEVICE_IN_USB_DEVICE},
    {std::string{ "PAL_DEVICE_IN_USB_HEADSET" },           PAL_DEVICE_IN_USB_HEADSET},
    {std::string{ "PAL_DEVICE_IN_FM_TUNER" },              PAL_DEVICE_IN_FM_TUNER},
    {std::string{ "PAL_DEVICE_IN_LINE" },                  PAL_DEVICE_IN_LINE},
    {std::string{ "PAL_DEVICE_IN_SPDIF" },                 PAL_DEVICE_IN_SPDIF},
    {std::string{ "PAL_DEVICE_IN_PROXY" },                 PAL_DEVICE_IN_PROXY},
    {std::string{ "PAL_DEVICE_IN_RECORD_PROXY" },          PAL_DEVICE_IN_RECORD_PROXY},
    {std::string{ "PAL_DEVICE_IN_HANDSET_VA_MIC" },        PAL_DEVICE_IN_HANDSET_VA_MIC},
    {std::string{ "PAL_DEVICE_IN_BLUETOOTH_A2DP" },        PAL_DEVICE_IN_BLUETOOTH_A2DP},
    {std::string{ "PAL_DEVICE_IN_BLUETOOTH_BLE" },         PAL_DEVICE_IN_BLUETOOTH_BLE},
    {std::string{ "PAL_DEVICE_IN_HEADSET_VA_MIC" },        PAL_DEVICE_IN_HEADSET_VA_MIC},
    {std::string{ "PAL_DEVICE_IN_VI_FEEDBACK" },           PAL_DEVICE_IN_VI_FEEDBACK},
    {std::string{ "PAL_DEVICE_IN_TELEPHONY_RX" },          PAL_DEVICE_IN_TELEPHONY_RX},
    {std::string{ "PAL_DEVICE_IN_ULTRASOUND_MIC" },        PAL_DEVICE_IN_ULTRASOUND_MIC},
    {std::string{ "PAL_DEVICE_IN_EXT_EC_REF" },            PAL_DEVICE_IN_EXT_EC_REF},
    {std::string{ "PAL_DEVICE_IN_ECHO_REF" },              PAL_DEVICE_IN_ECHO_REF},
    {std::string{ "PAL_DEVICE_IN_HAPTICS_VI_FEEDBACK" },   PAL_DEVICE_IN_HAPTICS_VI_FEEDBACK},
    {std::string{ "PAL_DEVICE_IN_CPS_FEEDBACK" },          PAL_DEVICE_IN_CPS_FEEDBACK},
    {std::string{ "PAL_DEVICE_IN_DUMMY" },                 PAL_DEVICE_IN_DUMMY},
    {std::string{ "PAL_DEVICE_IN_BLUETOOTH_HFP" },         PAL_DEVICE_IN_BLUETOOTH_HFP},
    {std::string{ "PAL_DEVICE_IN_SPEAKER_MIC2" },          PAL_DEVICE_IN_SPEAKER_MIC2},
    {std::string{ "PAL_DEVICE_IN_SPEAKER_MIC3" },          PAL_DEVICE_IN_SPEAKER_MIC3},
};

//reverse mapping
static const std::map<uint32_t, std::string> deviceNameLUT {
    {PAL_DEVICE_OUT_MIN,                  std::string{"PAL_DEVICE_OUT_MIN"}},
    {PAL_DEVICE_NONE,                     std::string{"PAL_DEVICE_NONE"}},
    {PAL_DEVICE_OUT_HANDSET,              std::string{"PAL_DEVICE_OUT_HANDSET"}},
    {PAL_DEVICE_OUT_SPEAKER,              std::string{"PAL_DEVICE_OUT_SPEAKER"}},
    {PAL_DEVICE_OUT_WIRED_HEADSET,        std::string{"PAL_DEVICE_OUT_WIRED_HEADSET"}},
    {PAL_DEVICE_OUT_WIRED_HEADPHONE,      std::string{"PAL_DEVICE_OUT_WIRED_HEADPHONE"}},
    {PAL_DEVICE_OUT_LINE,                 std::string{"PAL_DEVICE_OUT_LINE"}},
    {PAL_DEVICE_OUT_BLUETOOTH_SCO,        std::string{"PAL_DEVICE_OUT_BLUETOOTH_SCO"}},
    {PAL_DEVICE_OUT_BLUETOOTH_A2DP,       std::string{"PAL_DEVICE_OUT_BLUETOOTH_A2DP"}},
    {PAL_DEVICE_OUT_BLUETOOTH_BLE,        std::string{"PAL_DEVICE_OUT_BLUETOOTH_BLE"}},
    {PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST, std::string{"PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST"}},
    {PAL_DEVICE_OUT_AUX_DIGITAL,          std::string{"PAL_DEVICE_OUT_AUX_DIGITAL"}},
    {PAL_DEVICE_OUT_HDMI,                 std::string{"PAL_DEVICE_OUT_HDMI"}},
    {PAL_DEVICE_OUT_USB_DEVICE,           std::string{"PAL_DEVICE_OUT_USB_DEVICE"}},
    {PAL_DEVICE_OUT_USB_HEADSET,          std::string{"PAL_DEVICE_OUT_USB_HEADSET"}},
    {PAL_DEVICE_OUT_SPDIF,                std::string{"PAL_DEVICE_OUT_SPDIF"}},
    {PAL_DEVICE_OUT_FM,                   std::string{"PAL_DEVICE_OUT_FM"}},
    {PAL_DEVICE_OUT_AUX_LINE,             std::string{"PAL_DEVICE_OUT_AUX_LINE"}},
    {PAL_DEVICE_OUT_PROXY,                std::string{"PAL_DEVICE_OUT_PROXY"}},
    {PAL_DEVICE_OUT_RECORD_PROXY,         std::string{"PAL_DEVICE_OUT_RECORD_PROXY"}},
    {PAL_DEVICE_OUT_AUX_DIGITAL_1,        std::string{"PAL_DEVICE_OUT_AUX_DIGITAL_1"}},
    {PAL_DEVICE_OUT_HEARING_AID,          std::string{"PAL_DEVICE_OUT_HEARING_AID"}},
    {PAL_DEVICE_OUT_HAPTICS_DEVICE,       std::string{"PAL_DEVICE_OUT_HAPTICS_DEVICE"}},
    {PAL_DEVICE_OUT_ULTRASOUND,           std::string{"PAL_DEVICE_OUT_ULTRASOUND"}},
    {PAL_DEVICE_OUT_ULTRASOUND_DEDICATED, std::string{"PAL_DEVICE_OUT_ULTRASOUND_DEDICATED"}},
    {PAL_DEVICE_OUT_DUMMY,                std::string{"PAL_DEVICE_OUT_DUMMY"}},
    {PAL_DEVICE_OUT_SOUND_DOSE,           std::string{"PAL_DEVICE_OUT_SOUND_DOSE"}},
    {PAL_DEVICE_OUT_BLUETOOTH_HFP,        std::string{"PAL_DEVICE_OUT_BLUETOOTH_HFP"}},
    {PAL_DEVICE_OUT_SPEAKER2,             std::string{"PAL_DEVICE_OUT_SPEAKER2"}},
    {PAL_DEVICE_OUT_SPEAKER3,             std::string{"PAL_DEVICE_OUT_SPEAKER3"}},
    {PAL_DEVICE_OUT_MAX,                  std::string{"PAL_DEVICE_OUT_MAX"}},
    {PAL_DEVICE_IN_HANDSET_MIC,           std::string{"PAL_DEVICE_IN_HANDSET_MIC"}},
    {PAL_DEVICE_IN_SPEAKER_MIC,           std::string{"PAL_DEVICE_IN_SPEAKER_MIC"}},
    {PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET, std::string{"PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET"}},
    {PAL_DEVICE_IN_WIRED_HEADSET,         std::string{"PAL_DEVICE_IN_WIRED_HEADSET"}},
    {PAL_DEVICE_IN_AUX_DIGITAL,           std::string{"PAL_DEVICE_IN_AUX_DIGITAL"}},
    {PAL_DEVICE_IN_HDMI,                  std::string{"PAL_DEVICE_IN_HDMI"}},
    {PAL_DEVICE_IN_USB_ACCESSORY,         std::string{"PAL_DEVICE_IN_USB_ACCESSORY"}},
    {PAL_DEVICE_IN_USB_DEVICE,            std::string{"PAL_DEVICE_IN_USB_DEVICE"}},
    {PAL_DEVICE_IN_USB_HEADSET,           std::string{"PAL_DEVICE_IN_USB_HEADSET"}},
    {PAL_DEVICE_IN_FM_TUNER,              std::string{"PAL_DEVICE_IN_FM_TUNER"}},
    {PAL_DEVICE_IN_LINE,                  std::string{"PAL_DEVICE_IN_LINE"}},
    {PAL_DEVICE_IN_SPDIF,                 std::string{"PAL_DEVICE_IN_SPDIF"}},
    {PAL_DEVICE_IN_PROXY,                 std::string{"PAL_DEVICE_IN_PROXY"}},
    {PAL_DEVICE_IN_RECORD_PROXY,          std::string{"PAL_DEVICE_IN_RECORD_PROXY"}},
    {PAL_DEVICE_IN_HANDSET_VA_MIC,        std::string{"PAL_DEVICE_IN_HANDSET_VA_MIC"}},
    {PAL_DEVICE_IN_BLUETOOTH_A2DP,        std::string{"PAL_DEVICE_IN_BLUETOOTH_A2DP"}},
    {PAL_DEVICE_IN_BLUETOOTH_BLE,         std::string{"PAL_DEVICE_IN_BLUETOOTH_BLE"}},
    {PAL_DEVICE_IN_HEADSET_VA_MIC,        std::string{"PAL_DEVICE_IN_HEADSET_VA_MIC"}},
    {PAL_DEVICE_IN_VI_FEEDBACK,           std::string{"PAL_DEVICE_IN_VI_FEEDBACK"}},
    {PAL_DEVICE_IN_TELEPHONY_RX,          std::string{"PAL_DEVICE_IN_TELEPHONY_RX"}},
    {PAL_DEVICE_IN_ULTRASOUND_MIC,        std::string{"PAL_DEVICE_IN_ULTRASOUND_MIC"}},
    {PAL_DEVICE_IN_EXT_EC_REF,            std::string{"PAL_DEVICE_IN_EXT_EC_REF"}},
    {PAL_DEVICE_IN_ECHO_REF,              std::string{"PAL_DEVICE_IN_ECHO_REF"}},
    {PAL_DEVICE_IN_HAPTICS_VI_FEEDBACK,   std::string{"PAL_DEVICE_IN_HAPTICS_VI_FEEDBACK"}},
    {PAL_DEVICE_IN_CPS_FEEDBACK,          std::string{"PAL_DEVICE_IN_CPS_FEEDBACK"}},
    {PAL_DEVICE_IN_DUMMY,                 std::string{"PAL_DEVICE_IN_DUMMY"}},
    {PAL_DEVICE_IN_BLUETOOTH_HFP,         std::string{"PAL_DEVICE_IN_BLUETOOTH_HFP"}},
    {PAL_DEVICE_IN_SPEAKER_MIC2,          std::string{"PAL_DEVICE_IN_SPEAKER_MIC2"}},
    {PAL_DEVICE_IN_SPEAKER_MIC3,          std::string{"PAL_DEVICE_IN_SPEAKER_MIC3"}},
};

const std::map<std::string, uint32_t> usecaseIdLUT {
    {std::string{ "PAL_STREAM_LOW_LATENCY" },              PAL_STREAM_LOW_LATENCY},
    {std::string{ "PAL_STREAM_DEEP_BUFFER" },              PAL_STREAM_DEEP_BUFFER},
    {std::string{ "PAL_STREAM_COMPRESSED" },               PAL_STREAM_COMPRESSED},
    {std::string{ "PAL_STREAM_VOIP" },                     PAL_STREAM_VOIP},
    {std::string{ "PAL_STREAM_VOIP_RX" },                  PAL_STREAM_VOIP_RX},
    {std::string{ "PAL_STREAM_VOIP_TX" },                  PAL_STREAM_VOIP_TX},
    {std::string{ "PAL_STREAM_VOICE_CALL_MUSIC" },         PAL_STREAM_VOICE_CALL_MUSIC},
    {std::string{ "PAL_STREAM_GENERIC" },                  PAL_STREAM_GENERIC},
    {std::string{ "PAL_STREAM_RAW" },                      PAL_STREAM_RAW},
    {std::string{ "PAL_STREAM_VOICE_RECOGNITION" },        PAL_STREAM_VOICE_RECOGNITION},
    {std::string{ "PAL_STREAM_VOICE_CALL_RECORD" },        PAL_STREAM_VOICE_CALL_RECORD},
    {std::string{ "PAL_STREAM_VOICE_CALL_TX" },            PAL_STREAM_VOICE_CALL_TX},
    {std::string{ "PAL_STREAM_VOICE_CALL_RX_TX" },         PAL_STREAM_VOICE_CALL_RX_TX},
    {std::string{ "PAL_STREAM_VOICE_CALL" },               PAL_STREAM_VOICE_CALL},
    {std::string{ "PAL_STREAM_LOOPBACK" },                 PAL_STREAM_LOOPBACK},
    {std::string{ "PAL_STREAM_TRANSCODE" },                PAL_STREAM_TRANSCODE},
    {std::string{ "PAL_STREAM_VOICE_UI" },                 PAL_STREAM_VOICE_UI},
    {std::string{ "PAL_STREAM_PCM_OFFLOAD" },              PAL_STREAM_PCM_OFFLOAD},
    {std::string{ "PAL_STREAM_ULTRA_LOW_LATENCY" },        PAL_STREAM_ULTRA_LOW_LATENCY},
    {std::string{ "PAL_STREAM_PROXY" },                    PAL_STREAM_PROXY},
    {std::string{ "PAL_STREAM_NON_TUNNEL" },               PAL_STREAM_NON_TUNNEL},
    {std::string{ "PAL_STREAM_HAPTICS" },                  PAL_STREAM_HAPTICS},
    {std::string{ "PAL_STREAM_ACD" },                      PAL_STREAM_ACD},
    {std::string{ "PAL_STREAM_ASR" },                      PAL_STREAM_ASR},
    {std::string{ "PAL_STREAM_ULTRASOUND" },               PAL_STREAM_ULTRASOUND},
    {std::string{ "PAL_STREAM_SENSOR_PCM_DATA" },          PAL_STREAM_SENSOR_PCM_DATA},
    {std::string{ "PAL_STREAM_SPATIAL_AUDIO" },            PAL_STREAM_SPATIAL_AUDIO},
    {std::string{ "PAL_STREAM_CONTEXT_PROXY" },            PAL_STREAM_CONTEXT_PROXY},
    {std::string{ "PAL_STREAM_COMMON_PROXY" },             PAL_STREAM_COMMON_PROXY},
    {std::string{ "PAL_STREAM_SENSOR_PCM_RENDERER" },      PAL_STREAM_SENSOR_PCM_RENDERER},
    {std::string{ "PAL_STREAM_ASR" },                      PAL_STREAM_ASR},
    {std::string{ "PAL_STREAM_HPCM" },                     PAL_STREAM_HPCM},
    {std::string{ "PAL_STREAM_DUMMY" },                    PAL_STREAM_DUMMY},
    {std::string{ "PAL_STREAM_CALL_TRANSLATION" },         PAL_STREAM_CALL_TRANSLATION},
    {std::string{ "PAL_STREAM_PLAYBACK_BUS" },             PAL_STREAM_PLAYBACK_BUS},
    {std::string{ "PAL_STREAM_CAPTURE_BUS" },              PAL_STREAM_CAPTURE_BUS},
};

/* Update the reverse mapping as well when new stream is added */
const std::map<uint32_t, std::string> streamNameLUT {
    {PAL_STREAM_LOW_LATENCY,        std::string{ "PAL_STREAM_LOW_LATENCY" } },
    {PAL_STREAM_DEEP_BUFFER,        std::string{ "PAL_STREAM_DEEP_BUFFER" } },
    {PAL_STREAM_COMPRESSED,         std::string{ "PAL_STREAM_COMPRESSED" } },
    {PAL_STREAM_VOIP,               std::string{ "PAL_STREAM_VOIP" } },
    {PAL_STREAM_VOIP_RX,            std::string{ "PAL_STREAM_VOIP_RX" } },
    {PAL_STREAM_VOIP_TX,            std::string{ "PAL_STREAM_VOIP_TX" } },
    {PAL_STREAM_VOICE_CALL_MUSIC,   std::string{ "PAL_STREAM_VOICE_CALL_MUSIC" } },
    {PAL_STREAM_GENERIC,            std::string{ "PAL_STREAM_GENERIC" } },
    {PAL_STREAM_RAW,                std::string{ "PAL_STREAM_RAW" } },
    {PAL_STREAM_VOICE_RECOGNITION,  std::string{ "PAL_STREAM_VOICE_RECOGNITION" } },
    {PAL_STREAM_VOICE_CALL_RECORD,  std::string{ "PAL_STREAM_VOICE_CALL_RECORD" } },
    {PAL_STREAM_VOICE_CALL_TX,      std::string{ "PAL_STREAM_VOICE_CALL_TX" } },
    {PAL_STREAM_VOICE_CALL_RX_TX,   std::string{ "PAL_STREAM_VOICE_CALL_RX_TX" } },
    {PAL_STREAM_VOICE_CALL,         std::string{ "PAL_STREAM_VOICE_CALL" } },
    {PAL_STREAM_LOOPBACK,           std::string{ "PAL_STREAM_LOOPBACK" } },
    {PAL_STREAM_TRANSCODE,          std::string{ "PAL_STREAM_TRANSCODE" } },
    {PAL_STREAM_VOICE_UI,           std::string{ "PAL_STREAM_VOICE_UI" } },
    {PAL_STREAM_PCM_OFFLOAD,        std::string{ "PAL_STREAM_PCM_OFFLOAD" } },
    {PAL_STREAM_ULTRA_LOW_LATENCY,  std::string{ "PAL_STREAM_ULTRA_LOW_LATENCY" } },
    {PAL_STREAM_PROXY,              std::string{ "PAL_STREAM_PROXY" } },
    {PAL_STREAM_NON_TUNNEL,         std::string{ "PAL_STREAM_NON_TUNNEL" } },
    {PAL_STREAM_HAPTICS,            std::string{ "PAL_STREAM_HAPTICS" } },
    {PAL_STREAM_CONTEXT_PROXY,      std::string{ "PAL_STREAM_CONTEXT_PROXY" } },
    {PAL_STREAM_ACD,                std::string{ "PAL_STREAM_ACD" } },
    {PAL_STREAM_ASR,                std::string{ "PAL_STREAM_ASR" } },
    {PAL_STREAM_ULTRASOUND,         std::string{ "PAL_STREAM_ULTRASOUND" } },
    {PAL_STREAM_SENSOR_PCM_DATA,    std::string{ "PAL_STREAM_SENSOR_PCM_DATA" } },
    {PAL_STREAM_SPATIAL_AUDIO,      std::string{ "PAL_STREAM_SPATIAL_AUDIO" } },
    {PAL_STREAM_COMMON_PROXY,       std::string{ "PAL_STREAM_COMMON_PROXY" } },
    {PAL_STREAM_SENSOR_PCM_RENDERER,std::string{ "PAL_STREAM_SENSOR_PCM_RENDERER" } },
    {PAL_STREAM_ASR,                std::string{ "PAL_STREAM_ASR" } },
    {PAL_STREAM_HPCM,               std::string{ "PAL_STREAM_HPCM" } },
    {PAL_STREAM_DUMMY,              std::string{ "PAL_STREAM_DUMMY" } },
    {PAL_STREAM_CALL_TRANSLATION,   std::string{ "PAL_STREAM_CALL_TRANSLATION" } },
    {PAL_STREAM_PLAYBACK_BUS,       std::string{ "PAL_STREAM_PLAYBACK_BUS" } },
    {PAL_STREAM_CAPTURE_BUS,        std::string{ "PAL_STREAM_CAPTURE_BUS" } },
};

const std::map<uint32_t, std::string> vsidLUT {
    {VOICEMMODE1,    std::string{ "VOICEMMODE1" } },
    {VOICEMMODE2,    std::string{ "VOICEMMODE2" } },
    {VOICELBMMODE1,  std::string{ "VOICELBMMODE1" } },
    {VOICELBMMODE2,  std::string{ "VOICELBMMODE2" } },
};

const std::map<uint32_t, std::string> loopbackLUT {
    {PAL_STREAM_LOOPBACK_PCM,           std::string{ "PAL_STREAM_LOOPBACK_PCM" } },
    {PAL_STREAM_LOOPBACK_HFP_RX,        std::string{ "PAL_STREAM_LOOPBACK_HFP_RX" } },
    {PAL_STREAM_LOOPBACK_HFP_TX,        std::string{ "PAL_STREAM_LOOPBACK_HFP_TX" } },
    {PAL_STREAM_LOOPBACK_COMPRESS,      std::string{ "PAL_STREAM_LOOPBACK_COMPRESS" } },
    {PAL_STREAM_LOOPBACK_FM,            std::string{ "PAL_STREAM_LOOPBACK_FM" } },
    {PAL_STREAM_LOOPBACK_KARAOKE,       std::string{ "PAL_STREAM_LOOPBACK_KARAOKE" }},
    {PAL_STREAM_LOOPBACK_PLAYBACK_ONLY, std::string{ "PAL_STREAM_LOOPBACK_PLAYBACK_ONLY" } },
    {PAL_STREAM_LOOPBACK_CAPTURE_ONLY,  std::string{ "PAL_STREAM_LOOPBACK_CAPTURE_ONLY" } },
    {PAL_STREAM_LOOPBACK_ICC,           std::string{ "PAL_STREAM_LOOPBACK_ICC" } },
};

const std::map<uint32_t, std::string> hapticsLUT {
    {PAL_STREAM_HAPTICS_TOUCH,        std::string{ "PAL_STREAM_HAPTICS_TOUCH" } },
    {PAL_STREAM_HAPTICS_RINGTONE,     std::string{ "PAL_STREAM_HAPTICS_RINGTONE" } },
};

enum class PalAddressTag { ID, MAC, IPv4, IPv6, ALSA };

static PalAddressTag getAddressTag(const pal_device_id_t deviceId) {
    // don't have cases for ipv4/ ipv6 devices, add once have exact usecases.
    switch (deviceId) {
        case PAL_DEVICE_OUT_USB_DEVICE:
        case PAL_DEVICE_OUT_USB_HEADSET:
        case PAL_DEVICE_IN_USB_ACCESSORY:
        case PAL_DEVICE_IN_USB_DEVICE:
        case PAL_DEVICE_IN_USB_HEADSET:
            return PalAddressTag::ALSA;
        case PAL_DEVICE_OUT_BLUETOOTH_SCO:
        case PAL_DEVICE_OUT_BLUETOOTH_A2DP:
        case PAL_DEVICE_OUT_HEARING_AID:
        case PAL_DEVICE_OUT_BLUETOOTH_BLE:
        case PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST:
        case PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET:
        case PAL_DEVICE_IN_BLUETOOTH_A2DP:
        case PAL_DEVICE_IN_BLUETOOTH_BLE:
            return PalAddressTag::MAC;
        default:
            return PalAddressTag::ID;
    }
}

static std::string toString(const pal_device* device) {
    std::ostringstream oss;
    auto tag = getAddressTag(device->id);
    if (deviceNameLUT.find(device->id) != deviceNameLUT.end()) {
        oss << " " << deviceNameLUT.at(device->id);
    } else {
        oss << " Unknown Device";
        return oss.str();
    }

    oss << " Address ( ";
    switch (tag) {
        case PalAddressTag::ID:
            oss << "id: " << device->addressV1.id;
            break;
        case PalAddressTag::MAC:
            oss << "mac: ";
            for (int i = 0; i < 6; ++i) {
                oss << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<int>(device->addressV1.mac[i]);
                if (i < 5) oss << ":";
            }
            oss << std::dec;
            break;
        case PalAddressTag::IPv4:
            oss << " ipv4: ";
            for (int i = 0; i < 4; ++i) {
                oss << static_cast<int>(device->addressV1.ipv4[i]);
                if (i < 3) oss << ".";
            }

            break;
        case PalAddressTag::IPv6:
            oss << " ipv6: ";
            for (int i = 0; i < 8; ++i) {
                oss << std::hex << device->addressV1.ipv6[i];
                if (i < 7) oss << ":";
            }
            oss << std::dec;
            break;
        case PalAddressTag::ALSA:
            oss << " alsa: ";
            for (int i = 0; i < 2; ++i) {
                oss << device->addressV1.alsa[i];
                if (i < 1) oss << ":";
            }
            break;
    }

    oss << ")";
    return oss.str();
}

#endif
#endif /*PAL_MAPPINGS_H*/
