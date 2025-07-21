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

#ifndef ASR_PLATFORM_INFO_H
#define ASR_PLATFORM_INFO_H

#define OUT_BUF_SIZE_DEFAULT 10000 /* In bytes. Around 98sec of text */
#define VAD_HANG_OVER_DURTION_DEFAULT_MS 1000

#include "ResourceManager.h"

typedef enum asr_param_id_type {
    ASR_INPUT_CONFIG = 0,
    ASR_OUTPUT_CONFIG,
    ASR_INPUT_BUF_DURATON,
    ASR_OUTPUT,
    ASR_FORCE_OUTPUT,
    SDZ_ENABLE,
    SDZ_OUTPUT_CONFIG,
    SDZ_INPUT_BUF_DURATION,
    SDZ_OUTPUT,
    SDZ_FORCE_OUTPUT,
    ASR_ABORT_EVENT,
    ASR_MAX_PARAM_IDS
}asr_param_id_type_t;

class ASRCommonConfig : public SoundTriggerXml
{
public:
    ASRCommonConfig();

    void HandleStartTag(const std::string& tag, const char **attribs);
    void HandleEndTag(struct xml_userdata *data, const std::string& tag);

    bool PartialModeInLpiSupported() const { return partial_mode_in_lpi_; }
    size_t GetInputBufferSize() const { return input_buffer_size_; }
    size_t GetPartialModeInputBufferSize() const { return partial_mode_input_buffer_size_; }
    size_t GetBufferingModeOutBufferSize() const { return buffering_mode_out_buffer_size_; }
    uint32_t GetSdzOutputBufferSize() const { return sdz_output_buffer_size_; }
    uint32_t GetInputBufferSize(int mode);
    uint32_t GetOutputBufferSize(int mode);

private:
    bool partial_mode_in_lpi_;
    size_t input_buffer_size_;
    size_t partial_mode_input_buffer_size_;
    size_t buffering_mode_out_buffer_size_;
    size_t sdz_output_buffer_size_;
};

class ASRStreamConfig : public SoundTriggerXml
{
public:
    ASRStreamConfig();
    ASRStreamConfig(ACDStreamConfig &rhs) = delete;
    ASRStreamConfig & operator=(ACDStreamConfig &rhs) = delete;

    void HandleStartTag(const std::string& tag, const char **attribs);
    void HandleEndTag(struct xml_userdata *data, const std::string& tag);

    std::string GetStreamConfigName() const { return name_; }
    uint32_t GetModuleTagId(asr_param_id_type_t param_id) const {
        return module_tag_ids_[param_id];
    }
    uint32_t GetParamId(asr_param_id_type_t param_id) const {
        return param_ids_[param_id];
    }
    std::shared_ptr<CaptureProfile> GetCaptureProfile(
        std::pair<StOperatingModes, StInputModes> mode_pair) const {
        return asr_op_modes_.at(mode_pair);
    }
    UUID GetUUID() const { return vendor_uuid_; }
    bool GetStreamLPIFlag() const { return lpi_enable_; }
    int32_t GetIndex(std::string& param_name);

private:
    std::string name_;
    st_op_modes_t asr_op_modes_;
    UUID vendor_uuid_;
    bool lpi_enable_;
    std::shared_ptr<SoundTriggerXml> curr_child_;
    uint32_t module_tag_ids_[ASR_MAX_PARAM_IDS];
    uint32_t param_ids_[ASR_MAX_PARAM_IDS];
};

class ASRPlatformInfo : public SoundTriggerPlatformInfo
{
public:
    ASRPlatformInfo();
    ASRPlatformInfo(ASRStreamConfig &rhs) = delete;
    ASRPlatformInfo & operator=(ASRPlatformInfo &rhs) = delete;

    void HandleStartTag(const std::string& tag, const char **attribs) override;
    void HandleEndTag(struct xml_userdata *data, const std::string& tag) override;

    static std::shared_ptr<ASRPlatformInfo> GetInstance();
    std::shared_ptr<ASRStreamConfig> GetStreamConfig(const UUID& uuid) const;
    std::shared_ptr<ASRCommonConfig> GetCommonConfig() const { return cm_cfg_; }

private:
    static std::shared_ptr<ASRPlatformInfo> me_;
    std::map<UUID, std::shared_ptr<ASRStreamConfig>> stream_cfg_list_;
    std::shared_ptr<SoundTriggerXml> curr_child_;
    std::shared_ptr<ASRCommonConfig> cm_cfg_;
};
#endif
