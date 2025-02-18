/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
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
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2023,2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#ifndef CONTEXT_DETECTION_ENGINE_H
#define CONTEXT_DETECTION_ENGINE_H

#include <condition_variable>
#include <thread>
#include <mutex>
#include <vector>

#include "PalDefs.h"
#include "PalCommon.h"
#include "Device.h"
#include "SoundTriggerUtils.h"
#include "ACDPlatformInfo.h"
#include "PayloadBuilder.h"
#include "SessionAR.h"

typedef enum {
    ENG_IDLE,
    ENG_LOADED,
    ENG_ACTIVE,
    ENG_DETECTED,
} eng_state_t;

class SessionAR;
class StreamACD;
class ACDPlatformInfo;

class ContextDetectionEngine
{
public:
    static std::shared_ptr<ContextDetectionEngine> Create(StreamACD *s,
        std::shared_ptr<ACDStreamConfig> sm_cfg);

    ContextDetectionEngine(StreamACD *s, std::shared_ptr<ACDStreamConfig> sm_cfg);
    virtual ~ContextDetectionEngine();

    virtual int32_t StartEngine(StreamACD *s);
    virtual int32_t StopEngine(StreamACD *s);
    virtual int32_t SetupEngine(StreamACD *s, void *config);
    virtual int32_t TeardownEngine(StreamACD *s, void *config);
    virtual int32_t ReconfigureEngine(StreamACD *s, void *old_config, void *new_config);
    virtual bool isEngActive() { return eng_state_ == ENG_ACTIVE; }

    virtual int32_t ConnectSessionDevice(
        StreamACD* stream_handle,
        pal_stream_type_t stream_type,
        std::shared_ptr<Device> device_to_connect);
    virtual int32_t DisconnectSessionDevice(
        StreamACD* stream_handle,
        pal_stream_type_t stream_type,
        std::shared_ptr<Device> device_to_disconnect);
    virtual int32_t SetupSessionDevice(
        StreamACD* streamHandle,
        pal_stream_type_t streamType,
        std::shared_ptr<Device> deviceToConnect);
    virtual int32_t setECRef(
        StreamACD *s,
        std::shared_ptr<Device> dev,
        bool is_enable);
    int32_t getCustomParam(custom_payload_uc_info_t* uc_info, std::string param_str,
                                    void* param_payload, size_t* payload_size, Stream *s);

private:
    virtual int32_t LoadSoundModel();
    virtual int32_t UnloadSoundModel();
    static void ContextHandleSessionCallBack(uint64_t hdl, uint32_t event_id, void *data,
                                      uint32_t event_size);

protected:
    std::shared_ptr<ACDPlatformInfo> acd_info_;
    std::shared_ptr<ACDStreamConfig> sm_cfg_;
    std::vector<StreamACD *> eng_streams_;
    SessionAR *session_;
    StreamACD *stream_handle_;
    PayloadBuilder *builder_;
    uint32_t sample_rate_;
    uint32_t bit_width_;
    uint32_t channels_;
    int32_t dev_disconnect_count_;

    eng_state_t eng_state_;
    std::thread event_thread_handler_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool exit_thread_;
};

#endif  // CONTEXT_DETECTION_ENGINE_H
