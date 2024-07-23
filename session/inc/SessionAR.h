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
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef SESSION_AR_H
#define SESSION_AR_H

#include "PayloadBuilder.h"
#include "Session.h"
#include "PalDefs.h"
#include <mutex>
#include <algorithm>
#include <vector>
#include <string.h>
#include <stdlib.h>
#include <memory>
#include <errno.h>
#include "PalCommon.h"
#include "Device.h"

typedef enum {
    PM_QOS_VOTE_DISABLE = 0,
    PM_QOS_VOTE_ENABLE  = 1
} pmQosVote;

#define MSPP_SOFT_PAUSE_DELAY 150
#define DEFAULT_RAMP_PERIOD 0x28

class Stream;
class ResourceManager;
class SessionAR : public Session
{
protected:
    std::mutex kvMutex;
    PayloadBuilder* builder;
    struct mixer *mixer;
    static struct pcm *pcmEcTx;
    static std::vector<int> pcmDevEcTxIds;
    static int extECRefCnt;
    static std::mutex extECMutex;
    pal_device_id_t ecRefDevId;
    int32_t setInitialVolume();
public:
    static Session* makeARSession(const std::shared_ptr<ResourceManager>& rm, const struct pal_stream_attributes *sAttr);
    static Session* makeACDBSession(const std::shared_ptr<ResourceManager>& rm, const struct pal_stream_attributes *sAttr);
    int HDRConfigKeyToDevOrientation(const char* hdr_custom_key);
    void setPmQosMixerCtl(pmQosVote vote);
    virtual int pause(Stream * s){return 0;};
    virtual int suspend(Stream * s){return 0;};
    virtual int resume(Stream * s){return 0;};
    virtual int drain(pal_drain_type_t type __unused){return 0;};
    virtual int flush(){return 0;};
    virtual int read(Stream *s __unused, struct pal_buffer *buf __unused, int * size __unused) {return 0;};
    virtual int write(Stream *s __unused, struct pal_buffer *buf __unused, int * size __unused) {return 0;};
    virtual int getTimestamp(struct pal_session_time *stime __unused) {return 0;};
    virtual int setTKV(Stream * s __unused, configType type __unused, effect_pal_payload_t *payload __unused) {return 0;};
    virtual uint32_t getMIID(const char *backendName __unused, uint32_t tagId __unused, uint32_t *miid __unused) { return -EINVAL; }
    virtual void setEventPayload(uint32_t event_id __unused, void *payload __unused, size_t payload_size __unused) {  };
    virtual struct mixer_ctl* getFEMixerCtl(const char *controlName __unused, int *device __unused, pal_stream_direction_t dir __unused) {return nullptr;}
    virtual int getTagsWithModuleInfo(Stream *s __unused, size_t *size __unused,
                                      uint8_t *payload __unused) {return -EINVAL;}
    virtual int checkAndSetExtEC(const std::shared_ptr<ResourceManager>& rm,
                                 Stream *s, bool is_enable);
    virtual void AdmRoutingChange(Stream *s __unused) {  };
    int NotifyChargerConcurrency(std::shared_ptr<ResourceManager>rm, bool state);
    int EnableChargerConcurrency(std::shared_ptr<ResourceManager>rm, Stream *s);
    int getEffectParameters(Stream *s, effect_pal_payload_t *effectPayload);
    int setEffectParameters(Stream *s, effect_pal_payload_t *effectPayload);
    int rwACDBParameters(void *payload, uint32_t sampleRate, bool isParamWrite);
    int rwACDBParamTunnel(void *payload, pal_device_id_t palDeviceId,
        pal_stream_type_t palStreamType, uint32_t sampleRate, uint32_t instanceId,
        bool isParamWrite, Stream *s);
private:
    uint32_t getModuleInfo(const char *control, uint32_t tagId, uint32_t *miid, struct mixer_ctl **ctl, int *device);
    int setEffectParametersTKV(Stream *s, effect_pal_payload_t *effectPayload);
    int setEffectParametersNonTKV(Stream *s, effect_pal_payload_t *effectPayload);
};

#endif //SESSION_AR_H
