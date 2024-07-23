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

#define INVALID_TAG -1
#define MUTE_TAG 0
#define UNMUTE_TAG 1
#define PAUSE_TAG 2
#define RESUME_TAG 3
#define MFC_SR_8K 4
#define MFC_SR_16K 5
#define MFC_SR_32K 6
#define MFC_SR_44K 7
#define MFC_SR_48K 8
#define MFC_SR_96K 9
#define MFC_SR_192K 10
#define MFC_SR_384K 11
#define ECNS_ON_TAG 12
#define ECNS_OFF_TAG 13
#define EC_ON_TAG 14
#define NS_ON_TAG 15
#define CHS_1 16
#define CHS_2 17
#define CHS_3 18
#define CHS_4 19
#define CHS_5 20
#define CHS_6 21
#define CHS_7 22
#define CHS_8 23
#define BW_16 24
#define BW_24 25
#define BW_32 26
#define TTY_MODE 27
#define VOICE_SLOW_TALK_OFF 28
#define VOICE_SLOW_TALK_ON 29
#define VOICE_VOLUME_BOOST 30
#define SPKR_PROT_ENABLE 31
#define INCALL_RECORD_UPLINK 32
#define INCALL_RECORD_DOWNLINK 33
#define INCALL_RECORD_UPLINK_DOWNLINK_MONO 34
#define INCALL_RECORD_UPLINK_DOWNLINK_STEREO 35
#define SPKR_VI_ENABLE 36
#define VOICE_HD_VOICE 37
#define LPI_LOGGING_ON 38
#define LPI_LOGGING_OFF 39
#define DEVICE_MUTE 40
#define DEVICE_UNMUTE 41
#define CHANNEL_INFO 42
#define CHARGE_CONCURRENCY_ON_TAG 43
#define CHARGE_CONCURRENCY_OFF_TAG 44
#define DEVICEPP_MUTE 45
#define DEVICEPP_UNMUTE 46
#define ORIENTATION_TAG 47
#define HANDSET_PROT_ENABLE 48
#define HAPTICS_VI_ENABLE 49
#define HAPTICS_PROT_ENABLE 50
#define CRS_CALL_VOLUME 51
#define GAIN_LVL 52
#define VOLUME_LVL 53

#define MSPP_SOFT_PAUSE_DELAY 150
#define DEFAULT_RAMP_PERIOD 0x28
#define EVENT_ID_SOFT_PAUSE_PAUSE_COMPLETE 0x0800103F /*look into moving */

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
    static std::mutex pauseMutex;
    static std::condition_variable pauseCV;
    pal_device_id_t ecRefDevId;
    bool isPauseRegistrationDone = false;
    int32_t setInitialVolume();
public:
    static Session* makeARSession(const std::shared_ptr<ResourceManager>& rm, const struct pal_stream_attributes *sAttr);
    static Session* makeACDBSession(const std::shared_ptr<ResourceManager>& rm, const struct pal_stream_attributes *sAttr);
    static void handleSoftPauseCallBack(uint64_t hdl, uint32_t event_id, void *data, uint32_t event_size);
    int HDRConfigKeyToDevOrientation(const char* hdr_custom_key);
    void setPmQosMixerCtl(pmQosVote vote);
    virtual int32_t getParameters(Stream *s, uint32_t param_id, void **payload) override;
    virtual int32_t getParamWithTag(Stream *s, int tagId, uint32_t param_id, void **payload) = 0;
    virtual int setParameters(Stream *s, uint32_t param_id, void *payload) override;
    virtual int setParamWithTag(Stream *s, int tagId __unused, uint32_t param_id, void *payload) = 0;
    virtual int setConfig(Stream * s, configType type, int tag) = 0;
    virtual int pause(Stream * s) override;
    virtual int suspend(Stream * s) override {return 0;};
    virtual int resume(Stream * s) override;
    virtual int mute(Stream * s, bool state) override;
    virtual int setVolume(Stream * s) override;
    virtual int drain(pal_drain_type_t type __unused) override {return 0;};
    virtual int flush() override {return 0;};
    virtual int read(Stream *s __unused, struct pal_buffer *buf __unused, int * size __unused) override {return 0;};
    virtual int write(Stream *s __unused, struct pal_buffer *buf __unused, int * size __unused) override {return 0;};
    virtual int getTimestamp(struct pal_session_time *stime __unused) override {return 0;};
    virtual int setTKV(Stream * s __unused, configType type __unused, effect_pal_payload_t *payload __unused) {return 0;};
    virtual int setConfig(Stream * s __unused, configType type __unused, uint32_t tag1 __unused,
            uint32_t tag2 __unused, uint32_t tag3 __unused) {return 0;};
    virtual int setConfig(Stream * s __unused, configType type __unused, int tag __unused, int dir __unused) {return 0;};
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
    int getEffectParameters(Stream *s, effect_pal_payload_t *effectPayload) override;
    int setEffectParameters(Stream *s, effect_pal_payload_t *effectPayload) override;
    int rwACDBParameters(void *payload, uint32_t sampleRate, bool isParamWrite) override;
    int rwACDBParamTunnel(void *payload, pal_device_id_t palDeviceId,
        pal_stream_type_t palStreamType, uint32_t sampleRate, uint32_t instanceId,
        bool isParamWrite, Stream *s) override;
private:
    uint32_t getModuleInfo(const char *control, uint32_t tagId, uint32_t *miid, struct mixer_ctl **ctl, int *device);
    int setEffectParametersTKV(Stream *s, effect_pal_payload_t *effectPayload);
    int setEffectParametersNonTKV(Stream *s, effect_pal_payload_t *effectPayload);
};

#endif //SESSION_AR_H
