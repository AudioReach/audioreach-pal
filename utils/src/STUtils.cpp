/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "PAL: STUtils"
#include <vector>
#include <algorithm>
#include <dlfcn.h>

#include "PalDefs.h"
#include "Device.h"

#include "STUtils.h"

#define NLPI_LPI_SWITCH_DELAY_SEC 5
#define NLPI_LPI_SWITCH_SLEEP_INTERVAL_SEC 1

static int TxconcurrencyEnableCount = 0;
static int concurrencyDisableCount = 0;
static int ACDConcurrencyDisableCount = 0;
static int ASRConcurrencyDisableCount = 0;
static int SNSPCMDataConcurrencyDisableCount = 0;

/*
 * Thread to handle deferred switch, only applicable
 * when low latency bargein is enabled.
 */
static defer_switch_state_t deferredSwitchState = NO_DEFER;
static std::thread vui_deferred_switch_thread_;
static std::condition_variable vui_switch_cv_;
static std::mutex vui_switch_mutex_;
static std::mutex st_utils_mutex_;
static bool vui_switch_thread_exit_ = false;
static int deferred_switch_cnt_ = -1;
#ifndef VUI_DMGR_AUDIO_UNSUPPORTED
void* vui_utils_dmgr_lib_handle = NULL;
vui_dmgr_init_t vui_utils_dmgr_init = NULL;
vui_dmgr_deinit_t vui_utils_dmgr_deinit = NULL;
#endif
std::shared_ptr<CaptureProfile> SoundTriggerCaptureProfile;
std::shared_ptr<CaptureProfile> TXMacroCaptureProfile;
std::unordered_map<int, pal_stream_handle_t *> mStCaptureInfo;
std::set<Stream*> mNLPIStreams;
std::list <Stream*> mStartDeferredStreams;
std::vector<std::pair<SoundTriggerOnResourceAvailableCallback, uint64_t>> onResourceAvailCbList;
bool use_lpi_ = true;
bool charging_state_;

// default properties which will be updated based on platform configuration
static struct pal_st_properties qst_properties = {
        "QUALCOMM Technologies, Inc",  // implementor
        "Sound Trigger HAL",  // description
        1,  // version
        { 0x68ab2d40, 0xe860, 0x11e3, 0x95ef,
         { 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b } },  // uuid
        8,  // max_sound_models
        10,  // max_key_phrases
        10,  // max_users
        PAL_RECOGNITION_MODE_VOICE_TRIGGER |
        PAL_RECOGNITION_MODE_GENERIC_TRIGGER,  // recognition_modes
        true,  // capture_transition
        0,  // max_capture_ms
        false,  // concurrent_capture
        false,  // trigger_in_event
        0  // power_consumption_mw
};

void STUtilsInit() {
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    if (rm && IsLowLatencyBargeinSupported()) {
        vui_deferred_switch_thread_ = std::thread(
            voiceUIDeferredSwitchLoop, rm);
    }
    PAL_INFO(LOG_TAG, "Initialize voiceui dmgr");
#ifndef VUI_DMGR_AUDIO_UNSUPPORTED
    voiceuiDmgrManagerInit();
#endif
}

void STUtilsDeinit() {
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    if (rm && IsLowLatencyBargeinSupported()) {
        vui_switch_mutex_.lock();
        vui_switch_thread_exit_ = true;
        vui_switch_cv_.notify_all();
        vui_switch_mutex_.unlock();
        if (vui_deferred_switch_thread_.joinable())
            vui_deferred_switch_thread_.join();
        PAL_DBG(LOG_TAG, "VoiceUI deferred switch thread joined");
    }
#ifndef VUI_DMGR_AUDIO_UNSUPPORTED
    voiceuiDmgrManagerDeInit();
#endif
}

#ifndef VUI_DMGR_AUDIO_UNSUPPORTED
void getMatchingStreams(std::list<Stream*> &active_streams, std::vector<Stream*> &streams, vui_dmgr_uuid_t &uuid)
{
    int ret = 0;
    pal_param_payload *param_payload = nullptr;
    size_t payload_size = sizeof(pal_param_payload) + sizeof(struct st_uuid);

    for (auto s : active_streams) {
        if (NULL != s) {
            param_payload = (pal_param_payload *)calloc(1, payload_size);
            param_payload->payload_size = sizeof(struct st_uuid);
            ret = s->getParameters(PAL_PARAM_ID_VENDOR_UUID, (void**)&param_payload);
            if(ret){
                PAL_ERR(LOG_TAG, "failed to get UUID");
            } else {
                if (!memcmp(param_payload->payload, &uuid, sizeof(uuid))) {
                    PAL_INFO(LOG_TAG, "vendor uuid matched");
                    streams.push_back(s);
                }
            }
            free(param_payload);
        }
    }
}
#endif
#ifndef VUI_DMGR_AUDIO_UNSUPPORTED
int32_t voiceuiDmgrRestartUseCases(vui_dmgr_param_restart_usecases_t *uc_info)
{
    int status = 0;
    std::vector<Stream*> streams;
    std::list<Stream*> activeStreams;
    pal_stream_type_t type;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    if(!rm){
        PAL_ERR(LOG_TAG, "unable to get resourcemanager instance");
        status = -EINVAL;
        goto exit;
    }

    for (int i = 0; i < uc_info->num_usecases; i++) {
        status = rm->getActiveStreamByType(activeStreams, (pal_stream_type_t)uc_info->usecases[i].stream_type);
        if(status || !activeStreams.size()){
            PAL_ERR(LOG_TAG, "failed to get active streams for stream type %d", uc_info->usecases[i].stream_type);
            break;
        }
        getMatchingStreams(activeStreams, streams, uc_info->usecases[i].vendor_uuid);
    }
    // Reuse SSR mechanism for stream teardown and bring up.
    PAL_INFO(LOG_TAG, "restart %d streams", streams.size());
    for (auto &s : streams) {
        s->getStreamType(&type);
        status = s->ssrDownHandler();
        if (status) {
            PAL_ERR(LOG_TAG, "stream teardown failed %d", type);
        }
        status = s->ssrUpHandler();
        if (status) {
            PAL_ERR(LOG_TAG, "strem bring up failed %d", type);
        }
    }
exit:
    return status;
}

int32_t voiceuiDmgrPalCallback(int32_t param_id, void *payload, size_t payload_size)
{
    int status = 0;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    PAL_DBG(LOG_TAG, "Enter param id: %d", param_id);
    if (!payload) {
        PAL_ERR(LOG_TAG, "Null payload");
        return -EINVAL;
    }
    if (!rm) {
        PAL_ERR(LOG_TAG, "null resource manager");
        return -EINVAL;
    }

    switch (param_id) {
        case VUI_DMGR_PARAM_ID_RESTART_USECASES:
        {
            vui_dmgr_param_restart_usecases_t *uc_info = (vui_dmgr_param_restart_usecases_t *)payload;
            if (payload_size != sizeof(vui_dmgr_param_restart_usecases_t)) {
                PAL_ERR(LOG_TAG, "Incorrect payload size %zu", payload_size);
                status = -EINVAL;
                break;
            }
            voiceuiDmgrRestartUseCases(uc_info);
        }
        break;
        default:
            PAL_ERR(LOG_TAG, "Unknown param id: %d", param_id);
            break;
    }

    PAL_DBG(LOG_TAG, "Exit status: %d", status);
    return status;
}

void voiceuiDmgrManagerInit()
{
    int status = 0;

    vui_utils_dmgr_lib_handle = dlopen(VUI_DMGR_LIB_PATH, RTLD_NOW);

    if (!vui_utils_dmgr_lib_handle) {
        PAL_ERR(LOG_TAG, "dlopen failed for voiceui dmgr %s", dlerror());
        return;
    }

    vui_utils_dmgr_init = (vui_dmgr_init_t)dlsym(vui_utils_dmgr_lib_handle, "vui_dmgr_init");
    if (!vui_utils_dmgr_init) {
        PAL_ERR(LOG_TAG, "dlsym for vui_utils_dmgr_init failed %s", dlerror());
        goto exit;
    }
    vui_utils_dmgr_deinit = (vui_dmgr_deinit_t)dlsym(vui_utils_dmgr_lib_handle, "vui_dmgr_deinit");
    if (!vui_utils_dmgr_deinit) {
        PAL_ERR(LOG_TAG, "dlsym for voiceui dmgr failed %s", dlerror());
        goto exit;
    }
    status = vui_utils_dmgr_init(voiceuiDmgrPalCallback);
    if (status) {
        PAL_DBG(LOG_TAG, "voiceui dmgr failed to initialize, status %d", status);
        goto exit;
    }
    PAL_INFO(LOG_TAG, "voiceui dgmr initialized");
    return;

exit:
    if (vui_utils_dmgr_lib_handle) {
        dlclose(vui_utils_dmgr_lib_handle);
        vui_utils_dmgr_lib_handle = NULL;
    }
    vui_utils_dmgr_init = NULL;
    vui_utils_dmgr_deinit = NULL;
}

void voiceuiDmgrManagerDeInit()
{
    if (vui_utils_dmgr_deinit)
        vui_utils_dmgr_deinit();

    if (vui_utils_dmgr_lib_handle) {
        dlclose(vui_utils_dmgr_lib_handle);
        vui_utils_dmgr_lib_handle = NULL;
    }
    vui_utils_dmgr_init = NULL;
    vui_utils_dmgr_deinit = NULL;
}
#endif
/* Moved from resource manager */

void GetVoiceUIProperties(struct pal_st_properties *qstp)
{
    std::shared_ptr<VoiceUIPlatformInfo> vui_info =
        VoiceUIPlatformInfo::GetInstance();

    if (!qstp) {
        return;
    }

    memcpy(qstp, &qst_properties, sizeof(struct pal_st_properties));

    if (vui_info) {
        qstp->version = vui_info->GetVersion();
        qstp->concurrent_capture = vui_info->GetConcurrentCaptureEnable();
    }
}

bool isNLPISwitchSupported() {

    std::shared_ptr<SoundTriggerPlatformInfo> st_info =
                             SoundTriggerPlatformInfo::GetInstance();
    return st_info != nullptr ? st_info->GetSupportNLPISwitch() : false;
}

bool CheckForForcedTransitToNonLPI() {
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    if (rm->IsTransitToNonLPIOnChargingSupported() && charging_state_)
      return true;

    return false;
}

// this should only be called when LPI supported by platform
void GetSTDisableConcurrencyCount_l(
    pal_stream_type_t type, int32_t *disable_count) {

    pal_stream_attributes st_attr;
    bool voice_conc_enable = IsVoiceCallConcurrencySupported();
    bool voip_conc_enable = IsVoipConcurrencySupported();
    bool audio_capture_conc_enable =
        IsAudioCaptureConcurrencySupported();
    bool low_latency_bargein_enable = IsLowLatencyBargeinSupported();

    if (type == PAL_STREAM_ACD) {
        *disable_count = ACDConcurrencyDisableCount;
    } else if (type == PAL_STREAM_VOICE_UI) {
        *disable_count = concurrencyDisableCount;
    } else if (type == PAL_STREAM_ASR) {
        *disable_count = ASRConcurrencyDisableCount;
    } else if (type == PAL_STREAM_SENSOR_PCM_DATA) {
        *disable_count = SNSPCMDataConcurrencyDisableCount;
    } else {
        PAL_ERR(LOG_TAG, "Error:%d Invalid stream type %d", -EINVAL, type);
        return;
    }

    PAL_INFO(LOG_TAG, "stream: %d conc disable count %d", type, *disable_count);
}

bool IsLowLatencyBargeinSupported() {
    std::shared_ptr<SoundTriggerPlatformInfo> st_info =
                SoundTriggerPlatformInfo::GetInstance();
    return st_info != nullptr ? st_info->GetLowLatencyBargeinEnable() : false;
}

bool IsAudioCaptureConcurrencySupported() {
    std::shared_ptr<SoundTriggerPlatformInfo> st_info =
                SoundTriggerPlatformInfo::GetInstance();
    return st_info != nullptr ? st_info->GetConcurrentCaptureEnable() : false;
}

bool IsVoiceCallConcurrencySupported() {
    std::shared_ptr<SoundTriggerPlatformInfo> st_info =
                SoundTriggerPlatformInfo::GetInstance();
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    /* if CRS call allow concurrency*/
    if (rm->IsCRSCallEnabled()){
        PAL_INFO(LOG_TAG, "In CRS call, allow voice concurrency");
        return true;
    }

    return st_info != nullptr ? st_info->GetConcurrentVoiceCallEnable() : false;
}

bool IsVoipConcurrencySupported() {
    std::shared_ptr<SoundTriggerPlatformInfo> st_info =
                SoundTriggerPlatformInfo::GetInstance();
    return st_info != nullptr ? st_info->GetConcurrentVoipCallEnable() : false;
}

std::shared_ptr<CaptureProfile> GetASRCaptureProfileByPriority(
    Stream *s, std::shared_ptr<CaptureProfile> cap_prof_priority,
    std::string backend) {
    std::shared_ptr<CaptureProfile> cap_prof = nullptr;
    std::list<Stream*> activeStreams;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    rm->getActiveStreamByType_l(activeStreams, PAL_STREAM_ASR);
    for (auto& str: activeStreams) {
        if (str == s) {
            continue;
        }

        if (!str->isActive())
            continue;

        cap_prof = str->GetCurrentCaptureProfile();
        if (!cap_prof) {
            PAL_ERR(LOG_TAG, "Failed to get capture profile");
            continue;
        } else if (cap_prof->GetBackend().compare(backend) != 0) {
            continue;
        } else if (cap_prof->ComparePriority(cap_prof_priority) >=
                   CAPTURE_PROFILE_PRIORITY_HIGH) {
            cap_prof_priority = cap_prof;
        }
    }

    return cap_prof_priority;
}

std::shared_ptr<CaptureProfile> GetACDCaptureProfileByPriority(
    Stream *s, std::shared_ptr<CaptureProfile> cap_prof_priority,
    std::string backend) {
    std::shared_ptr<CaptureProfile> cap_prof = nullptr;
    std::list<Stream*> activeStreams;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    rm->getActiveStreamByType_l(activeStreams, PAL_STREAM_ACD);
    for (auto& str: activeStreams) {
       // NOTE: input param s can be nullptr here
        if (str == s) {
            continue;
        }

        if (!str->isActive())
            continue;

        cap_prof = str->GetCurrentCaptureProfile();
        if (!cap_prof) {
            PAL_ERR(LOG_TAG, "Failed to get capture profile");
            continue;
        } else if (cap_prof->GetBackend().compare(backend) != 0) {
            continue;
        } else if (cap_prof->ComparePriority(cap_prof_priority) >=
                   CAPTURE_PROFILE_PRIORITY_HIGH) {
            cap_prof_priority = cap_prof;
        }
    }

    return cap_prof_priority;
}

std::shared_ptr<CaptureProfile> GetSVACaptureProfileByPriority(
    Stream *s, std::shared_ptr<CaptureProfile> cap_prof_priority,
    std::string backend) {
    std::shared_ptr<CaptureProfile> cap_prof = nullptr;
    std::list<Stream*> activeStreams;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    rm->getActiveStreamByType_l(activeStreams, PAL_STREAM_VOICE_UI);
    for (auto& str: activeStreams) {
        // NOTE: input param s can be nullptr here
        if (str == s) {
            continue;
        }

        /*
         * Ignore capture profile for streams in below states:
         * 1. sound model loaded but not started by sthal
         * 2. stop recognition called by sthal
         */
        if (!str->isActive())
            continue;

        cap_prof = str->GetCurrentCaptureProfile();
        if (!cap_prof) {
            PAL_ERR(LOG_TAG, "Failed to get capture profile");
            continue;
        } else if ((cap_prof->GetDevId() == PAL_DEVICE_IN_HANDSET_MIC) ||
            (cap_prof->GetDevId() == PAL_DEVICE_IN_SPEAKER_MIC) ||
            (cap_prof->GetDevId() == PAL_DEVICE_IN_WIRED_HEADSET)) {
            continue;
        } else if (cap_prof->GetBackend().compare(backend) != 0) {
            continue;
        } else if (cap_prof->ComparePriority(cap_prof_priority) >=
                   CAPTURE_PROFILE_PRIORITY_HIGH) {
            cap_prof_priority = cap_prof;
        }
    }

    return cap_prof_priority;
}

std::shared_ptr<CaptureProfile> GetSPDCaptureProfileByPriority(
    Stream *s, std::shared_ptr<CaptureProfile> cap_prof_priority,
    std::string backend) {
    std::shared_ptr<CaptureProfile> cap_prof = nullptr;
    std::list<Stream*> activeStreams;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    rm->getActiveStreamByType_l(activeStreams, PAL_STREAM_SENSOR_PCM_DATA);
    for (auto& str: activeStreams) {
        // NOTE: input param s can be nullptr here
        if (str == s) {
            continue;
        }

        if (NULL != str) {
            cap_prof = str->GetCurrentCaptureProfile();
            if (!cap_prof) {
                PAL_ERR(LOG_TAG, "Failed to get capture profile");
                continue;
            } else if (cap_prof->GetBackend().compare(backend) != 0) {
                continue;
            } else if (cap_prof->ComparePriority(cap_prof_priority) >=
                       CAPTURE_PROFILE_PRIORITY_HIGH) {
                cap_prof_priority = cap_prof;
            }
        }
    }

    return cap_prof_priority;
}

std::shared_ptr<CaptureProfile> GetCaptureProfileByPriority(
    Stream *s, std::string backend)
{
    struct pal_stream_attributes sAttr;
    std::shared_ptr<CaptureProfile> cap_prof_priority = nullptr;
    int32_t status = 0;

    if (!s)
        goto get_priority;
    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG, "stream get attributes failed");
        goto exit;
    }

get_priority:
    cap_prof_priority = GetSVACaptureProfileByPriority(s, cap_prof_priority, backend);
    cap_prof_priority = GetACDCaptureProfileByPriority(s, cap_prof_priority, backend);
    cap_prof_priority = GetSPDCaptureProfileByPriority(s, cap_prof_priority, backend);
    cap_prof_priority = GetASRCaptureProfileByPriority(s, cap_prof_priority, backend);
 exit:
   return cap_prof_priority;
}

bool UpdateSoundTriggerCaptureProfile(Stream *s, bool is_active) {

    bool backend_update = false;
    std::shared_ptr<CaptureProfile> cap_prof = nullptr;
    std::shared_ptr<CaptureProfile> st_cap_prof_priority = nullptr;
    std::shared_ptr<CaptureProfile> tx_cap_prof_priority = nullptr;
    std::shared_ptr<CaptureProfile> common_cap_prof = nullptr;
    struct pal_stream_attributes sAttr;
    int32_t status = 0;

    if (!s) {
        PAL_ERR(LOG_TAG, "Invalid stream");
        return false;
    }

    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG, "stream get attributes failed");
        return false;
    }

    if (sAttr.type != PAL_STREAM_VOICE_UI &&
        sAttr.type != PAL_STREAM_ACD &&
        sAttr.type != PAL_STREAM_SENSOR_PCM_DATA &&
        sAttr.type != PAL_STREAM_ASR) {
        PAL_ERR(LOG_TAG, "Error:%d Invalid stream type", -EINVAL);
        return false;
    }
    // backend config update
    if (is_active) {
        cap_prof = s->GetCurrentCaptureProfile();
        if (!cap_prof) {
            PAL_ERR(LOG_TAG, "Failed to get capture profile");
            return false;
        }
        st_utils_mutex_.lock();
        common_cap_prof = (cap_prof->GetBackend().compare("tx_macro") != 0) ?
                          SoundTriggerCaptureProfile : TXMacroCaptureProfile;
        if (!common_cap_prof) {
            common_cap_prof = cap_prof;
        } else if (cap_prof->ComparePriority(common_cap_prof) >=
                   CAPTURE_PROFILE_PRIORITY_HIGH){
            common_cap_prof = cap_prof;
            backend_update = true;
        }
        ((cap_prof->GetBackend().compare("tx_macro") != 0) ?
                          SoundTriggerCaptureProfile : TXMacroCaptureProfile) = common_cap_prof;
        st_utils_mutex_.unlock();
    } else {
        /* Updating SoundTriggerCaptureProfile for streams use VA Macro capture profiles */
        st_cap_prof_priority = GetCaptureProfileByPriority(s, "va_macro");

        /* Updating TXMacroCaptureProfile for streams use TX Macro capture profiles */
        tx_cap_prof_priority = GetCaptureProfileByPriority(s, "tx_macro");

        st_utils_mutex_.lock();
        if (!st_cap_prof_priority) {
            PAL_DBG(LOG_TAG, "No session active, reset VA Macro common capture profiles");
            SoundTriggerCaptureProfile = nullptr;
        } else if (st_cap_prof_priority->ComparePriority(SoundTriggerCaptureProfile) >=
                CAPTURE_PROFILE_PRIORITY_HIGH) {
                SoundTriggerCaptureProfile = st_cap_prof_priority;
                backend_update = true;
        }
        if (!tx_cap_prof_priority) {
            PAL_DBG(LOG_TAG, "No session active, reset TX Macro common capture profiles");
            TXMacroCaptureProfile = nullptr;
        } else if (tx_cap_prof_priority->ComparePriority(TXMacroCaptureProfile) >=
                CAPTURE_PROFILE_PRIORITY_HIGH) {
                TXMacroCaptureProfile = tx_cap_prof_priority;
                backend_update = true;
        }
        st_utils_mutex_.unlock();
    }

    return backend_update;
}

int HandleDetectionStreamAction(pal_stream_type_t type, int32_t action, void *data)
{
    int status = 0;
    pal_stream_attributes st_attr;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    if ((type == PAL_STREAM_VOICE_UI &&
         !rm->getActiveStreamMap().count(PAL_STREAM_VOICE_UI)) ||
        (type == PAL_STREAM_ACD &&
         !rm->getActiveStreamMap().count(PAL_STREAM_ACD)) ||
        (type == PAL_STREAM_ASR &&
         !rm->getActiveStreamMap().count(PAL_STREAM_ASR)) ||
        (type == PAL_STREAM_SENSOR_PCM_DATA &&
         !rm->getActiveStreamMap().count(PAL_STREAM_SENSOR_PCM_DATA))) {
        PAL_VERBOSE(LOG_TAG, "No active stream for type %d, skip action", type);
        return 0;
    }

    PAL_DBG(LOG_TAG, "Enter");
    for (auto& str: rm->getActiveStreamList()) {
        if (!rm->isStreamActive(str))
            continue;

        str->getStreamAttributes(&st_attr);
        if (st_attr.type != type)
            continue;

        switch (action) {
            case ST_PAUSE:
                if (str != (Stream *)data) {
                    status = str->Pause();
                    if (status)
                        PAL_ERR(LOG_TAG, "Failed to pause stream");
                }
                break;
            case ST_RESUME:
                if (str != (Stream *)data) {
                    status = str->Resume();
                    if (status)
                        PAL_ERR(LOG_TAG, "Failed to do resume stream");
                }
                break;
            case ST_HANDLE_CONCURRENT_STREAM: {
                bool enable = *(bool *)data;
                status = str->HandleConcurrentStream(enable);
                if (status)
                    PAL_ERR(LOG_TAG, "Failed to stop/unload stream");
                }
                break;
            case ST_HANDLE_DISCONNECT_DEVICE: {
                pal_device_id_t device_to_disconnect = *(pal_device_id_t*)data;
                status = str->DisconnectDevice(device_to_disconnect);
                if (status)
                    PAL_ERR(LOG_TAG, "Failed to disconnect device %d",
                            device_to_disconnect);
                }
                break;
            case ST_HANDLE_CONNECT_DEVICE: {
                pal_device_id_t device_to_connect = *(pal_device_id_t*)data;
                status = str->ConnectDevice(device_to_connect);
                if (status)
                    PAL_ERR(LOG_TAG, "Failed to connect device %d",
                            device_to_connect);
                }
                break;
            case ST_INTERNAL_PAUSE:
                if (str != (Stream *)data) {
                    status = str->Pause(true);
                    if (status)
                        PAL_ERR(LOG_TAG, "Internal pause stream failed");
                }
                break;
            case ST_INTERNAL_RESUME:
                if (str != (Stream *)data) {
                    status = str->Resume(true);
                    if (status)
                        PAL_ERR(LOG_TAG, "Internal resume stream failed");
                }
                break;
            default:
                break;
        }
    }
    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int StopOtherDetectionStreams(void *st) {
    HandleDetectionStreamAction(PAL_STREAM_VOICE_UI, ST_INTERNAL_PAUSE, st);
    HandleDetectionStreamAction(PAL_STREAM_ACD, ST_PAUSE, st);
    HandleDetectionStreamAction(PAL_STREAM_ASR, ST_PAUSE, st);
    HandleDetectionStreamAction(PAL_STREAM_SENSOR_PCM_DATA, ST_PAUSE, st);
    return 0;
}

int StartOtherDetectionStreams(void *st) {
    HandleDetectionStreamAction(PAL_STREAM_VOICE_UI, ST_INTERNAL_RESUME, st);
    HandleDetectionStreamAction(PAL_STREAM_ACD, ST_RESUME, st);
    HandleDetectionStreamAction(PAL_STREAM_ASR, ST_RESUME, st);
    HandleDetectionStreamAction(PAL_STREAM_SENSOR_PCM_DATA, ST_RESUME, st);
    return 0;
}

void HandleStreamPauseResume(pal_stream_type_t st_type, bool active)
{
    int32_t *local_dis_count;

    if (st_type == PAL_STREAM_ACD)
        local_dis_count = &ACDConcurrencyDisableCount;
    else if (st_type == PAL_STREAM_ASR)
        local_dis_count = &ASRConcurrencyDisableCount;
    else if (st_type == PAL_STREAM_VOICE_UI)
        local_dis_count = &concurrencyDisableCount;
    else if (st_type == PAL_STREAM_SENSOR_PCM_DATA)
        local_dis_count = &SNSPCMDataConcurrencyDisableCount;
    else
        return;

    if (active) {
        if (++(*local_dis_count) == 1) {
            // pause all sva/acd streams
            HandleDetectionStreamAction(st_type, ST_PAUSE, NULL);
        }
    } else {
        if ((*local_dis_count) < 0) {
            (*local_dis_count) = 0;
        } else if ((*local_dis_count) > 0 && --(*local_dis_count) == 0) {
            // resume all sva/acd streams
            HandleDetectionStreamAction(st_type, ST_RESUME, NULL);
        }
    }
}

void RegisterSTCaptureHandle(pal_param_st_capture_info_t stCaptureInfo,
                                              bool start) {
    PAL_DBG(LOG_TAG, "start %d, capture handle %d, pal handle %pK",
                          start, stCaptureInfo.capture_handle, stCaptureInfo.pal_handle);
    if (start) {
        mStCaptureInfo[stCaptureInfo.capture_handle] = stCaptureInfo.pal_handle;
    } else {
        mStCaptureInfo.erase(stCaptureInfo.capture_handle);
    }
}

bool checkAndUpdateDeferSwitchState(bool stream_active)
{
    std::unique_lock<std::mutex> lck(vui_switch_mutex_);
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    std::list<Stream*> activeSTStreams;

    /*
     * When switching from NLPI to LPI:
     * 1. If low latency bargein is enabled, nlpi to lpi switch
     *    will be delayed by 5s until sleep is done in thread
     *    voiceUIDeferredSwitchLoop.
     * 2. Else if there's any VoiceUI stream in buffering, delay
     *    switch until buffering is done.
     *
     * When switching from LPI to NLPI:
     * 1. If there's any VoiceUI stream in buffering, delay switch
     *    until buffering is done.
     * 2. Additionally if low latency bargein is enabled and there's
     *    pending NLPI to LPI switch, then skip the pending switch
     *    and exit the sleep in voiceUIDeferredSwitchLoop.
     */
    if (!stream_active) {
        rm->getActiveStreamByType_l(activeSTStreams, PAL_STREAM_VOICE_UI);
        if (IsLowLatencyBargeinSupported() && activeSTStreams.size()) {
            deferredSwitchState =
                (deferredSwitchState == DEFER_LPI_NLPI_SWITCH) ? NO_DEFER :
                 DEFER_NLPI_LPI_SWITCH;
            PAL_INFO(LOG_TAG,
                "Low latency bargein enabled, defer NLPI->LPI switch, deferred state:%d",
                deferredSwitchState);
            deferred_switch_cnt_ = NLPI_LPI_SWITCH_DELAY_SEC;
            vui_switch_cv_.notify_all();
            return true;
        } else if (rm->isAnyStreamBuffering()) {
            deferredSwitchState =
                (deferredSwitchState == DEFER_LPI_NLPI_SWITCH) ? NO_DEFER :
                 DEFER_NLPI_LPI_SWITCH;
            PAL_INFO(LOG_TAG,
                "VUI stream in buffering, defer NLPI->LPI switch, deferred state:%d",
                deferredSwitchState);
            return true;
        }
    } else {
        if (rm->isAnyStreamBuffering()) {
            deferredSwitchState =
                (deferredSwitchState == DEFER_NLPI_LPI_SWITCH) ? NO_DEFER :
                 DEFER_LPI_NLPI_SWITCH;
            PAL_INFO(LOG_TAG,
                "VUI stream in buffering, defer LPI->NLPI switch, deferred state:%d,"
                " LPI will be used until buffering done, hence EC won't be applied",
                deferredSwitchState);
            return true;
        }
        if (IsLowLatencyBargeinSupported() &&
            deferredSwitchState == DEFER_NLPI_LPI_SWITCH) {
            deferredSwitchState = NO_DEFER;
            deferred_switch_cnt_ = -1;
            PAL_INFO(LOG_TAG,
                "Cancel pending NLPI to LPI switch as new concurrency coming");
            return true;
        }
    }

    return false;
}

void voiceUIDeferredSwitchLoop(std::shared_ptr<ResourceManager> rm)
{
    bool is_wake_lock_acquired = false;
    PAL_INFO(LOG_TAG, "Enter");
    std::unique_lock<std::mutex> lck(vui_switch_mutex_);

    while (!vui_switch_thread_exit_) {
        if (deferred_switch_cnt_ < 0) {
            if (is_wake_lock_acquired) {
                rm->releaseWakeLock();
                is_wake_lock_acquired = false;
            }
            vui_switch_cv_.wait(lck);
            rm->acquireWakeLock();
            is_wake_lock_acquired = true;
        }

        if (vui_switch_thread_exit_) {
            if (is_wake_lock_acquired)
                rm->releaseWakeLock();
            break;
        }

        if (deferred_switch_cnt_ > 0) {
            deferred_switch_cnt_--;
            lck.unlock();
            sleep(NLPI_LPI_SWITCH_SLEEP_INTERVAL_SEC);
            lck.lock();
        }

        if (deferred_switch_cnt_ == 0) {
            lck.unlock();
            stHandleDeferredSwitch();
            lck.lock();
            if (deferred_switch_cnt_ == 0)
                deferred_switch_cnt_ = -1;
        }
    }
}

/* This function should be called with mActiveStreamMutex lock acquired */
void handleConcurrentStreamSwitch(std::vector<pal_stream_type_t>& st_streams)
{
    std::shared_ptr<CaptureProfile> st_cap_prof_priority = nullptr;
    std::shared_ptr<CaptureProfile> tx_cap_prof_priority = nullptr;
    PAL_DBG(LOG_TAG, "Enter");

    // update common capture profile after use_lpi_ updated for all streams
    if (st_streams.size()) {
        /* Updating SoundTriggerCaptureProfile for streams use VA Macro capture profiles */
        SoundTriggerCaptureProfile = nullptr;
        st_cap_prof_priority = GetCaptureProfileByPriority(nullptr, "va_macro");

        /* Updating TXMacroCaptureProfile for streams use TX Macro capture profiles */
        TXMacroCaptureProfile = nullptr;
        tx_cap_prof_priority = GetCaptureProfileByPriority(nullptr, "tx_macro");

        st_utils_mutex_.lock();
        if (!st_cap_prof_priority) {
            PAL_DBG(LOG_TAG, "No session active, reset VA Macro common capture profile");
            SoundTriggerCaptureProfile = nullptr;
        } else if (st_cap_prof_priority->ComparePriority(SoundTriggerCaptureProfile) >=
                CAPTURE_PROFILE_PRIORITY_HIGH) {
            SoundTriggerCaptureProfile = st_cap_prof_priority;
        }
        if (!tx_cap_prof_priority) {
            PAL_DBG(LOG_TAG, "No session active, reset TX Macro common capture profile");
            TXMacroCaptureProfile = nullptr;
        } else if (tx_cap_prof_priority->ComparePriority(TXMacroCaptureProfile) >=
                CAPTURE_PROFILE_PRIORITY_HIGH) {
            TXMacroCaptureProfile = tx_cap_prof_priority;
        }
        st_utils_mutex_.unlock();
    }

    for (pal_stream_type_t st_stream_type_to_stop : st_streams) {
        // stop/unload SVA/ACD/Sensor PCM Data streams
        bool action = false;
        PAL_DBG(LOG_TAG, "stop/unload stream type %d", st_stream_type_to_stop);
        HandleDetectionStreamAction(st_stream_type_to_stop,
            ST_HANDLE_CONCURRENT_STREAM, (void *)&action);
    }

    for (pal_stream_type_t st_stream_type_to_start : st_streams) {
        // load/start SVA/ACD/Sensor PCM Data streams
        bool action = true;
        PAL_DBG(LOG_TAG, "load/start stream type %d", st_stream_type_to_start);
        HandleDetectionStreamAction(st_stream_type_to_start,
            ST_HANDLE_CONCURRENT_STREAM, (void *)&action);
    }
}

void GetConcurrencyInfo(Stream* s, bool *rx_conc, bool *tx_conc, bool *conc_en)
{
    int32_t status  = 0;
    bool voice_conc_enable = IsVoiceCallConcurrencySupported();
    bool voip_conc_enable = IsVoipConcurrencySupported();
    bool low_latency_bargein_enable = IsLowLatencyBargeinSupported();
    bool audio_capture_conc_enable = IsAudioCaptureConcurrencySupported();
    struct pal_stream_attributes sAttr = {};

    status = s->getStreamAttributes(&sAttr);
    if (status) {
        PAL_ERR(LOG_TAG, "stream get attributes failed, status: %d", status);
        return;
    }

    if (sAttr.direction == PAL_AUDIO_OUTPUT) {
        if (sAttr.type == PAL_STREAM_LOW_LATENCY && !low_latency_bargein_enable) {
            PAL_VERBOSE(LOG_TAG, "Ignore low latency playback stream");
        } else if (sAttr.type == PAL_STREAM_SENSOR_PCM_RENDERER) {
            PAL_VERBOSE(LOG_TAG, "Ignore sensor renderer stream");
        } else if (sAttr.type == PAL_STREAM_HAPTICS) {
            PAL_VERBOSE(LOG_TAG, "Ignore haptics stream");
        } else {
            *rx_conc = true;
        }
    }

    /*
     * Generally voip/voice rx stream comes with related tx streams,
     * so there's no need to switch to NLPI for voip/voice rx stream
     * if corresponding voip/voice tx stream concurrency is not supported.
     * Also note that capture concurrency has highest proirity that
     * when capture concurrency is disabled then concurrency for voip
     * and voice call should also be disabled even voice_conc_enable
     * or voip_conc_enable is set to true.
     */
    if (sAttr.type == PAL_STREAM_VOICE_CALL) {
        *tx_conc = true;
        *rx_conc = true;
        if (!audio_capture_conc_enable || !voice_conc_enable) {
            PAL_DBG(LOG_TAG, "pause on voice concurrency");
            *conc_en = false;
        }
    } else if (sAttr.type == PAL_STREAM_VOIP_TX) {
        *tx_conc = true;
        if (!audio_capture_conc_enable || !voip_conc_enable) {
            PAL_DBG(LOG_TAG, "pause on voip concurrency");
            *conc_en = false;
        }
    } else if (sAttr.type == PAL_STREAM_ACD ||
               sAttr.type == PAL_STREAM_SENSOR_PCM_DATA ||
               sAttr.type == PAL_STREAM_VOICE_UI ||
               sAttr.type == PAL_STREAM_ASR) {
        if (!s->ConfigSupportLPI()) {
            *tx_conc = true;
            PAL_DBG(LOG_TAG, "ST stream type: %d is in NLPI", sAttr.type);
        }
    } else if (sAttr.direction == PAL_AUDIO_INPUT &&
               sAttr.type != PAL_STREAM_CONTEXT_PROXY) {
        *tx_conc = true;
        if (!audio_capture_conc_enable && sAttr.type != PAL_STREAM_PROXY) {
            PAL_DBG(LOG_TAG, "pause on audio capture concurrency");
            *conc_en = false;
        }
    } else if (sAttr.type == PAL_STREAM_LOOPBACK){
        *tx_conc = true;
        *rx_conc = true;
        if (!audio_capture_conc_enable) {
            PAL_DBG(LOG_TAG, "pause on LOOPBACK concurrency");
            *conc_en = false;
        }
    }

    PAL_INFO(LOG_TAG, "stream type %d Tx conc %d, Rx conc %d, concurrency%s allowed",
        sAttr.type, *tx_conc, *rx_conc, *conc_en? "" : " not");
}

// called with mActiveStreamMutex locked
void onChargingStateChange()
{
    std::vector<pal_stream_type_t> st_streams;
    bool need_switch = false;
    bool use_lpi_temp = false;
    std::list<Stream*> activeVUIStreams, activeACDStreams, activeASRStreams, activeSPDStreams;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();


    rm->getActiveStreamByType_l(activeVUIStreams, PAL_STREAM_VOICE_UI);
    rm->getActiveStreamByType_l(activeACDStreams, PAL_STREAM_ACD);
    rm->getActiveStreamByType_l(activeASRStreams, PAL_STREAM_ASR);
    rm->getActiveStreamByType_l(activeSPDStreams, PAL_STREAM_SENSOR_PCM_DATA);
    // no need to handle car mode if no Voice Stream exists
    if (activeVUIStreams.size() == 0)
        return;

    if (charging_state_ && use_lpi_) {
        use_lpi_temp = false;
        need_switch = true;
    } else if (!charging_state_ && !use_lpi_) {
        if (!mNLPIStreams.size()) {
            use_lpi_temp = true;
            need_switch = true;
        }
    }

    if (need_switch) {
        if (activeVUIStreams.size())
            st_streams.push_back(PAL_STREAM_VOICE_UI);
        if (activeACDStreams.size())
            st_streams.push_back(PAL_STREAM_ACD);
        if (activeASRStreams.size())
            st_streams.push_back(PAL_STREAM_ASR);
        if (activeSPDStreams.size())
            st_streams.push_back(PAL_STREAM_SENSOR_PCM_DATA);

        if (!checkAndUpdateDeferSwitchState(!use_lpi_temp)) {
            use_lpi_ = use_lpi_temp;
            handleConcurrentStreamSwitch(st_streams);
        }
    }
}

// called with mActiveStreamMutex locked
void onVUIStreamRegistered()
{
    std::vector<pal_stream_type_t> st_streams;
    std::list<Stream*> activeACDStreams, activeSPDStreams;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    rm->lockActiveStream();

    rm->getActiveStreamByType_l(activeACDStreams, PAL_STREAM_ACD);
    rm->getActiveStreamByType_l(activeSPDStreams, PAL_STREAM_SENSOR_PCM_DATA);

    if (!charging_state_ || !rm->IsTransitToNonLPIOnChargingSupported()) {
        rm->unlockActiveStream();
        return;
    }

    if (activeACDStreams.size())
        st_streams.push_back(PAL_STREAM_ACD);
    if (activeSPDStreams.size())
        st_streams.push_back(PAL_STREAM_SENSOR_PCM_DATA);

    if (use_lpi_) {
        use_lpi_ = false;
        handleConcurrentStreamSwitch(st_streams);
    }

    rm->unlockActiveStream();
}

// called with mActiveStreamMutex locked
void onVUIStreamDeregistered()
{
    std::vector<pal_stream_type_t> st_streams;
    std::list<Stream*> activeACDStreams, activeSPDStreams;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    rm->lockActiveStream();

    rm->getActiveStreamByType_l(activeACDStreams, PAL_STREAM_ACD);
    rm->getActiveStreamByType_l(activeSPDStreams, PAL_STREAM_SENSOR_PCM_DATA);

    if (!charging_state_ || !rm->IsTransitToNonLPIOnChargingSupported()) {
        rm->unlockActiveStream();
        return;
    }

    if (activeACDStreams.size())
        st_streams.push_back(PAL_STREAM_ACD);
    if (activeSPDStreams.size())
        st_streams.push_back(PAL_STREAM_SENSOR_PCM_DATA);

    if (!use_lpi_ && !mNLPIStreams.size()) {
        use_lpi_ = true;
        handleConcurrentStreamSwitch(st_streams);
    }

    rm->unlockActiveStream();
}

void GetSTDisableConcurrencyCount(
    pal_stream_type_t type, int32_t *disable_count) {
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    rm->lockActiveStream();
    GetSTDisableConcurrencyCount_l(type, disable_count);
    rm->unlockActiveStream();
}

//move locked functions

void SwitchSoundTriggerDevices(bool connect_state,
                               pal_device_id_t st_device) {
    pal_device_id_t device_to_disconnect;
    pal_device_id_t device_to_connect;
    std::vector<pal_stream_type_t> st_streams;
    std::shared_ptr<CaptureProfile> st_cap_prof_priority = nullptr;
    std::shared_ptr<CaptureProfile> tx_cap_prof_priority = nullptr;
    std::shared_ptr<SoundTriggerPlatformInfo> st_info = nullptr;
    std::list<Stream*> activeStream;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    PAL_DBG(LOG_TAG, "Enter");

    /*
     * Voice UI, ACD, ASR and Sensor PCM Data(SPD)
     * share the sound trigger platform info.
     */
    st_info = SoundTriggerPlatformInfo::GetInstance();
    if (st_info && (false == st_info->GetSupportDevSwitch())) {
        PAL_INFO(LOG_TAG, "Device switch not supported");
        goto exit;
    }

    /* Updating SoundTriggerCaptureProfile for streams use VA Macro capture profiles */
    SoundTriggerCaptureProfile = nullptr;
    st_cap_prof_priority = GetCaptureProfileByPriority(nullptr, "va_macro");

    /* Updating TXMacroCaptureProfile for streams use TX Macro capture profiles */
    TXMacroCaptureProfile = nullptr;
    tx_cap_prof_priority = GetCaptureProfileByPriority(nullptr, "tx_macro");

    st_utils_mutex_.lock();
    if (!st_cap_prof_priority) {
        PAL_DBG(LOG_TAG, "No ST session active, reset VA Macro capture profile");
        SoundTriggerCaptureProfile = nullptr;
    } else if (st_cap_prof_priority->ComparePriority(SoundTriggerCaptureProfile) >=
            CAPTURE_PROFILE_PRIORITY_HIGH) {
        SoundTriggerCaptureProfile = st_cap_prof_priority;
    }
    if (!tx_cap_prof_priority) {
        PAL_DBG(LOG_TAG, "No ST session active, reset TX Macro capture profile");
        TXMacroCaptureProfile = nullptr;
    } else if (tx_cap_prof_priority->ComparePriority(TXMacroCaptureProfile) >=
            CAPTURE_PROFILE_PRIORITY_HIGH) {
        TXMacroCaptureProfile = tx_cap_prof_priority;
    }
    st_utils_mutex_.unlock();

    if (true == connect_state) {
        device_to_connect = st_device;
        device_to_disconnect = PAL_DEVICE_IN_HANDSET_VA_MIC;
    } else {
        device_to_connect = PAL_DEVICE_IN_HANDSET_VA_MIC;
        device_to_disconnect = st_device;
    }

    rm->lockActiveStream();
    rm->getActiveStreamByType_l(activeStream, PAL_STREAM_VOICE_UI);
    if (activeStream.size())
            st_streams.push_back(PAL_STREAM_VOICE_UI);
    activeStream.clear();
    rm->getActiveStreamByType_l(activeStream, PAL_STREAM_ACD);
    if (activeStream.size())
            st_streams.push_back(PAL_STREAM_ACD);
    activeStream.clear();
    rm->getActiveStreamByType_l(activeStream, PAL_STREAM_ASR);
    if (activeStream.size())
            st_streams.push_back(PAL_STREAM_ASR);
    activeStream.clear();
     rm->getActiveStreamByType_l(activeStream, PAL_STREAM_SENSOR_PCM_DATA);
    if (activeStream.size())
            st_streams.push_back(PAL_STREAM_SENSOR_PCM_DATA);

    for (pal_stream_type_t st_stream_type : st_streams) {
        /* Disconnect device for all active sound trigger streams */
        HandleDetectionStreamAction(st_stream_type, ST_HANDLE_DISCONNECT_DEVICE,
                                    (void *)&device_to_disconnect);
    }

    for (pal_stream_type_t st_stream_type : st_streams) {
        /* Connect device for all active sound trigger streams */
        HandleDetectionStreamAction(st_stream_type, ST_HANDLE_CONNECT_DEVICE,
                                    (void *)&device_to_connect);
    }
    rm->unlockActiveStream();

exit:
    PAL_DBG(LOG_TAG, "Exit");
}

void forceSwitchSoundTriggerStreams(bool active) {
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    if (!PAL_CARD_STATUS_DOWN(rm->getSoundCardState())) {
        rm->lockActiveStream();
        std::vector<pal_stream_type_t> st_streams;

        if (rm->getActiveStreamMap().count(PAL_STREAM_VOICE_UI))
            st_streams.push_back(PAL_STREAM_VOICE_UI);
        if (rm->getActiveStreamMap().count(PAL_STREAM_ACD))
            st_streams.push_back(PAL_STREAM_ACD);
        if (rm->getActiveStreamMap().count(PAL_STREAM_SENSOR_PCM_DATA))
            st_streams.push_back(PAL_STREAM_SENSOR_PCM_DATA);

        if (checkAndUpdateDeferSwitchState(active)) {
            PAL_DBG(LOG_TAG, "Switch is deferred");
        } else {
            handleConcurrentStreamSwitch(st_streams);
        }
        rm->unlockActiveStream();
    }
}

void stHandleDeferredSwitch()
{
    int32_t status = 0;
    bool switch_to_nlpi = false;
    std::vector<pal_stream_type_t> st_streams;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    do {
        status = rm->tryLockActiveStream();
    } while (!status && rm->getSoundCardState() == CARD_STATUS_ONLINE);

    if (rm->getSoundCardState() != CARD_STATUS_ONLINE) {
        if (status)
            rm->unlockActiveStream();
        PAL_DBG(LOG_TAG, "Sound card is offline");
        return;
    }

    PAL_DBG(LOG_TAG, "enter, rm->isAnyStreamBuffering:%d deferred state:%d",
        rm->isAnyStreamBuffering(), deferredSwitchState);

    if (!rm->isAnyStreamBuffering() && deferredSwitchState != NO_DEFER) {
        if (deferredSwitchState == DEFER_LPI_NLPI_SWITCH)
            switch_to_nlpi = true;
        else if (deferredSwitchState == DEFER_NLPI_LPI_SWITCH)
            switch_to_nlpi = false;

        use_lpi_ = !switch_to_nlpi;

        if (rm->getActiveStreamMap().count(PAL_STREAM_VOICE_UI))
            st_streams.push_back(PAL_STREAM_VOICE_UI);
        if (rm->getActiveStreamMap().count(PAL_STREAM_ACD))
            st_streams.push_back(PAL_STREAM_ACD);
        if (rm->getActiveStreamMap().count(PAL_STREAM_ASR))
            st_streams.push_back(PAL_STREAM_ASR);
        if (rm->getActiveStreamMap().count(PAL_STREAM_SENSOR_PCM_DATA))
            st_streams.push_back(PAL_STREAM_SENSOR_PCM_DATA);

        handleConcurrentStreamSwitch(st_streams);
        // reset the defer switch state after handling LPI/NLPI switch
        deferredSwitchState = NO_DEFER;

        // now start deferred NLPI ST streams which was not started
        // previously due to LPI to NLPI switch deferred
        if (switch_to_nlpi) {
            for (auto it = mStartDeferredStreams.begin();
                      it != mStartDeferredStreams.end(); it++) {
                (*it)->lockStreamMutex();
                (*it)->start_l();
                (*it)->unlockStreamMutex();
            }
            mStartDeferredStreams.clear();
        }
    }
    rm->unlockActiveStream();
    PAL_DBG(LOG_TAG, "Exit");
}

void HandleConcurrencyForSoundTriggerStreams(Stream* s, bool active)
{
    std::vector<pal_stream_type_t> st_streams;
    bool do_st_stream_switch = false;
    bool use_lpi_temp = use_lpi_;
    bool st_stream_conc_en = true;
    bool notify_resources_available = false;
    bool st_stream_tx_conc = false;
    bool st_stream_rx_conc = false;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    rm->lockActiveStream();
    PAL_DBG(LOG_TAG, "Enter, stream: %pK, active %d", s, active);

    if (deferredSwitchState == DEFER_LPI_NLPI_SWITCH) {
        use_lpi_temp = false;
    } else if (deferredSwitchState == DEFER_NLPI_LPI_SWITCH) {
        use_lpi_temp = true;
    }

    GetConcurrencyInfo(s, &st_stream_rx_conc, &st_stream_tx_conc, &st_stream_conc_en);

    if (st_stream_tx_conc) {
        if (active)
            TxconcurrencyEnableCount++;
        else if (TxconcurrencyEnableCount > 0)
            TxconcurrencyEnableCount--;
    }

    if (st_stream_conc_en && (st_stream_rx_conc || st_stream_tx_conc)) {
        if (!isNLPISwitchSupported()) {
            PAL_INFO(LOG_TAG,
                     "Skip switch as st stream LPI/NLPI switch disabled");
        } else if (active) {
            registerNLPIStream(s);
            if (mNLPIStreams.size() > 0) {
                if (use_lpi_temp) {
                    do_st_stream_switch = true;
                    use_lpi_temp = false;
                }
            }
        } else {
            deregisterNLPIStream(s);
            if (mNLPIStreams.size() == 0) {
                if (!(rm->getActiveStreamMap().count(PAL_STREAM_VOICE_UI) && charging_state_ &&
                    rm->IsTransitToNonLPIOnChargingSupported())) {
                    do_st_stream_switch = true;
                    use_lpi_temp = true;
                }
            }
        }
    }

    st_streams.push_back(PAL_STREAM_VOICE_UI);
    st_streams.push_back(PAL_STREAM_ACD);
    st_streams.push_back(PAL_STREAM_ASR);
    st_streams.push_back(PAL_STREAM_SENSOR_PCM_DATA);

    for (pal_stream_type_t st_stream_type : st_streams) {
        if (!st_stream_conc_en) {
            if (st_stream_type == PAL_STREAM_VOICE_UI &&
                concurrencyDisableCount == 1 && !active)
                notify_resources_available = true;
            HandleStreamPauseResume(st_stream_type, active);
        }
    }

    if (do_st_stream_switch) {
        if (checkAndUpdateDeferSwitchState(active)) {
            PAL_DBG(LOG_TAG, "Switch is deferred/cancelled");
        } else {
            use_lpi_ = use_lpi_temp;
            handleConcurrentStreamSwitch(st_streams);
        }
    }

    rm->unlockActiveStream();

    /*
     * The usecases using ST framework register the onResourcesAvailable callback.
     * Notify the framework upon concurrency is inactive.
     */
    for (int i = 0; i < onResourceAvailCbList.size() && notify_resources_available; i++) {
        onResourceAvailCbList[i].first(onResourceAvailCbList[i].second);
    }

    PAL_DBG(LOG_TAG, "Exit");
}

int setSTParameter(uint32_t param_id, void *param_payload,
                     size_t payload_size) {
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    int status = 0;

    PAL_DBG(LOG_TAG, "Enter param id: %d", param_id);
    switch (param_id) {
        case PAL_PARAM_ID_CHARGING_STATE:
            {
                pal_param_charging_state *battery_charging_state =
                    (pal_param_charging_state *)param_payload;
                bool action;
                if (rm->IsTransitToNonLPIOnChargingSupported()) {
                    if (payload_size == sizeof(pal_param_charging_state)) {
                        PAL_INFO(LOG_TAG, "Charging State = %d",
                                  battery_charging_state->charging_state);
                        if (charging_state_ ==
                            battery_charging_state->charging_state) {
                            PAL_DBG(LOG_TAG, "Charging state unchanged, ignore");
                            break;
                        }
                        charging_state_ = battery_charging_state->charging_state;
                        rm->lockActiveStream();
                        onChargingStateChange();
                        rm->unlockActiveStream();
                    } else {
                        PAL_ERR(LOG_TAG,
                                "Incorrect size : expected (%zu), received(%zu)",
                                sizeof(pal_param_charging_state), payload_size);
                        status = -EINVAL;
                    }
                } else {
                    PAL_DBG(LOG_TAG,
                              "transit_to_non_lpi_on_charging set to false\n");
                }
            }
            break;
        case PAL_PARAM_ID_RESOURCES_AVAILABLE:
            {
                pal_param_resources_available_t *resources_avail =
                    (pal_param_resources_available_t *)param_payload;
                if (resources_avail) {
                    std::pair<SoundTriggerOnResourceAvailableCallback, uint64_t> cb =
                    std::make_pair((SoundTriggerOnResourceAvailableCallback)resources_avail->callback,
                              resources_avail->cookie);
                    if (std::find(onResourceAvailCbList.begin(), onResourceAvailCbList.end(), cb) ==
                        onResourceAvailCbList.end()) {
                        onResourceAvailCbList.push_back(cb);
                        PAL_VERBOSE(LOG_TAG, "setParameter onResourceAvailCb %pk"
                                    " onResourceAvailCookie %pk", resources_avail->callback,
                                    resources_avail->cookie);
                    } else {
                        PAL_DBG(LOG_TAG, "Resource available callback is already registered");
                    }
                } else {
                    PAL_ERR(LOG_TAG, "Invalid ST resource payload");
                    status = -EINVAL;
                }
            }
            break;
        default:
            status = -ENOENT;
    }

    return status;
}

int getSTParameter(uint32_t param_id, void **param_payload,
                     size_t *payload_size, void *query __unused) {
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    int status = 0;

    PAL_DBG(LOG_TAG, "Enter param id: %d", param_id);
    switch (param_id) {
        case PAL_PARAM_ID_GET_SOUND_TRIGGER_PROPERTIES:
        {
            PAL_DBG(LOG_TAG, "get sound trigger properties, status %d", status);
            struct pal_st_properties *qstp =
                (struct pal_st_properties *)calloc(1, sizeof(struct pal_st_properties));

            GetVoiceUIProperties(qstp);

            *param_payload = qstp;
            *payload_size = sizeof(pal_st_properties);
            break;
        }
        case PAL_PARAM_ID_ST_CAPTURE_INFO:
        {
            pal_param_st_capture_info_t *stCaptureInfo =
                                    (pal_param_st_capture_info_t *) param_payload;
            int captureHandle = stCaptureInfo->capture_handle;
            if (mStCaptureInfo.find(captureHandle) != mStCaptureInfo.end()) {
                stCaptureInfo->pal_handle = mStCaptureInfo[captureHandle];
                PAL_VERBOSE(LOG_TAG, "getParameter capture handle %d, pal handle %pK",
                                                captureHandle, stCaptureInfo->pal_handle);
            } else {
                PAL_ERR(LOG_TAG, "capture handle not found %d", captureHandle);
            }
        }
        break;
        default:
            status = -ENOENT;
    }
    return status;
}

std::shared_ptr<Device> GetPalDevice(Stream *streamHandle, pal_device_id_t dev_id)
{
    std::shared_ptr<CaptureProfile> cap_prof = nullptr;
    std::shared_ptr<CaptureProfile> common_cap_prof = nullptr;
    std::shared_ptr<Device> device = nullptr;
    struct pal_device dev;
    struct pal_stream_attributes sAttr;
    int status = 0;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    if (!streamHandle) {
        PAL_ERR(LOG_TAG, "Stream is invalid");
        goto exit;
    }

    status = streamHandle->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG, "stream get attributes failed");
        goto exit;
    }

    PAL_DBG(LOG_TAG, "Enter, stream: %d, device_id: %d", sAttr.type, dev_id);

    cap_prof = streamHandle->GetCurrentCaptureProfile();

    if (!cap_prof && !GetSoundTriggerCaptureProfile() &&
        !GetTXMacroCaptureProfile()) {
        PAL_ERR(LOG_TAG, "Failed to get local and common cap_prof for stream: %d",
                sAttr.type);
        goto exit;
    }

    common_cap_prof = (dev_id == PAL_DEVICE_IN_ULTRASOUND_MIC) ?
                       GetTXMacroCaptureProfile(): GetSoundTriggerCaptureProfile();
    if (common_cap_prof) {
        /* Use the rm's common capture profile if local capture profile is not
         * available, or the common capture profile has the highest priority.
         */
        if (!cap_prof ||
            common_cap_prof->ComparePriority(cap_prof) >= CAPTURE_PROFILE_PRIORITY_HIGH) {
            PAL_DBG(LOG_TAG, "common cap_prof %s has the highest priority.",
                    common_cap_prof->GetName().c_str());
            cap_prof = common_cap_prof;
        }
    }

    dev.id = dev_id;
    dev.config.bit_width = cap_prof->GetBitWidth();
    dev.config.ch_info.channels = cap_prof->GetChannels();
    dev.config.sample_rate = cap_prof->GetSampleRate();
    dev.config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S16_LE;

    device = Device::getInstance(&dev, rm);
    if (!device) {
        PAL_ERR(LOG_TAG, "Failed to get device instance");
        goto exit;
    }

    device->setDeviceAttributes(dev);
    device->setSndName(cap_prof->GetSndName());

exit:
    PAL_DBG(LOG_TAG, "Exit");
    return device;
}

void updateCaptureProfiles() {
    std::shared_ptr<CaptureProfile> st_cap_prof_priority = nullptr;
    std::shared_ptr<CaptureProfile> tx_cap_prof_priority = nullptr;

    st_cap_prof_priority = GetCaptureProfileByPriority(nullptr, "va_macro");
    tx_cap_prof_priority = GetCaptureProfileByPriority(nullptr, "tx_macro");

    std::lock_guard<std::mutex> lck(st_utils_mutex_);
    SoundTriggerCaptureProfile = st_cap_prof_priority;
    TXMacroCaptureProfile = tx_cap_prof_priority;
}

std::shared_ptr<CaptureProfile> GetSoundTriggerCaptureProfile() {
    std::lock_guard<std::mutex> lck(st_utils_mutex_);
    return SoundTriggerCaptureProfile;
}

bool isTxConcurrencyActive() {
    return (TxconcurrencyEnableCount > 0);
}

std::shared_ptr<CaptureProfile> GetTXMacroCaptureProfile() {
    std::lock_guard<std::mutex> lck(st_utils_mutex_);
    return TXMacroCaptureProfile;
}

void registerNLPIStream(Stream *s)
{
    std::lock_guard<std::mutex> lck(st_utils_mutex_);
    PAL_DBG(LOG_TAG, "register NLPI stream: %pK", s);
    mNLPIStreams.insert(s);
}

void deregisterNLPIStream(Stream *s)
{
    std::lock_guard<std::mutex> lck(st_utils_mutex_);
    PAL_DBG(LOG_TAG, "deregister NLPI stream: %pK", s);
    mNLPIStreams.erase(s);
}

bool getLPIUsage()
{
    int status  = 0;
    bool nlpi_active = false;

    std::lock_guard<std::mutex> lck(st_utils_mutex_);
    if (mNLPIStreams.size() > 0)
        nlpi_active = true;

    return !nlpi_active && use_lpi_;
}

defer_switch_state_t getSTDeferedSwitchState()
{
    defer_switch_state_t defered_state = NO_DEFER;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    rm->lockActiveStream();
    defered_state = deferredSwitchState;
    rm->unlockActiveStream();

    return defered_state;
}

void updateDeferredSTStreams(Stream* s, bool active)
{
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    rm->lockActiveStream();
    if (active)
        mStartDeferredStreams.push_back(s);
    else
        mStartDeferredStreams.remove(s);
    rm->unlockActiveStream();
}
