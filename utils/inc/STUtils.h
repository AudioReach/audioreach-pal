/*
 * Copyright (c) 2023-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef ST_UTILS_H
#define ST_UTILS_H

#include <stdio.h>
#ifndef VUI_DMGR_AUDIO_UNSUPPORTED
#include <vui_dmgr_audio_intf.h>
#endif
#include "ResourceManager.h"
#include "Stream.h"
#include "SoundTriggerPlatformInfo.h"

typedef void (*SoundTriggerOnResourceAvailableCallback)(uint64_t cookie);

static void voiceUIDeferredSwitchLoop(std::shared_ptr<ResourceManager> rm);
void onChargingStateChange();
void STUtilsInit();
void STUtilsDeinit();
#ifndef VUI_DMGR_AUDIO_UNSUPPORTED
void voiceuiDmgrManagerInit();
void voiceuiDmgrManagerDeInit();
int32_t voiceuiDmgrPalCallback(int32_t param_id, void *payload, size_t payload_size);
int32_t voiceuiDmgrRestartUseCases(vui_dmgr_param_restart_usecases_t *uc_info);
#endif
void GetVoiceUIProperties(struct pal_st_properties *qstp);
bool isNLPISwitchSupported();
void registerNLPIStream(Stream* s);
void deregisterNLPIStream(Stream* s);
void GetSTDisableConcurrencyCount_l(pal_stream_type_t type, int32_t *disable_count);
bool IsLowLatencyBargeinSupported();
bool IsAudioCaptureConcurrencySupported();
bool IsVoiceCallConcurrencySupported();
bool IsVoipConcurrencySupported();
bool CheckForForcedTransitToNonLPI();
std::shared_ptr<CaptureProfile> GetASRCaptureProfileByPriority(
    Stream *s, std::shared_ptr<CaptureProfile> cap_prof_priority, std::string backend);
std::shared_ptr<CaptureProfile> GetACDCaptureProfileByPriority(
    Stream *s, std::shared_ptr<CaptureProfile> cap_prof_priority, std::string backend);
std::shared_ptr<CaptureProfile> GetSVACaptureProfileByPriority(
    Stream *s, std::shared_ptr<CaptureProfile> cap_prof_priority, std::string backend);
std::shared_ptr<CaptureProfile> GetSPDCaptureProfileByPriority(
    Stream *s, std::shared_ptr<CaptureProfile> cap_prof_priority, std::string backend);
std::shared_ptr<CaptureProfile> GetCaptureProfileByPriority(Stream *s, std::string backend);
bool UpdateSoundTriggerCaptureProfile(Stream *s, bool is_active);
int HandleDetectionStreamAction(pal_stream_type_t type, int32_t action, void *data);
int StopOtherDetectionStreams(void *st);
int StartOtherDetectionStreams(void *st);
void HandleStreamPauseResume(pal_stream_type_t st_type, bool active);
void RegisterSTCaptureHandle(pal_param_st_capture_info_t stCaptureInfo, bool start);
bool checkAndUpdateDeferSwitchState(bool stream_active);

void handleConcurrentStreamSwtch(std::vector<pal_stream_type_t>& st_streams);
void SwitchSoundTriggerDevices(bool connect_state, pal_device_id_t st_device);
void GetSTDisableConcurrencyCount(pal_stream_type_t type, int32_t *disable_count);
void GetConcurrencyInfo(Stream* s, bool *rx_conc, bool *tx_conc, bool *conc_en);
void HandleConcurrencyForSoundTriggerStreams(Stream* s, bool active);
void stHandleDeferredSwitch();
void onVUIStreamRegistered();
void onVUIStreamDeregistered();
void forceSwitchSoundTriggerStreams(bool active);
std::shared_ptr<CaptureProfile> GetSoundTriggerCaptureProfile();
int setSTParameter(uint32_t param_id, void *param_payload, size_t payload_size);
int getSTParameter(uint32_t param_id, void **param_payload, size_t *payload_size, void *query);
std::shared_ptr<Device> GetPalDevice(Stream *streamHandle, pal_device_id_t dev_id);
void updateCaptureProfiles();
bool isTxConcurrencyActive();
std::shared_ptr<CaptureProfile> GetTXMacroCaptureProfile();
bool getLPIUsage();
void updateDeferredSTStreams(Stream* s, bool active);
defer_switch_state_t getSTDeferedSwitchState();

#endif
