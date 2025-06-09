/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "PAL: PluginManager"

#include "PluginManager.h"
#include <iostream>
#include <fstream>
#include <map>
#include "PalCommon.h"
#include <cutils/properties.h>
#include "PalDefs.h"
#include "PalMappings.h"
/*include all streams*/
#include "StreamACD.h"
#include "StreamCompress.h"
#include "StreamHaptics.h"
#include "StreamInCall.h"
#include "StreamNonTunnel.h"
#include "StreamPCM.h"
#include "StreamSoundTrigger.h"
#include "StreamCommonProxy.h"
#include "StreamContextProxy.h"
#include "StreamSensorPCMData.h"
#include "StreamSensorRenderer.h"
#include "StreamUltraSound.h"
#include "StreamASR.h"
#include "StreamDummy.h"
#include "StreamCallTranslation.h"
/*include all sessions*/
#include "SessionAgm.h"
#include "SessionAlsaCompress.h"
#include "SessionAlsaPcm.h"
#include "SessionAlsaVoice.h"
/*include all devices*/
#include "Bluetooth.h"
#include "DisplayPort.h"
#include "DummyDev.h"
#include "ECRefDevice.h"
#include "ExtEC.h"
#include "FMDevice.h"
#include "Handset.h"
#include "HandsetMic.h"
#include "HandsetVaMic.h"
#include "HapticsDev.h"
#include "Headphone.h"
#include "HeadsetMic.h"
#include "HeadsetVaMic.h"
#include "RTProxy.h"
#include "Speaker.h"
#include "SpeakerMic.h"
#include "UltrasoundDevice.h"
#include "USBAudio.h"
#include "A2B2Mic.h"
#include "A2B2Speaker.h"
#include "A2BMic.h"
#include "A2BSpeaker.h"


std::shared_ptr<PluginManager> PluginManager::pm = nullptr;
std::mutex PluginManager::mPluginManagerMutex;

PluginManager::PluginManager() {
}

PluginManager::~PluginManager() {
}

void PluginManager::deinitStreamPlugins(){
}

int32_t PluginManager::registeredPlugin(pm_item_t item, pal_plugin_manager_t type){
    return 0;
}

int32_t getStreamFunc(void** func, std::string name) {
    int32_t status = 0;

    switch (usecaseIdLUT.at(name)) {
        case PAL_STREAM_LOW_LATENCY:
        case PAL_STREAM_DEEP_BUFFER:
        case PAL_STREAM_SPATIAL_AUDIO:
        case PAL_STREAM_GENERIC:
        case PAL_STREAM_VOIP_TX:
        case PAL_STREAM_VOIP_RX:
        case PAL_STREAM_PCM_OFFLOAD:
        case PAL_STREAM_VOICE_CALL:
        case PAL_STREAM_LOOPBACK:
        case PAL_STREAM_ULTRA_LOW_LATENCY:
        case PAL_STREAM_PROXY:
        case PAL_STREAM_PLAYBACK_BUS:
        case PAL_STREAM_CAPTURE_BUS:
        case PAL_STREAM_RAW:
        case PAL_STREAM_VOICE_RECOGNITION:
            *reinterpret_cast<StreamCreate*>(func) = &CreatePCMStream;
            break;
        case PAL_STREAM_COMPRESSED:
            *reinterpret_cast<StreamCreate*>(func) = &CreateCompressStream;
            break;
        case PAL_STREAM_VOICE_UI:
            *reinterpret_cast<StreamCreate*>(func) = &CreateSoundTriggerStream;
            break;
        case PAL_STREAM_VOICE_CALL_RECORD:
        case PAL_STREAM_VOICE_CALL_MUSIC:
            *reinterpret_cast<StreamCreate*>(func) = &CreateInCallStream;
            break;
        case PAL_STREAM_CALL_TRANSLATION:
            *reinterpret_cast<StreamCreate*>(func) = &CreateCallTranslationStream;
            break;
        case PAL_STREAM_NON_TUNNEL:
            *reinterpret_cast<StreamCreate*>(func) = &CreateNonTunnelStream;
            break;
        case PAL_STREAM_ACD:
            *reinterpret_cast<StreamCreate*>(func) = &CreateACDStream;
            break;
        case PAL_STREAM_HAPTICS:
            *reinterpret_cast<StreamCreate*>(func) = &CreateHapticsStream;
            break;
        case PAL_STREAM_CONTEXT_PROXY:
            *reinterpret_cast<StreamCreate*>(func) = &CreateContextProxyStream;
            break;
        case PAL_STREAM_ULTRASOUND:
            *reinterpret_cast<StreamCreate*>(func) = &CreateUltraSoundStream;
            break;
        case PAL_STREAM_SENSOR_PCM_DATA:
            *reinterpret_cast<StreamCreate*>(func) = &CreateSensorPCMDataStream;
            break;
        case PAL_STREAM_COMMON_PROXY:
            *reinterpret_cast<StreamCreate*>(func) = &CreateCommonProxyStream;
        break;
        case PAL_STREAM_SENSOR_PCM_RENDERER:
            *reinterpret_cast<StreamCreate*>(func) = &CreateSensorRendererStream;
            break;
        case PAL_STREAM_ASR:
            *reinterpret_cast<StreamCreate*>(func) = &CreateASRStream;
            break;
        case PAL_STREAM_DUMMY:
             *reinterpret_cast<StreamCreate*>(func) = &CreateDummyStream;
            break;
        default:
            PAL_ERR(LOG_TAG, "unsupported stream type %s", name.c_str());
            break;
    }
    exit:
    return status;
}

int32_t getSessionFunc(void** func, std::string name) {
    int32_t status = 0;
    switch(usecaseIdLUT.at(name)){
        case PAL_STREAM_COMPRESSED:
            *reinterpret_cast<SessionCreate*>(func) = &CreateCompressSession;
            break;
        case PAL_STREAM_VOICE_CALL:
            *reinterpret_cast<SessionCreate*>(func) = &CreateVoiceSession;
            break;
        case PAL_STREAM_NON_TUNNEL:
            *reinterpret_cast<SessionCreate*>(func) = &CreateAgmSession;
            break;
         default:
            *reinterpret_cast<SessionCreate*>(func) = &CreatePcmSession;
            break;
    }
    return status;
}

int32_t getDeviceFunc(void** func, std::string name) {
    int32_t status = 0;
    switch(deviceIdLUT.at(name)){
        case PAL_DEVICE_NONE:
            PAL_DBG(LOG_TAG,"device none");
            *reinterpret_cast<DeviceCreate*>(func) = nullptr;
            break;
        case PAL_DEVICE_OUT_HANDSET:
            PAL_VERBOSE(LOG_TAG, "handset device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateHandsetDevice;
            break;
        case PAL_DEVICE_OUT_SPEAKER:
            PAL_VERBOSE(LOG_TAG, "speaker device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateSpeakerDevice;
            break;
        case PAL_DEVICE_IN_VI_FEEDBACK:
            PAL_VERBOSE(LOG_TAG, "speaker feedback device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateSpeakerDevice;
            break;
        case PAL_DEVICE_IN_CPS_FEEDBACK:
            PAL_VERBOSE(LOG_TAG, "speaker feedback device CPS");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateSpeakerDevice;
            break;
        case PAL_DEVICE_OUT_WIRED_HEADSET:
        case PAL_DEVICE_OUT_WIRED_HEADPHONE:
            PAL_VERBOSE(LOG_TAG, "headphone device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateHeadphoneDevice;
            break;
        case PAL_DEVICE_OUT_USB_DEVICE:
        case PAL_DEVICE_OUT_USB_HEADSET:
        case PAL_DEVICE_IN_USB_DEVICE:
        case PAL_DEVICE_IN_USB_HEADSET:
            PAL_VERBOSE(LOG_TAG, "USB device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateUsbDevice;
            break;
        case PAL_DEVICE_IN_HANDSET_MIC:
            PAL_VERBOSE(LOG_TAG, "HandsetMic device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateHandsetMicDevice;
            break;
        case PAL_DEVICE_IN_SPEAKER_MIC:
            PAL_VERBOSE(LOG_TAG, "speakerMic device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateSpeakerMicDevice;
            break;
        case PAL_DEVICE_IN_WIRED_HEADSET:
            PAL_VERBOSE(LOG_TAG, "HeadsetMic device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateHeadsetMicDevice;
            break;
        case PAL_DEVICE_IN_HANDSET_VA_MIC:
            PAL_VERBOSE(LOG_TAG, "HandsetVaMic device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateHeadsetVaDevice;
            break;
        case PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET:
        case PAL_DEVICE_OUT_BLUETOOTH_SCO:
        case PAL_DEVICE_IN_BLUETOOTH_HFP:
        case PAL_DEVICE_OUT_BLUETOOTH_HFP:
            PAL_VERBOSE(LOG_TAG, "BTSCO/HFP device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateBtDevice;
            break;
        case PAL_DEVICE_IN_BLUETOOTH_A2DP:
        case PAL_DEVICE_OUT_BLUETOOTH_A2DP:
        case PAL_DEVICE_IN_BLUETOOTH_BLE:
        case PAL_DEVICE_OUT_BLUETOOTH_BLE:
        case PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST:
            PAL_VERBOSE(LOG_TAG, "BTA2DP device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateBtDevice;
            break;
        case PAL_DEVICE_OUT_AUX_DIGITAL:
        case PAL_DEVICE_OUT_AUX_DIGITAL_1:
        case PAL_DEVICE_OUT_HDMI:
            PAL_VERBOSE(LOG_TAG, "Display Port device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateDisplayDevice;
            break;
        case PAL_DEVICE_IN_HEADSET_VA_MIC:
            PAL_VERBOSE(LOG_TAG, "HeadsetVaMic device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateHeadsetVaDevice;
            break;
        case PAL_DEVICE_OUT_PROXY:
            PAL_VERBOSE(LOG_TAG, "RTProxyOut device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateRTProxyDevice;
            break;
        case PAL_DEVICE_OUT_RECORD_PROXY:
            PAL_VERBOSE(LOG_TAG, "RTProxyOut record device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateRTProxyDevice;
            break;
        case PAL_DEVICE_OUT_HEARING_AID:
            PAL_VERBOSE(LOG_TAG, "RTProxy Hearing Aid device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateRTProxyDevice;
            break;
        case PAL_DEVICE_OUT_HAPTICS_DEVICE:
            PAL_VERBOSE(LOG_TAG, "Haptics Device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateHapticsDevice;
            break;
        case PAL_DEVICE_IN_PROXY:
            PAL_VERBOSE(LOG_TAG, "RTProxyIn device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateRTProxyDevice;
            break;
        case PAL_DEVICE_IN_RECORD_PROXY:
            PAL_VERBOSE(LOG_TAG, "RTProxyIn record device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateRTProxyDevice;
            break;
        case PAL_DEVICE_IN_TELEPHONY_RX:
            PAL_VERBOSE(LOG_TAG, "RTProxy Telephony Rx device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateRTProxyDevice;
            break;
        case PAL_DEVICE_IN_FM_TUNER:
            PAL_VERBOSE(LOG_TAG, "FM device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateFmDevice;
            break;
        case PAL_DEVICE_IN_ULTRASOUND_MIC:
        case PAL_DEVICE_OUT_ULTRASOUND:
        case PAL_DEVICE_OUT_ULTRASOUND_DEDICATED:
            PAL_VERBOSE(LOG_TAG, "Ultrasound device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateUltrasoundDevice;
            break;
        case PAL_DEVICE_IN_EXT_EC_REF:
            PAL_VERBOSE(LOG_TAG, "ExtEC device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateExtEcDevice;
            break;
        case PAL_DEVICE_IN_ECHO_REF:
            PAL_VERBOSE(LOG_TAG, "Echo ref device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateECRefDevice;
            break;
        case PAL_DEVICE_OUT_DUMMY:
        case PAL_DEVICE_IN_DUMMY:
            PAL_VERBOSE(LOG_TAG, "Dummy device");
            *reinterpret_cast<DeviceCreate*>(func) = &CreateDummyDevice;
            break;
        case PAL_DEVICE_OUT_SPEAKER2:
            *reinterpret_cast<DeviceCreate*>(func) = &Createa2bspeakerDevice;
            break;
        case PAL_DEVICE_OUT_SPEAKER3:
            *reinterpret_cast<DeviceCreate*>(func) = &Createa2b2speakerDevice;
            break;
        case PAL_DEVICE_IN_SPEAKER_MIC2:
            *reinterpret_cast<DeviceCreate*>(func) = &Createa2bmicDevice;
            break;
        case PAL_DEVICE_IN_SPEAKER_MIC3:
            *reinterpret_cast<DeviceCreate*>(func) = &Createa2b2micDevice;
            break;
        default:
            PAL_ERR(LOG_TAG, "unsupported device type %s", name.c_str());
            status = -EINVAL;
            break;
    }
    return status;
}

int32_t PluginManager::openPlugin(pal_plugin_manager_t type, std::string keyName, void* &plugin){
    int32_t status = 0;
    std::vector<pm_item_t> *pluginList = nullptr;

    PAL_DBG(LOG_TAG, "Enter");
    mPluginManagerMutex.lock();
    switch (type) {
        case PAL_PLUGIN_MANAGER_STREAM:
            status = getStreamFunc(&plugin, keyName);
            break;
        case PAL_PLUGIN_MANAGER_SESSION:
            status = getSessionFunc(&plugin, keyName);
            break;
        case PAL_PLUGIN_MANAGER_DEVICE:
            status = getDeviceFunc(&plugin, keyName);
            break;
        default:
            PAL_ERR(LOG_TAG, "unsupported Plugin type %d", type);
            status = -EINVAL;
            break;
    }
    if(status){
        PAL_ERR(LOG_TAG, "failed to get function pointer for %s", keyName.c_str());
    }
    if (!plugin) {
        PAL_ERR(LOG_TAG, "cannot find the stream for stream type %s",
                keyName.c_str());
        status = -EINVAL;
    }
    exit:
    PAL_DBG(LOG_TAG, "exit status: %d", status);
    mPluginManagerMutex.unlock();
    return status;
}

int32_t  PluginManager::closePlugin(pal_plugin_manager_t type, std::string keyName) {
    return 0;
}

/*public APIs*/

std::shared_ptr<PluginManager> PluginManager::getInstance()
{
    if (!pm) {
        std::lock_guard<std::mutex> lock(PluginManager::mPluginManagerMutex);
        std::shared_ptr<PluginManager> sp(new PluginManager());
        pm = sp;
    }
    return pm;
}

