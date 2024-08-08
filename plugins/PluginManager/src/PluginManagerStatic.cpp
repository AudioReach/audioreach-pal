/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
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
#include "StreamACDB.h"
#include "StreamASR.h"
/*include all sessions*/
#include "SessionAgm.h"
#include "SessionAlsaCompress.h"
#include "SessionAlsaPcm.h"
#include "SessionAlsaVoice.h"

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

int32_t getStreamFunc(void* func, std::string name) {
    int32_t status = 0;

    if( name =="PAL_USE_ACDB_STREAM"){
         func = (void*)&CreateACDBStream;
         goto exit;
    }
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
        case PAL_STREAM_RAW:
        case PAL_STREAM_VOICE_RECOGNITION:
            func = (void*)&CreatePCMStream;
            break;
        case PAL_STREAM_COMPRESSED:
            func = (void*)&CreateCompressStream;
            break;
        case PAL_STREAM_VOICE_UI:
            func = (void*)&CreateSoundTriggerStream;
            break;
        case PAL_STREAM_VOICE_CALL_RECORD:
        case PAL_STREAM_VOICE_CALL_MUSIC:
            func = (void*)&CreateInCallStream;
            break;
        case PAL_STREAM_NON_TUNNEL:
            func = (void*)&CreateNonTunnelStream;
            break;
        case PAL_STREAM_ACD:
            func = (void*)&CreateACDStream;
            break;
        case PAL_STREAM_HAPTICS:
            func = (void*)&CreateHapticsStream;
            break;
        case PAL_STREAM_CONTEXT_PROXY:
            func = (void*)&CreateContextProxyStream;
            break;
        case PAL_STREAM_ULTRASOUND:
            func = (void*)&CreateUltraSoundStream;
            break;
        case PAL_STREAM_SENSOR_PCM_DATA:
            func = (void*)&CreateSensorPCMDataStream;
            break;
        case PAL_STREAM_COMMON_PROXY:
            func = (void*)&CreateCommonProxyStream;
        break;
        case PAL_STREAM_SENSOR_PCM_RENDERER:
            func = (void*)&CreateSensorRendererStream;
            break;
        case PAL_STREAM_ASR:
            func = (void*)&CreateASRStream;
        default:
            PAL_ERR(LOG_TAG, "unsupported stream type %s", name.c_str());
            break;
    }
    exit:
    return status;
}

int32_t getSessionFunc(void* func, std::string name) {
    int32_t status = 0;
    switch(usecaseIdLUT.at(name)){
        case PAL_STREAM_COMPRESSED:
            func = (void*)&CreateCompressSession;
            break;
        case PAL_STREAM_VOICE_CALL:
            func = (void*)&CreateVoiceSession;
            break;
        case PAL_STREAM_NON_TUNNEL:
            func = (void*)&CreateAgmSession;
            break;
         default:
            func = (void*)&CreatePcmSession;
            break;
    }
    return status;
}

int32_t getDeviceFunc(void* func, std::string name) {
    int32_t status = 0;
    switch(deviceIdLUT.at(name)){
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
            status = getStreamFunc(plugin, keyName);
            break;
        case PAL_PLUGIN_MANAGER_SESSION:
            status = getSessionFunc(plugin, keyName);
            break;
        case PAL_PLUGIN_MANAGER_DEVICE:
            status = getDeviceFunc(plugin, keyName);
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

