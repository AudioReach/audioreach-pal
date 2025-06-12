/*
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "PalServerNotify"
#include "PalServerNotify.h"
#include <aidl/vendor/qti/hardware/pal/PalMessageQueueFlagBits.h>
#include <cutils/android_filesystem_config.h>
#include <aidlcommonsupport/NativeHandle.h>
#include <pal/BinderStatus.h>
#include <pal/SharedMemoryWrapper.h>
#include <pal/Utils.h>

using ndk::ScopedAStatus;

namespace aidl::vendor::qti::hardware::paleventnotifier {

static list_declare(client_list);
static pthread_mutex_t client_list_lock = PTHREAD_MUTEX_INITIALIZER;

int is_device_specified(ClientInfo *client_handle, PalCallbackConfig *palCallbackConfig, uint32_t event) {
    for (int i = 0; i < client_handle->deviceID.size(); i++) {
        int k = 0;
        for (int j = 0; j < palCallbackConfig->noOfCurrentDevices; j++) {
            if ((palCallbackConfig->currentDevices[j] == client_handle->deviceID[i]) ||((k < palCallbackConfig->noOfPrevDevices)
               && (palCallbackConfig->prevDevices[k] == client_handle->deviceID[i]) && (event == PAL_NOTIFY_DEVICESWITCH))) {
                return true;
            }
        }
        k++;
    }
    return false;
}

bool is_event_specified(uint32_t event) {
    bool rc = true;
    switch (event) {
        case PAL_NOTIFY_CALL_TRANSLATION_TEXT:
            break;
        default:
            rc = false;
            ALOGD("%s: Event %d is not a specified translation event", LOG_TAG, event);
            break;
     }
     return rc;
}

bool is_stream_specified(ClientInfo *client_handle, PalCallbackConfig *palCallbackConfig) {
    for (int i = 0; i < client_handle->streamType.size(); i++) {
        if (palCallbackConfig->streamAttributes.type == client_handle->streamType[i])
            return true;
    }
    return false;
}

bool is_notification_required(ClientInfo *client_handle, PalCallbackConfig *palCallbackConfig, uint32_t event) {
    bool streamSpecified, deviceSpecified, eventSpecified;
    bool ret = false;
    streamSpecified = is_stream_specified(client_handle, palCallbackConfig);
    deviceSpecified = is_device_specified(client_handle, palCallbackConfig, event);
    eventSpecified = is_event_specified(event);
    if (streamSpecified && deviceSpecified) {
        ret = true;
    } else if(client_handle->deviceID.empty() && client_handle->streamType.empty()) {
        ret = true;
    } else if(streamSpecified  && client_handle->deviceID.empty()) {
        ret = true;
    } else if(deviceSpecified  && client_handle->streamType.empty()) {
        ret = true;
    } else if (eventSpecified) {
        ret = true;
    }
    return ret;
}

void convertEvent(pal_nmt_event *palEvent, uint16_t event_id, PalCallbackEvent &aidlEvent) {
    if (!palEvent) {
        ALOGE("%s: Null palEvent pointer provided", LOG_TAG);
        return;
    }

    struct pal_nmt_engine_event *engEvent = nullptr;
    int offset = 0;

    if (event_id != PAL_NOTIFY_CALL_TRANSLATION_TEXT) {
        ALOGE("%s: Invalid event_id %d provided", LOG_TAG, event_id);
        return;
    }

    aidlEvent.event_id = static_cast<PalCallbackEventType>(event_id);
    for (int i = 0; i < palEvent->num_events; i++) {
        engEvent = &palEvent->event[offset];
        palEvent->event[i].output_text_size = MAX_TRANSCRIPTION_CHAR_SIZE;
        palEvent->event[i].input_text_size = MAX_TRANSCRIPTION_CHAR_SIZE;
        palEvent->event[i].json_size = MAX_JSON_CHAR_SIZE;
        offset += sizeof(struct pal_nmt_engine_event) + engEvent->data_size;
    }
    const uint8_t *src = reinterpret_cast<const uint8_t *>(palEvent);
    size_t textEventSize = sizeof(struct pal_nmt_event) + offset;
    aidlEvent.payload.resize(textEventSize);
    memcpy(&aidlEvent.payload[0], src, textEventSize);
}

int handle_pal_cb(pal_callback_config_t *config, uint32_t event, bool is_register) {

    ALOGD("handle_pal_cb event is %d", event);
    ClientInfo *client_handle = NULL;
    struct listnode* node = NULL;
    uint16_t inChannels = 0;
    uint16_t outChannels = 0;
    PalCallbackConfig palCallbackConfig;
    PalCallbackEvent event_payload {};
    if(config) {
        if(event == PAL_NOTIFY_CALL_TRANSLATION_TEXT) {
            struct pal_nmt_event *palNmtEvent = nullptr;
            palNmtEvent = (struct pal_nmt_event*) config->event;
            convertEvent(palNmtEvent, event, event_payload);
        }
        else {
            // Set channel values if config is valid
            inChannels = config->streamAttributes.in_media_config.ch_info.channels;
            outChannels = config->streamAttributes.out_media_config.ch_info.channels;
            palCallbackConfig.currentDevices.resize(config->noOfCurrentDevices);
            for(int i = 0; i < config->noOfCurrentDevices; i++) {
                palCallbackConfig.currentDevices[i] = (PalDeviceId)config->currentDevices[i];
            }
            if(event == PAL_NOTIFY_DEVICESWITCH) {
                palCallbackConfig.prevDevices.resize(config->noOfPrevDevices);
                for(int i = 0; i < config->noOfPrevDevices; i++) {
                     palCallbackConfig.prevDevices[i] = (PalDeviceId)config->prevDevices[i];
                }
            }
            palCallbackConfig.noOfCurrentDevices = config->noOfCurrentDevices;
            palCallbackConfig.noOfPrevDevices = config->noOfPrevDevices;
            palCallbackConfig.streamAttributes.type = (PalStreamType)config->streamAttributes.type;
            palCallbackConfig.streamAttributes.info.version = config->streamAttributes.info.opt_stream_info.version;
            palCallbackConfig.streamAttributes.info.size = config->streamAttributes.info.opt_stream_info.size;
            palCallbackConfig.streamAttributes.info.durationUs = config->streamAttributes.info.opt_stream_info.duration_us;
            palCallbackConfig.streamAttributes.info.rxProxyType = config->streamAttributes.info.opt_stream_info.rx_proxy_type;
            palCallbackConfig.streamAttributes.info.txProxyType = config->streamAttributes.info.opt_stream_info.tx_proxy_type;
            palCallbackConfig.streamAttributes.info.hasVideo = config->streamAttributes.info.opt_stream_info.has_video;
            palCallbackConfig.streamAttributes.info.isStreaming = config->streamAttributes.info.opt_stream_info.is_streaming;
            palCallbackConfig.streamAttributes.info.loopbackType = config->streamAttributes.info.opt_stream_info.loopback_type;
            palCallbackConfig.streamAttributes.info.hapticsType = config->streamAttributes.info.opt_stream_info.haptics_type;
            palCallbackConfig.streamAttributes.flags = (PalStreamFlag)config->streamAttributes.flags;
            palCallbackConfig.streamAttributes.direction = (PalStreamDirection)config->streamAttributes.direction;
            palCallbackConfig.streamAttributes.inMediaConfig.sampleRate = config->streamAttributes.in_media_config.sample_rate;
            palCallbackConfig.streamAttributes.inMediaConfig.bitwidth = config->streamAttributes.in_media_config.bit_width;
            palCallbackConfig.streamAttributes.outMediaConfig.sampleRate = config->streamAttributes.out_media_config.sample_rate;
            palCallbackConfig.streamAttributes.outMediaConfig.bitwidth = config->streamAttributes.out_media_config.bit_width;
            if(inChannels) {
                palCallbackConfig.streamAttributes.inMediaConfig.chInfo.channels = config->streamAttributes.in_media_config.ch_info.channels;
                memset(&palCallbackConfig.streamAttributes.inMediaConfig.chInfo.chMap, 0, sizeof(uint8_t[PAL_MAX_CHANNELS_SUPPORTED]));
                memcpy(&palCallbackConfig.streamAttributes.inMediaConfig.chInfo.chMap, &config->streamAttributes.in_media_config.ch_info.ch_map, inChannels);
            }
            palCallbackConfig.streamAttributes.inMediaConfig.audioFormatId = (PalAudioFmt)config->streamAttributes.in_media_config.aud_fmt_id;
            if(outChannels) {
                palCallbackConfig.streamAttributes.outMediaConfig.chInfo.channels = config->streamAttributes.out_media_config.ch_info.channels;
                memset(&palCallbackConfig.streamAttributes.outMediaConfig.chInfo.chMap, 0, sizeof(uint8_t[PAL_MAX_CHANNELS_SUPPORTED]));
                memcpy(&palCallbackConfig.streamAttributes.outMediaConfig.chInfo.chMap, &config->streamAttributes.out_media_config.ch_info.ch_map, outChannels);
            }
            palCallbackConfig.streamAttributes.outMediaConfig.audioFormatId = (PalAudioFmt)config->streamAttributes.out_media_config.aud_fmt_id;
        }
    }

    list_for_each(node, &client_list) {
        client_handle = node_to_item(node, ClientInfo, list);
        if(is_notification_required(client_handle, &palCallbackConfig, event) == true) {
            if(is_register) {
                ALOGE("PAL_NOTIFY_START is_register notification for client %d", client_handle->pid);
                if(client_handle->isnotified == false)
                    client_handle->pal_clbk->onStart(palCallbackConfig);
            }
            else {
                if(event == PAL_NOTIFY_START) {
                    ALOGD("PAL_NOTIFY_START notification for client %d", client_handle->pid);
                    client_handle->pal_clbk->onStart(palCallbackConfig);
                }
                else if (event == PAL_NOTIFY_STOP) {
                    ALOGD("PAL_NOTIFY_STOP notification for client %d", client_handle->pid);
                    client_handle->pal_clbk->onStop(palCallbackConfig);
                }
                else if (event == PAL_NOTIFY_DEVICESWITCH) {
                    ALOGD("PAL_NOTIFY_DEVICESWITCH notification for client %d", client_handle->pid);
                    client_handle->pal_clbk->onDeviceSwitch(palCallbackConfig);
                } else if (event == PAL_NOTIFY_CALL_TRANSLATION_TEXT) {
                    ALOGD("PAL_NOTIFY_CALLTRANSLATION notification for client %d", client_handle->pid);
                    client_handle->pal_clbk->onEventCallback(event_payload);
                }
            }
        }
    }
    return 0;
}

void removeClient(int pid) {
    ClientInfo *client_handle = NULL;
    struct listnode* node = NULL;
        pthread_mutex_lock(&client_list_lock);
        list_for_each(node, &client_list) {
            client_handle = node_to_item(node, ClientInfo, list);
            if (client_handle->pid == pid) {
                list_remove(node);
                free(client_handle);
                ALOGI("removed client with pid: %d", client_handle->pid);
                break;
                }
            }
        pthread_mutex_unlock(&client_list_lock);
}

void ClientInfo::onDeath(void *cookie) {
    ClientInfo *client = static_cast<ClientInfo *>(cookie);
    ALOGE("Client died pid: %d", client->getPid());
    removeClient(client->getPid());
}

void ClientInfo::linkToDeath(const std::shared_ptr<IPALEventNotifierCallback>& cb, ClientInfo *client_handle) {
    auto linkRet = AIBinder_linkToDeath(cb->asBinder().get(), mDeathRecipient.get(),
                                        this);
    if (linkRet != STATUS_OK) {
        ALOGE("linkToDeath failed pid %d", pid);
    } else {
        ALOGD("linkToDeath success for client pid %d", pid);
    }
}
::ndk::ScopedAStatus PalServerNotify::ipc_pal_notify_register_callback(const std::shared_ptr<IPALEventNotifierCallback>& cb, int* ret) {

    int status = 0;
    return status_tToBinderResult(status);
}


::ndk::ScopedAStatus PalServerNotify::ipc_pal_notify_register_callback_v2(const std::shared_ptr<IPALEventNotifierCallback>& cb,
    const std::vector<PalDeviceId>& devID, const std::vector<PalStreamType>& streamType, int* ret) {

    int pid = AIBinder_getCallingPid();
    int status = 0;
    ClientInfo *client_handle = NULL;
    struct listnode* node = NULL;
    int client_unregistered = -EINVAL;
    ALOGE("register callback for client with PID %d", pid);
    if (cb == NULL) {
        removeClient(pid);
        client_unregistered = 0;
        return status_tToBinderResult(client_unregistered);
    }
    pthread_mutex_lock(&client_list_lock);
    list_for_each(node, &client_list) {
        client_handle = node_to_item(node, ClientInfo, list);
        if (client_handle->pid == pid)
            goto registered_cb;
    }
    client_handle = new ClientInfo(pid);
    if (client_handle == NULL) {
        ALOGE("%s: Cannot allocate memory for client handle\n", __func__);
        pthread_mutex_unlock(&client_list_lock);
        return status_tToBinderResult(-ENOMEM);
    }
    client_handle->pid = pid;
    client_handle->linkToDeath(cb, client_handle);
    if (devID.size() != 0) {
        client_handle->deviceID.resize(devID.size());
        for(int i = 0; i < devID.size(); i++) {
            client_handle->deviceID[i] = static_cast<PalDeviceId>(devID[i]);
        }
    }
    if (streamType.size() != 0) {
        client_handle->streamType.resize(streamType.size());
        for(int i = 0; i < streamType.size(); i++) {
            client_handle->streamType[i] = static_cast<PalStreamType>(streamType[i]);
        }
    }
    list_add_tail(&client_list, &client_handle->list);
registered_cb:
    client_handle->pal_clbk = cb;
    pthread_mutex_unlock(&client_list_lock);
    status = pal_register_for_events(handle_pal_cb);
    client_handle->isnotified = true;
    return status_tToBinderResult(status);
}
}
