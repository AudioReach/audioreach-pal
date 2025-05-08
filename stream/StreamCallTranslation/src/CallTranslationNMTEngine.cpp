/*
 * Copyright (c) 2021 The Linux Foundation. All rights reserved.
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

#define ATRACE_TAG (ATRACE_TAG_AUDIO | ATRACE_TAG_HAL)
#define LOG_TAG "PAL: CallTranslationNMTEngine"

#include "CallTranslationNMTEngine.h"
#include <cmath>
#include <cutils/trace.h>
#include <string.h>
#include "Session.h"
#include "SessionAR.h"
#include "Stream.h"
#include "StreamCallTranslation.h"
#include "ResourceManager.h"
#include "kvh2xml.h"

std::shared_ptr<CallTranslationNMTEngine> CallTranslationNMTEngine::engRX = nullptr;
std::shared_ptr<CallTranslationNMTEngine> CallTranslationNMTEngine::engTX = nullptr;

CallTranslationNMTEngine::CallTranslationNMTEngine(Stream *s)
{
    PAL_DBG(LOG_TAG, "Enter");
    int status = 0;
    streamHandle = s;
    nmtMiid = 0;
    exitThread = false;
    StreamCallTranslation* sCallTranslation = dynamic_cast<StreamCallTranslation *>(s);
    std::shared_ptr<ResourceManager> rm = nullptr;
    engTXState = NMT_ENG_IDLE;
    engRXState = NMT_ENG_IDLE;
    text_type = CALL_TRANSLATION_INOUT_TEXT;
    status = streamHandle->getStreamAttributes(&sAttr);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to get stream attributes");
        throw std::runtime_error("Failed to get stream attributes");
    }
    rm = ResourceManager::getInstance();
    if (!rm) {
        PAL_ERR(LOG_TAG, "Failed to get ResourceManager instance");
        throw std::runtime_error("Failed to get ResourceManager instance");
    }
    if (sAttr.direction == PAL_AUDIO_INPUT) {
        nmtTxSession = sCallTranslation->GetSession();
        if (!nmtTxSession) {
            PAL_ERR(LOG_TAG, "Failed to create NMT TX session");
            throw std::runtime_error("Failed to create NMT session");
        }
        nmtTxSession->registerCallBack(HandleSessionCallBack, (uint64_t)this);
        eventTxThreadHandler = std::thread(CallTranslationNMTEngine::EventProcessingThread, this);
        if (!eventTxThreadHandler.joinable()) {
            PAL_ERR(LOG_TAG, "Error:%d failed to create event processing thread",
                    status);
            throw std::runtime_error("Failed to create event processing thread");
         }
    } else if (sAttr.direction == PAL_AUDIO_OUTPUT) {
        nmtRxSession = sCallTranslation->GetSession();
        if (!nmtRxSession) {
            PAL_ERR(LOG_TAG, "Failed to create NMT RX session");
            throw std::runtime_error("Failed to create NMT session");
        }
        nmtRxSession->registerCallBack(HandleSessionCallBack, (uint64_t)this);
        eventRxThreadHandler = std::thread(CallTranslationNMTEngine::EventProcessingThread, this);
        if (!eventRxThreadHandler.joinable()) {
            PAL_ERR(LOG_TAG, "Error:%d failed to create event processing thread",
                    status);
            throw std::runtime_error("Failed to create event processing thread");
        }
    }
}

CallTranslationNMTEngine::~CallTranslationNMTEngine()
{
    streamHandle = nullptr;
    {
        std::unique_lock<std::mutex> lck(mutexEngine);
        exitThread = true;
        cv.notify_one();
    }
    if (eventTxThreadHandler.joinable()) {
        eventTxThreadHandler.join();
    }
    if (eventRxThreadHandler.joinable()) {
        eventRxThreadHandler.join();
    }
    if (builder) {
        delete builder;
        builder = nullptr;
    }

    PAL_INFO(LOG_TAG, "Exit");
}

std::shared_ptr<CallTranslationNMTEngine> CallTranslationNMTEngine::GetInstance(Stream *s)
{
    pal_stream_attributes strAttr;
    int32_t status = 0;
    std::shared_ptr<CallTranslationNMTEngine> eng = nullptr;
    status = s->getStreamAttributes(&strAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG, "Failed to get stream attributes, status: %d", status);
        return nullptr;
    }

    if(strAttr.direction == PAL_AUDIO_INPUT && !engTX ) {
        engTX = std::make_shared<CallTranslationNMTEngine>(s);
        eng = engTX;
    } else if (strAttr.direction == PAL_AUDIO_OUTPUT && !engRX) {
        engRX = std::make_shared<CallTranslationNMTEngine>(s);
        eng = engRX;
    } else {
        PAL_ERR(LOG_TAG, " get engine instance fail");
    }
    return eng;
}

void CallTranslationNMTEngine::ParseEventAndNotifyStream() {
    PAL_DBG(LOG_TAG, "Enter.");

    int32_t status = 0;
    bool eventStatus = false;
    void *payload = nullptr;
    uint8_t *temp = nullptr;
    size_t eventSize = 0;
    event_id_nmt_event_t *event = nullptr;
    nmt_output_status_t *ev = nullptr;
    nmt_io_status_t *io_ev = nullptr;
    nmt_input_status_t *in_ev = nullptr;
    eventPayload eventToStream = {};
    param_id_nmt_output_t *eventHeader = nullptr;
    StreamCallTranslation *sAsr = nullptr;
    event = (struct event_id_nmt_event_t *)eventQ.front();
    if (event == nullptr) {
        PAL_ERR(LOG_TAG, "Invalid event!!!");
        goto exit;
    }

    PAL_INFO(LOG_TAG, "output token : %d, num output : %d, payload size : %d",
             event->token, event->num_outputs, event->payload_size);

    if (event->num_outputs == 0) {
        PAL_ERR(LOG_TAG, "event raised without any transcript");
        goto exit;
    }
    /**
     * Don't move following variable updates after the getParam, as these variables
     * will be used by payload builder while handling the getParam.
     */
    numOutput = event->num_outputs;
    outputToken = event->token;
    payloadSize = event->payload_size;

    if (sAttr.direction == PAL_AUDIO_INPUT) {
        status = dynamic_cast<SessionAR*>(nmtTxSession)->getParamWithTag(streamHandle, TRANSLATION_NMT,
                                                                   PAL_PARAM_ID_NMT_OUTPUT,
                                                                   &payload);
    } else if (sAttr.direction == PAL_AUDIO_OUTPUT) {
        status = dynamic_cast<SessionAR*>(nmtRxSession)->getParamWithTag(streamHandle, TRANSLATION_NMT,
                                                                  PAL_PARAM_ID_NMT_OUTPUT,
                                                                  &payload);
    }
    if (status != 0) {
        PAL_ERR(LOG_TAG, "Failed to get output payload");
        goto cleanup;
    }
    numOutput = 0;
    outputToken = 0;
    payloadSize = 0;

    temp = (uint8_t *)payload;
    eventHeader = (struct param_id_nmt_output_t *)temp;
    if (text_type == CALL_TRANSLATION_INOUT_TEXT) {
        io_ev = (nmt_io_status_t *)(temp + sizeof(struct param_id_nmt_output_t));
        pal_nmt_event *eventPayload = nullptr;

        eventToStream.type = CALL_TRANSLATION_INOUT_TEXT;
        eventToStream.payloadSize = sizeof(pal_nmt_event) + eventHeader->num_outputs *
                                    sizeof(pal_nmt_engine_event);
        eventToStream.payload = calloc(1, eventToStream.payloadSize);
        if (eventToStream.payload == nullptr) {
            PAL_ERR(LOG_TAG, "Failed to allocate memory for stream event!!!");
            goto cleanup;
        }
        PAL_DBG(LOG_TAG, " event header token : %d, num output : %d, payload size : %d",
                eventHeader->token, eventHeader->num_outputs, eventHeader->payload_size);
        eventPayload = (pal_nmt_event *)eventToStream.payload;
        eventPayload->num_events = eventHeader->num_outputs;
        for (int i = 0; i < eventHeader->num_outputs; i++) {
             eventStatus = (io_ev[i].status == 0 ? true : false);
             if (!eventStatus) {
                 PAL_INFO(LOG_TAG, "Recieved failure event, ignoring this event!!!");
                 goto cleanup;
             }
             eventPayload->status = 0;
             eventPayload->event[i].is_final = io_ev[i].is_final;
             eventPayload->direction = sAttr.direction;
             eventPayload->event[i].output_text_size = io_ev[i].output_text_size< 0 ? 0 : io_ev[i].output_text_size;
             for (int j = 0; j < io_ev[i].output_text_size; ++j) {
                  eventPayload->event[i].output_text[j] = io_ev[i].output_text[j];
             }

             eventPayload->event[i].input_text_size = io_ev[i].input_text_size< 0 ? 0 : io_ev[i].input_text_size;
             for (int j = 0; j < io_ev[i].input_text_size; ++j) {
                  eventPayload->event[i].input_text[j] = io_ev[i].input_text[j];
             }
             sAsr = dynamic_cast<StreamCallTranslation *>(streamHandle);
             eventPayload->input_language_code = (uint32_t)(sAsr->GetNmtConfig().input_language_code);
             eventPayload->output_language_code = (uint32_t)(sAsr->GetNmtConfig().output_language_code);
             sAsr->HandleEventData(eventToStream);
        }
    } else if(text_type == CALL_TRANSLATION_IN_TEXT) {
        in_ev = (nmt_input_status_t *)(temp + sizeof(struct param_id_nmt_output_t));
        pal_nmt_event *eventPayload = nullptr;

        eventToStream.type = CALL_TRANSLATION_IN_TEXT;
        eventToStream.payloadSize = sizeof(pal_nmt_event) + eventHeader->num_outputs *
                                    sizeof(pal_nmt_engine_event);
        eventToStream.payload = calloc(1, eventToStream.payloadSize);
        if (eventToStream.payload == nullptr) {
            PAL_ERR(LOG_TAG, "Failed to allocate memory for stream event!!!");
            goto cleanup;
        }
        PAL_DBG(LOG_TAG, " event header token : %d, num output : %d, payload size : %d",
                eventHeader->token, eventHeader->num_outputs, eventHeader->payload_size);
        eventPayload = (pal_nmt_event *)eventToStream.payload;
        eventPayload->num_events = eventHeader->num_outputs;
        for (int i = 0; i < eventHeader->num_outputs; i++) {
             eventStatus = (in_ev[i].status == 0 ? true : false);
             if (!eventStatus) {
                 PAL_INFO(LOG_TAG, "Recieved failure event, ignoring this event!!!");
                 goto cleanup;
             }
             eventPayload->status = 0;
             eventPayload->event[i].output_text_size = 0;
             strlcpy(eventPayload->event[i].output_text, "", sizeof(eventPayload->event[i].output_text));
             eventPayload->event[i].is_final = in_ev[i].is_final;
             eventPayload->direction = sAttr.direction;
             eventPayload->event[i].input_text_size = in_ev[i].input_text_size< 0 ? 0 : in_ev[i].input_text_size;
             for (int j = 0; j < in_ev[i].input_text_size; ++j) {
                  eventPayload->event[i].input_text[j] = in_ev[i].input_text[j];
             }
             sAsr = dynamic_cast<StreamCallTranslation *>(streamHandle);
             eventPayload->input_language_code = (uint32_t)(sAsr->GetNmtConfig().input_language_code);
             eventPayload->output_language_code = (uint32_t)(sAsr->GetNmtConfig().output_language_code);
             sAsr->HandleEventData(eventToStream);
        }
    } else if (text_type == CALL_TRANSLATION_OUT_TEXT) {
        ev = (nmt_output_status_t *)(temp + sizeof(struct param_id_nmt_output_t));
        pal_nmt_event *eventPayload = nullptr;
        eventToStream.type = CALL_TRANSLATION_OUT_TEXT;
        eventToStream.payloadSize = sizeof(pal_nmt_event) + eventHeader->num_outputs *
                                      sizeof(pal_nmt_engine_event);
        eventToStream.payload = calloc(1, eventToStream.payloadSize);

        if (eventToStream.payload == nullptr) {
            PAL_ERR(LOG_TAG, "Failed to allocate memory for stream event!!!");
            goto cleanup;
        }
        PAL_DBG(LOG_TAG, " event header token : %d, num output : %d, payload size : %d",
                eventHeader->token, eventHeader->num_outputs, eventHeader->payload_size);

        eventPayload = (pal_nmt_event *)eventToStream.payload;
        eventPayload->num_events = eventHeader->num_outputs;
        for (int i = 0; i < eventHeader->num_outputs; i++) {
             eventStatus = (ev[i].status == 0 ? true : false);
             if (!eventStatus) {
                 PAL_INFO(LOG_TAG, "Recieved failure event, ignoring this event!!!");
                 goto cleanup;
             }
             eventPayload->status = 0;
             eventPayload->event[i].input_text_size = 0;
             strlcpy(eventPayload->event[i].input_text, "", sizeof(eventPayload->event[i].input_text));
             eventPayload->event[i].is_final = ev[i].is_final;
             eventPayload->direction = sAttr.direction;
             eventPayload->event[i].output_text_size = ev[i].output_text_size< 0 ? 0 : ev[i].output_text_size;
             for (int j = 0; j < ev[i].output_text_size; ++j)
                  eventPayload->event[i].output_text[j] = ev[i].output_text[j];
        }
        sAsr = dynamic_cast<StreamCallTranslation *>(streamHandle);
        eventPayload->input_language_code = (uint32_t)(sAsr->GetNmtConfig().input_language_code);
        eventPayload->output_language_code = (uint32_t)(sAsr->GetNmtConfig().output_language_code);
        sAsr->HandleEventData(eventToStream);
    }
cleanup:
    if (eventToStream.payload) {
        free(eventToStream.payload);
        eventToStream.payload = nullptr;
    }

    if (payload)
        free(payload);

    if (event)
        free(event);
exit:
    eventQ.pop();
}

void CallTranslationNMTEngine::EventProcessingThread(CallTranslationNMTEngine *engine)
{
    PAL_INFO(LOG_TAG, "Enter. start thread loop");
    if (!engine) {
        PAL_ERR(LOG_TAG, "Error:%d Invalid engine", -EINVAL);
        return;
    }
    std::unique_lock<std::mutex> lck(engine->mutexEngine);
    while (!engine->exitThread) {
        while (engine->eventQ.empty()) {
            PAL_DBG(LOG_TAG, "waiting on cond");
            engine->cv.wait(lck);
            PAL_DBG(LOG_TAG, "done waiting on cond");

            if (engine->exitThread) {
                PAL_VERBOSE(LOG_TAG, "Exit thread");
                break;
            }
        }
        //Adding this condition, as destructor can also notify this thread without any event
        if (!engine->eventQ.empty())
            engine->ParseEventAndNotifyStream();
    }

    PAL_DBG(LOG_TAG, "Exit");
}

void CallTranslationNMTEngine::HandleSessionEvent(uint32_t event_id __unused,
                                                   void *data, uint32_t size)
{
    void *eventData = nullptr;

    std::unique_lock<std::mutex> lck(mutexEngine);

    if (engRXState == NMT_ENG_IDLE && engTXState == NMT_ENG_IDLE) {
        PAL_INFO(LOG_TAG, "Engine not active, ignore");
        lck.unlock();
        return;
    }

    eventData = calloc(1, size);
    if (!eventData) {
        PAL_ERR(LOG_TAG, "Error:failed to allocate mem for event_data");
        return;
    }

    memcpy(eventData, data, size);
    eventQ.push(eventData);
    cv.notify_one();
}

void CallTranslationNMTEngine::HandleSessionCallBack(uint64_t hdl, uint32_t eventId,
                                                       void *data, uint32_t eventSize)
{
    CallTranslationNMTEngine *engine = nullptr;

    PAL_INFO(LOG_TAG, "Enter, nmt event detected on SPF, event id = 0x%x", eventId);
    if ((hdl == 0) || !data) {
        PAL_ERR(LOG_TAG, "Error:%d Invalid engine handle or event data", -EINVAL);
        return;
    }

    if (eventId != EVENT_ID_NMT_STATUS)
        return;

    engine = (CallTranslationNMTEngine *)hdl;
    engine->HandleSessionEvent(eventId, data, eventSize);

    PAL_DBG(LOG_TAG, "Exit");
    return;
}

uint32_t CallTranslationNMTEngine::GetNmtMiid()
{
    int32_t status = 0;

    if (sAttr.direction == PAL_AUDIO_INPUT) {
        status = dynamic_cast<SessionAR*>(nmtTxSession)->getMIID(nullptr, TRANSLATION_NMT, &nmtMiid);
    } else if (sAttr.direction == PAL_AUDIO_OUTPUT) {
        status = dynamic_cast<SessionAR*>(nmtRxSession)->getMIID(nullptr, TRANSLATION_NMT, &nmtMiid);
    }
    if (status != 0) {
        PAL_ERR(LOG_TAG, "Failed to get instance id for tag %x, status = %d",
                          TRANSLATION_NMT, status);
        return -EINVAL;
    }
    PAL_DBG(LOG_TAG, "translation nmt miid = 0x%08x", nmtMiid);
    return nmtMiid;
}

int32_t CallTranslationNMTEngine::setParameters(Stream *s, pal_param_id_type_t pid)
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter, param id %d ", pid);

    uint32_t paramId = 0;
    uint8_t *payload = nullptr;
    uint8_t *data = nullptr;
    size_t payloadSize = 0;
    size_t dataSize = 0;
    uint32_t sesParamId = 0;
    uint32_t miid = 0;
    uint32_t id = pid;

    StreamCallTranslation* str = dynamic_cast<StreamCallTranslation *>(s);
    miid = GetNmtMiid();

    switch (id) {
        case PAL_PARAM_ID_NMT_OUTPUT : {
            param_id_nmt_output_config_t *opConfig = str->GetOutputConfig(sAttr.direction);
            if (opConfig == nullptr) {
                PAL_ERR(LOG_TAG, "No output config available, can't start the engine!!!");
                goto exit;
            }
            data = (uint8_t *)opConfig;
            dataSize = sizeof(param_id_nmt_output_config_t);
            sesParamId = pid;
            paramId = PARAM_ID_NMT_OUTPUT_CONFIG;
            break;
        }
        default : {
            PAL_ERR(LOG_TAG, "Unexpected param ID is sent, not implemented yet");
            goto exit;
        }
    }

    builder = new PayloadBuilder();
    if (!builder) {
        PAL_ERR(LOG_TAG, "Error: Builder is null");
        status = -EINVAL;
        goto exit;
    }
    status = builder->payloadConfig(&payload, &payloadSize, data, dataSize,
                                        miid, paramId);
    if (status || !payload) {
        PAL_ERR(LOG_TAG, "Failed to construct NMT payload, status = %d",
            status);
        return -ENOMEM;
    }
    if (sAttr.direction == PAL_AUDIO_INPUT) {
        status = dynamic_cast<SessionAR*>(nmtTxSession)->setParamWithTag(streamHandle, TRANSLATION_NMT,
                                                                sesParamId, payload);
    } else if (sAttr.direction == PAL_AUDIO_OUTPUT) {
        status = dynamic_cast<SessionAR*>(nmtRxSession)->setParamWithTag(streamHandle, TRANSLATION_NMT,
                                                                 sesParamId, payload);
    }
    if (status != 0) {
        PAL_ERR(LOG_TAG, "Failed to set payload for param id %x, status = %d",
                sesParamId, status);
    }
exit:
    if (builder) {
         delete builder;
         builder = nullptr;
    }
    if (data != NULL)  {
        free(data);
        data = nullptr;
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t CallTranslationNMTEngine::StartEngine(Stream *s)
{
    PAL_DBG(LOG_TAG, "Enter");

    int32_t status = 0;
    uint8_t *eventPayload = NULL;
    struct pal_stream_attributes sAttr;
    size_t eventPayloadSize = sizeof(struct event_id_nmt_reg_cfg_t);
    struct event_id_nmt_reg_cfg_t *eventConfig =  NULL;
    status = s->getStreamAttributes(&sAttr);

    std::lock_guard<std::mutex> lck(mutexEngine);

    eventPayload = (uint8_t *)calloc(1, eventPayloadSize);
    if (eventPayload == NULL) {
        PAL_ERR(LOG_TAG, "Error: Failed to allocate memory for event payload");
        status = -ENOMEM;
        goto exit;
    }

    eventConfig = (struct event_id_nmt_reg_cfg_t *)eventPayload;
    eventConfig->event_payload_type = 0;
    eventConfig->event_text_type = (text_type - CALL_TRANSLATION_TEXT);

    if (sAttr.direction == PAL_AUDIO_INPUT) {
        dynamic_cast<SessionAR*>(nmtTxSession)->setEventPayload(
            EVENT_ID_NMT_STATUS, (void *)eventPayload, eventPayloadSize);
        engTXState = NMT_ENG_ACTIVE;
    } else if (sAttr.direction == PAL_AUDIO_OUTPUT) {
        dynamic_cast<SessionAR*>(nmtRxSession)->setEventPayload(
            EVENT_ID_NMT_STATUS, (void *)eventPayload, eventPayloadSize);
        engRXState = NMT_ENG_ACTIVE;
    }
exit:
    if (eventConfig)
        free(eventConfig);

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t CallTranslationNMTEngine::StopEngine(Stream *s)
{
    std::lock_guard<std::mutex> lck(mutexEngine);

    if (sAttr.direction == PAL_AUDIO_OUTPUT) {
        if (engRX) {
            engRXState = NMT_ENG_IDLE;
            engRX = nullptr;
        }
    } else if (sAttr.direction == PAL_AUDIO_INPUT) {
        if (engTX) {
            engTXState = NMT_ENG_IDLE;
            engTX = nullptr;
        }
    }
    return 0;
}
