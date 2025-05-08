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

#ifndef NMTENGINE_H
#define NMTENGINE_H

#include <map>

#include "StreamCallTranslation.h"
#include "PayloadBuilder.h"

#define OUT_BUF_SIZE_DEFAULT 3072 /* In bytes. Around 30sec of text */

typedef enum {
    NMT_ENG_IDLE,
    NMT_ENG_ACTIVE,
} nmt_eng_state_t;

class CallTranslationNMTEngine
{
public:
    CallTranslationNMTEngine(Stream *s);
    ~CallTranslationNMTEngine();
    static std::shared_ptr<CallTranslationNMTEngine> GetInstance(Stream *s);
    int32_t setParameters(Stream *s, pal_param_id_type_t pid);
    int32_t StartEngine(Stream *s);
    int32_t StopEngine(Stream *s);
    uint32_t GetNumOutput() { return numOutput; }
    uint32_t GetOutputToken() { return outputToken; }
    uint32_t GetPayloadSize() { return payloadSize; }
    uint32_t GetNmtMiid();
    void releaseEngine();
private:
    static void EventProcessingThread(CallTranslationNMTEngine *engine);
    static void HandleSessionCallBack(uint64_t hdl, uint32_t event_id, void *data,
                                      uint32_t eventSize);
    void ParseEventAndNotifyStream();
    void HandleSessionEvent(uint32_t eventId __unused, void *data, uint32_t size);
    bool exitThread;
    uint32_t numOutput;
    uint32_t payloadSize;
    uint32_t outputToken;
    uint32_t nmtMiid;

    std::queue<void *> eventQ;
    static std::shared_ptr<CallTranslationNMTEngine> engTX;
    static std::shared_ptr<CallTranslationNMTEngine> engRX;
     struct pal_stream_attributes sAttr;
    nmt_eng_state_t engTXState;
    nmt_eng_state_t engRXState;
    eventType text_type;
    std::thread eventTxThreadHandler;
    std::thread eventRxThreadHandler;
    std::mutex mutexEngine;
    std::condition_variable cv;
    Session *nmtTxSession;
    Session *nmtRxSession;
    Stream *streamHandle;
    PayloadBuilder *builder;
};
#endif  // NMTENGINE_H
