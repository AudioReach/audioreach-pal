/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#include <aidl/vendor/qti/hardware/paleventnotifier/BnPALEventNotifier.h>
#include <aidl/vendor/qti/hardware/paleventnotifier/IPALEventNotifierCallback.h>
#include <aidl/vendor/qti/hardware/paleventnotifier/IPALEventNotifier.h>
#include <aidl/vendor/qti/hardware/paleventnotifier/PalTranslationTextEvent.h>
#include <log/log.h>
#include <cutils/list.h>
#include "PalApi.h"
#include "PalDefs.h"

using PalCallbackConfig = ::aidl::vendor::qti::hardware::paleventnotifier::PalCallbackConfig;
using PalCallbackEvent = ::aidl::vendor::qti::hardware::paleventnotifier::PalCallbackEvent;
using PalCallbackEventType = ::aidl::vendor::qti::hardware::paleventnotifier::PalCallbackEventType;
using PalTranslationTextEvent = ::aidl::vendor::qti::hardware::paleventnotifier::PalTranslationTextEvent;

namespace aidl::vendor::qti::hardware::paleventnotifier {

class PalServerNotify : public BnPALEventNotifier {

  public:
    virtual ~PalServerNotify() {}
    ::ndk::ScopedAStatus ipc_pal_notify_register_callback(const std::shared_ptr<IPALEventNotifierCallback>& cb, int* ret) override;
    ::ndk::ScopedAStatus ipc_pal_notify_register_callback_v2(const std::shared_ptr<IPALEventNotifierCallback>& cb,
        const std::vector<PalDeviceId>& devID, const std::vector<PalStreamType>& streamType, int* ret);
};

class ClientInfo {
public:
    struct listnode list;
    uint32_t pid;
    std::shared_ptr<IPALEventNotifierCallback> pal_clbk;
    bool isnotified;
    std::vector<PalDeviceId> deviceID;
    std::vector<PalStreamType> streamType;
    ::ndk::ScopedAIBinder_DeathRecipient mDeathRecipient;
    int getPid() { return pid; }
    ClientInfo(int pid) {
        mDeathRecipient = ndk::ScopedAIBinder_DeathRecipient(
                AIBinder_DeathRecipient_new(ClientInfo::onDeath));
    }
    static void onDeath(void *cookie);
    void onDeath();
    void linkToDeath(const std::shared_ptr<IPALEventNotifierCallback>& cb, ClientInfo *client_handle);
};

}
