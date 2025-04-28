/*
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

//#define LOG_NDEBUG 0
#define LOG_TAG "PAL: SignalHandler"

#include <unistd.h>
#ifdef PAL_USE_SYSLOG
#include <syslog.h>
#define ALOGE(fmt, arg...) syslog (LOG_ERR, fmt, ##arg)
#define ALOGI(fmt, arg...) syslog (LOG_INFO, fmt, ##arg)
#define ALOGD(fmt, arg...) syslog (LOG_DEBUG, fmt, ##arg)
#define ALOGV(fmt, arg...) syslog (LOG_NOTICE, fmt, ##arg)
#else
#include <log/log.h>
#endif
#include <chrono>
#include <signal.h>
#include "SignalHandler.h"

#ifdef _ANDROID_
#include <utils/ProcessCallStack.h>
#include <cutils/android_filesystem_config.h>
#endif

std::mutex SignalHandler::sDefaultSigMapLock;
std::unordered_map<int, std::shared_ptr<struct sigaction>> SignalHandler::sDefaultSigMap;
std::function<void(int, pid_t, uid_t)> SignalHandler::sClientCb;
std::mutex SignalHandler::sAsyncRegisterLock;
std::future<void> SignalHandler::sAsyncHandle;
bool SignalHandler::sBuildDebuggable;

// static
void SignalHandler::asyncRegister(int signal) {
    std::lock_guard<std::mutex> lock(sAsyncRegisterLock);
    sigset_t pendingSigMask;
    uint32_t tries = kDefaultSignalPendingTries;
    // Delay registration to let default signal handler complete
    do {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(kDefaultRegistrationDelayMs));
        sigpending(&pendingSigMask);
        --tries;
    } while (tries > 0 && sigismember(&pendingSigMask, signal) == 1);

    // Register custom handler only if signal is not pending
    if (!sigismember(&pendingSigMask, signal)) {
        std::vector<int> signals({ signal });
        getInstance()->registerSignalHandler(signals);
    }
}

// static
void SignalHandler::setClientCallback(std::function<void(int, pid_t, uid_t)> cb) {
    sClientCb = cb;
}

// static
std::shared_ptr<SignalHandler> SignalHandler::getInstance() {
    static std::shared_ptr<SignalHandler> instance(new SignalHandler());
    return instance;
}

// static
#ifdef FEATURE_IPQ_OPENWRT
void SignalHandler::invokeDefaultHandler(std::shared_ptr<struct sigaction> sAct,
            int code, siginfo_t *si, void *sc) {
#else
void SignalHandler::invokeDefaultHandler(std::shared_ptr<struct sigaction> sAct,
            int code, struct siginfo *si, void *sc) {
#endif
    ALOGE("%s: invoke default handler for signal %d si->si_code %d from pid %d"
         , __func__, code, si->si_code, si->si_pid);
    // Remove custom handler so that default handler is invoked
    sigaction(code, sAct.get(), NULL);

    int status = 0;
    if (code == DEBUGGER_SIGNAL) {
        ALOGE("signal %d (%s), code -1 "
              "(SI_QUEUE from pid %d, uid %d)",
              code, sigToName.at(code).c_str(),
              si->si_pid, si->si_uid);
        status = sigqueue(getpid(), code, si->si_value);
#ifdef _ANDROID_
        if(isBuildDebuggable() && si->si_uid == AID_AUDIOSERVER) {
            std::string prefix = "audioserver_" + std::to_string(si->si_pid) + " ";
            android::ProcessCallStack pcs;
            pcs.update();
            pcs.log(LOG_TAG, ANDROID_LOG_FATAL, prefix.c_str());
        }
#endif
    } else {
        status = raise(code);
    }

    if (status < 0) {
        ALOGW("%s: Sending signal %d failed with error %d",
                  __func__, code, errno);
    }

    // Register custom handler back asynchronously
    sAsyncHandle = std::async(std::launch::async, SignalHandler::asyncRegister, code);
}

// static
#ifdef FEATURE_IPQ_OPENWRT
void SignalHandler::customSignalHandler(
            int code, siginfo_t *si, void *sc) {
#else
void SignalHandler::customSignalHandler(
            int code, struct siginfo *si, void *sc) {
#endif
    ALOGV("%s: enter", __func__);
    std::lock_guard<std::mutex> lock(sDefaultSigMapLock);
    if (sClientCb) {
        sClientCb(code, si->si_pid, si->si_uid);
    }
    // Invoke default handler
    auto it = sDefaultSigMap.find(code);
    if (it != sDefaultSigMap.end()) {
        invokeDefaultHandler(it->second, code, si, sc);
    }
}

// static
std::vector<int> SignalHandler::getRegisteredSignals() {
    std::vector<int> registeredSignals;
    std::lock_guard<std::mutex> lock(sDefaultSigMapLock);
    for (const auto& element : sDefaultSigMap) {
        registeredSignals.push_back(element.first);
    }
    return registeredSignals;
}

void SignalHandler::registerSignalHandler(std::vector<int> signalsToRegister) {
    ALOGV("%s: enter", __func__);
    struct sigaction regAction = {};
    regAction.sa_sigaction = customSignalHandler;
    regAction.sa_flags = SA_SIGINFO | SA_NODEFER;
    std::lock_guard<std::mutex> lock(sDefaultSigMapLock);
    for (int signal : signalsToRegister) {
        ALOGV("%s: register signal %d", __func__, signal);
        auto oldSigAction = std::make_shared<struct sigaction>();
        if (sigaction(signal, &regAction, oldSigAction.get()) < 0) {
           ALOGW("%s: Failed to register handler with error code %d for signal %d",
                 __func__, errno, signal);
        }
        sDefaultSigMap.emplace(signal, oldSigAction);
    }
}
