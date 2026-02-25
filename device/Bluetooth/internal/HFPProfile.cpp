/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#define LOG_TAG "HFPProfile"

#include "HFPProfile.h"

#include "BTHostAndroidWrapper.h"
#include "PalCommon.h"

namespace device::bt {

static BTHostSourceAPITable* sBTHostAPI = nullptr;

// static
std::string HFPProfile::toString(const State state) noexcept {
    switch (state) {
        case State::CLOSED:
            return "CLOSED";
        case State::OPENED:
            return "OPENED";
        case State::STARTED:
            return "STARTED";
        case State::STOPPED:
            return "STOPPED";
        default:
            return "UNKNOWN";
    }
}

// static
std::string HFPProfile::toString(const Codec::Type type) noexcept {
    switch (type) {
        case Codec::Type::INVALID:
            return "INVALID";
        case Codec::Type::LC3:
            return "LC3";
        case Codec::Type::MSBC:
            return "MSBC";
        case Codec::Type::CVSD:
            return "CVSD";
        case Codec::Type::APTX_SPEECH:
            return "APTX_SPEECH";
        default:
            return "UNKNOWN";
    }
}

// static
std::string HFPProfile::toString(const Codec& codec) noexcept {
    return std::string("Codec:") + toString(codec.mType) +
           ", isNRECEnabled:" + (codec.mIsNRECEnabled ? "true" : "false");
}

// static
HFPProfile* HFPProfile::getInstance() noexcept {
    static HFPProfile instance;
    return &instance;
}

HFPProfile::HFPProfile() {
    sBTHostAPI = BTHostSourceAPITable::getInstance();
    CHECK(sBTHostAPI != nullptr);
}

bool HFPProfile::Codec::operator==(const Codec& other) const noexcept {
    return (this->mType == other.mType && this->mIsNRECEnabled == other.mIsNRECEnabled);
}

Status HFPProfile::open() noexcept {
    std::lock_guard lock(mLock);
    PAL_INFO(LOG_TAG, "current state: %s, start count: %d", toString(mState).c_str(),
             mStreamCounter);
    if (mState == State::OPENED) {
        return Status::OK;
    }
    // CHECK(mState == State::CLOSED);
    if (mState == State::CLOSED) {
        auto ret = TIME_LOG(sBTHostAPI->audio_stream_open_api(HFP_HARDWARE_OFFLOAD_DATAPATH));
        // CHECK(ret == 0);
        mState = State::OPENED;
        mStreamCounter = 0;
        return Status::OK;
    }
    return Status::FAILED;
}

Status HFPProfile::start() noexcept {
    std::lock_guard lock(mLock);
    PAL_INFO(LOG_TAG, "current state: %s, start count: %d", toString(mState).c_str(),
             mStreamCounter);
    // CHECK((mState == State::OPENED && mStreamCounter == 0) ||
    //      (mState == State::STARTED && mStreamCounter >= 1));
    if (mState == State::OPENED || mState == State::STOPPED) {
        auto ret = TIME_LOG(sBTHostAPI->audio_start_stream_api(HFP_HARDWARE_OFFLOAD_DATAPATH));
        // CHECK(ret == 0);
        mStreamCounter = 1;
        mState = State::STARTED;
    } else if (mState == State::STARTED && mStreamCounter >= 1) {
        mStreamCounter++;
    } else {
        return Status::FAILED;
    }
    return Status::OK;
}

bool HFPProfile::isActive() noexcept {
    std::lock_guard lock(mLock);
    PAL_INFO(LOG_TAG, "current state: %s, start count: %d", toString(mState).c_str(),
             mStreamCounter);
    return (mState == State::STARTED);
}

Status HFPProfile::stop() noexcept {
    std::lock_guard lock(mLock);
    PAL_INFO(LOG_TAG, "current state: %s, start count: %d", toString(mState).c_str(),
             mStreamCounter);
    // CHECK((mState == State::STARTED && mStreamCounter == 1) ||
    //       (mState == State::STARTED && mStreamCounter > 1));
    if (mState == State::STARTED && mStreamCounter == 1) {
        auto ret = sBTHostAPI->audio_stop_stream_api(HFP_HARDWARE_OFFLOAD_DATAPATH);
        // CHECK(ret == 0);
        mStreamCounter = 0;
        mCodec = std::nullopt;
        mState = State::STOPPED;
    } else if (mState == State::STARTED && mStreamCounter > 1) {
        mStreamCounter--;
    } else {
        return Status::FAILED;
    }
    return Status::OK;
}

Status HFPProfile::close() noexcept {
    std::lock_guard lock(mLock);
    PAL_INFO(LOG_TAG, "current state: %s, start count: %d", toString(mState).c_str(),
             mStreamCounter);
    // CHECK(mState != State::STARTED)
    if (mState == State::CLOSED) {
        return Status::OK;
    } else if (mState == State::STOPPED || mState == State::OPENED) {
        auto ret = sBTHostAPI->audio_stream_close_api(HFP_HARDWARE_OFFLOAD_DATAPATH);
        mState = State::CLOSED;
        mStreamCounter = 0;
        return Status::OK;
    }
    return Status::FAILED;
}

std::optional<HFPProfile::Codec> HFPProfile::getCodec() noexcept {
    std::lock_guard lock(mLock);
    PAL_INFO(LOG_TAG, "current state: %s, start count: %d", toString(mState).c_str(),
             mStreamCounter);
    // CHECK(mState == State::STARTED);

    if (mState != State::STARTED) {
        return std::nullopt;
    }

    if (mCodec) {
        return mCodec;
    }

    uint8_t multi_cast = 0, num_dev = 1;
    audio_format_t codecFormat;
    void* codecPtr = sBTHostAPI->audio_get_codec_config_api(HFP_HARDWARE_OFFLOAD_DATAPATH,
                                                            &multi_cast, &num_dev, &codecFormat);
    // CHECK(codecPtr != nullptr);
    if (codecPtr == nullptr) {
        PAL_ERR(LOG_TAG, "No HFP config received from BTHost!!");
        // CHECK(1 == 0);
        return std::nullopt;
    }
    auto hfpConfig = reinterpret_cast<audio_hfp_config_t*>(codecPtr);
    Codec codec;
    if (hfpConfig->codecId == CODEC_ID_LC3) {
        codec.mType = Codec::Type::LC3;
    } else if (hfpConfig->codecId == CODEC_ID_MSBC) {
        codec.mType = Codec::Type::MSBC;
    } else if (hfpConfig->codecId == CODEC_ID_CVSD) {
        codec.mType = Codec::Type::CVSD;
    } else if (hfpConfig->codecId == AUDIO_CODEC_TYPE_APTX_SWB) {
        codec.mType = Codec::Type::APTX_SPEECH;
    } else {
        PAL_ERR(LOG_TAG, " invalid HFP config from BTHost!!");
        CHECK_MSG(2 == 0, "invalid HFP config from BTHost!!");
    }
    codec.mIsNRECEnabled = hfpConfig->nrec;

    /**
     * Ideally, this is owned by BTOffload only. Hence, It is responsiblity of BTOffload.
     * We do not free this and just set to nullptr.
     * Hence,
     * free(codecPtr); // is prohibited
     */
    codecPtr = nullptr;

    PAL_INFO(LOG_TAG, " HFP configuration: %s", toString(codec).c_str());
    mCodec = codec;
    return mCodec;
}

}  // namespace device::bt
