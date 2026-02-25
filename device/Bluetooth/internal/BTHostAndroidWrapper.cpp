/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#define LOG_TAG "BTHostAndroidWrapper"

#include "BTHostAndroidWrapper.h"

#include "PalCommon.h"

BTHostSourceAPITable::BTHostSourceAPITable() {
    const static std::string kLibName = "btaudio_offload_if.so";
    this->mLibHandle = dlopen(kLibName.c_str(), RTLD_NOW);
    if (this->mLibHandle == nullptr) {
        this->mErrString = "unable to load library: btaudio_offload_if.so";
        PAL_ERR(LOG_TAG, "unable to load library: btaudio_offload_if.so");
        CHECK(1 == 0);
    }
    std::ostringstream oss;
    auto load = [&](auto& fn, const char* name) -> bool {
        fn = reinterpret_cast<std::remove_reference_t<decltype(fn)>>(dlsym(this->mLibHandle, name));
        if (!fn) {
            oss << "unable to find symbol:" << name << " ";
            PAL_ERR(LOG_TAG, "unable to find symbol: %s", name);
            return false;
        }
        return true;
    };

    // Load all functions
    load(this->audio_start_stream, "audio_start_stream");
    load(this->audio_stop_stream, "audio_stop_stream");
    load(this->audio_suspend_stream, "audio_suspend_stream");
    load(this->audio_stream_open, "audio_stream_open");
    load(this->audio_stream_close, "audio_stream_close");
    load(this->audio_stream_start, "audio_stream_start");
    load(this->audio_stream_stop, "audio_stream_stop");
    load(this->audio_stream_suspend, "audio_stream_suspend");

    load(this->audio_get_codec_config, "audio_get_codec_config");
    load(this->audio_handoff_triggered, "audio_handoff_triggered");
    load(this->clear_a2dpsuspend_flag, "clear_a2dpsuspend_flag");
    load(this->audio_get_next_codec_config, "audio_get_next_codec_config");
    load(this->audio_check_a2dp_ready, "audio_check_a2dp_ready");
    load(this->audio_get_a2dp_sink_latency, "audio_get_a2dp_sink_latency");
    load(this->audio_sink_get_a2dp_latency, "audio_sink_get_a2dp_latency");
    load(this->wait_for_stack_response, "wait_for_stack_response");
    load(this->audio_is_scrambling_enabled, "audio_is_scrambling_enabled");
    load(this->bt_audio_pre_init, "bt_audio_pre_init");

    load(this->audio_sink_start_stream, "audio_sink_start_stream");
    load(this->audio_sink_stop_stream, "audio_sink_stop_stream");
    load(this->audio_sink_suspend_stream, "audio_sink_suspend_stream");
    load(this->audio_sink_stream_start, "audio_sink_stream_start");
    load(this->audio_sink_stream_stop, "audio_sink_stream_stop");
    load(this->audio_sink_stream_suspend, "audio_sink_stream_suspend");

    load(this->update_metadata, "update_metadata");
    load(this->audio_start_stream_api, "audio_start_stream_api");
    load(this->audio_stop_stream_api, "audio_stop_stream_api");
    load(this->audio_suspend_stream_api, "audio_suspend_stream_api");
    load(this->audio_stream_open_api, "audio_stream_open_api");
    load(this->audio_stream_close_api, "audio_stream_close_api");
    load(this->audio_check_a2dp_ready_api, "audio_check_a2dp_ready_api");
    load(this->audio_get_codec_config_api, "audio_get_codec_config_api");
    load(this->audio_sink_get_a2dp_latency_api, "audio_sink_get_a2dp_latency_api");
    load(this->register_reconfig_cb, "register_reconfig_cb");
    load(this->unregister_reconfig_cb, "unregister_reconfig_cb");
    load(this->audio_stream_get_supported_latency_modes_api,
         "audio_stream_get_supported_latency_modes_api");
    load(this->audio_stream_set_latency_mode_api, "audio_stream_set_latency_mode_api");
    this->mErrString = oss.str();
}

BTHostSourceAPITable::~BTHostSourceAPITable() {
    if (this->mLibHandle != nullptr) {
        dlclose(mLibHandle);
    }
}

// static
BTHostSourceAPITable* BTHostSourceAPITable::getInstance() {
    static BTHostSourceAPITable table;
    return &table;
}