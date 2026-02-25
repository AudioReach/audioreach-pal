/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <dlfcn.h>
#include <system/audio.h>

#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <type_traits>

/**
 * This BTHostAndroidWrapper holds and maintains copies the definitions of BTHost defined structs
 * and enums which are necessary to communicate with BTHost for Audio.
 *
 * This is necessary as we load the BTHost library dynamically from Audio.
 * The comments in this represent interface agreement between BTHost and Audio.
 *
 */

typedef enum {
    UNKNOWN,
    /** A2DP legacy that AVDTP media is encoded by Bluetooth Stack */
    A2DP_SOFTWARE_ENCODING_DATAPATH,
    /** The encoding of AVDTP media is done by HW and there is control only */
    A2DP_HARDWARE_OFFLOAD_ENCODING_DATAPATH,
    /** Used when encoded by Bluetooth Stack and streaming to Hearing Aid */
    HEARING_AID_SOFTWARE_ENCODING_DATAPATH,
    /** Used when encoded by Bluetooth Stack and streaming to LE Audio device */
    LE_AUDIO_SOFTWARE_ENCODING_DATAPATH,
    /** Used when decoded by Bluetooth Stack and streaming to audio framework */
    LE_AUDIO_SOFTWARE_DECODED_DATAPATH,
    /** Encoding is done by HW an there is control only */
    LE_AUDIO_HARDWARE_OFFLOAD_ENCODING_DATAPATH,
    /** Decoding is done by HW an there is control only */
    LE_AUDIO_HARDWARE_OFFLOAD_DECODING_DATAPATH,
    /** SW Encoding for LE Audio Broadcast */
    LE_AUDIO_BROADCAST_SOFTWARE_ENCODING_DATAPATH,
    /** HW Encoding for LE Audio Broadcast */
    LE_AUDIO_BROADCAST_HARDWARE_OFFLOAD_ENCODING_DATAPATH,
    /*A2DP legacy that AVDTP media is decoded by Bluetooth Stack */
    A2DP_SOFTWARE_DECODING_DATAPATH,
    /* The decoding of AVDTP media is done by HW and there is control only */
    A2DP_HARDWARE_OFFLOAD_DECODING_DATAPATH,
    /** Used when audio is encoded by Bluetooth Stack and is streaming to HFP device **/
    HFP_SOFTWARE_ENCODING_DATAPATH,
    /** Used when audio is decoded by Bluetooth Stack and is streaming to HFP device **/
    HFP_SOFTWARE_DECODING_DATAPATH,
    /**  Used when encoded and decoded by hardware offload and is streamed to HFP device.
     *   This is a control path only.
     **/
    HFP_HARDWARE_OFFLOAD_DATAPATH,
    MAX_SESSION_TYPE
} tSESSION_TYPE;

/*** start ** HFP Profile definitions */
typedef struct {
    uint32_t codecId;
    uint16_t connection_handle;
    bool nrec;
    bool controller_codec;
} audio_hfp_config_t;

#define CODEC_ID_CVSD 0x00UL
#define CODEC_ID_MSBC 0x02UL
#define CODEC_ID_LC3 0x06UL
#define AUDIO_CODEC_TYPE_APTX_SWB 0x33000000UL
#define HFP_APTX_VOICE_SWB_VENDOR_ID 0x0000004F
#define HFP_APTX_VOICE_SWB_CODEC_ID_BLUETOOTH 0x0003

/*** end ** HFP Profile definitions */

/**
 * This class warps the communication API with BTHost.
 * Todo: Refactor every other usage of BTHost APIs such that this is the single source of BTHost
 * object to communicate.
 */
class BTHostSourceAPITable final {
  private:
    /* start of Function pointers */
    using IntFVoidFn = int (*)();
    using AudioGetCodecConfigFn = void* (*)(uint8_t*, uint8_t*, audio_format_t*);
    using VoidFVoidFn = void (*)();
    using AudioGetNextCodecConfigFn = void* (*)(uint8_t, audio_format_t*);
    using Uint16FVoidFn = uint16_t (*)();
    using IntFUint8Fn = int (*)(uint8_t);
    using BoolFVoidFn = bool (*)();
    using UpdateMetadataFn = void (*)(tSESSION_TYPE, void*);
    using AudioStartStreamFn = int (*)(tSESSION_TYPE);
    using AudioStopStreamFn = int (*)(tSESSION_TYPE);
    using AudioSuspendStreamFn = int (*)(tSESSION_TYPE);
    using AudioStreamOpenFn = int (*)(tSESSION_TYPE);
    using AudioStreamCloseFn = int (*)(tSESSION_TYPE);
    using AudioCheckA2dpReadyFn = int (*)(tSESSION_TYPE);
    using AudioGetCodecConfigApiFn = void* (*)(const tSESSION_TYPE, uint8_t*, uint8_t*,
                                               audio_format_t*);
    using AudioSinkGetA2dpLatencyApiFn = uint16_t (*)(tSESSION_TYPE);

    using RegisterReconfigCbFn = void (*)(int (*)(tSESSION_TYPE, int));
    using UnregisterReconfigCbFn = void (*)(int (*)(tSESSION_TYPE, int));

    using AudioGetSupportedLatencyModesFn = int (*)(tSESSION_TYPE, size_t*, size_t, uint32_t*);
    using AudioSetLatencyModeFn = int (*)(tSESSION_TYPE, uint32_t);
    /* end of function pointers */

    BTHostSourceAPITable(const BTHostSourceAPITable&) = delete;
    BTHostSourceAPITable& operator=(const BTHostSourceAPITable&) = delete;
    BTHostSourceAPITable(BTHostSourceAPITable&&) = delete;
    BTHostSourceAPITable& operator=(BTHostSourceAPITable&&) = delete;
    BTHostSourceAPITable();
    ~BTHostSourceAPITable();

  public:
    static BTHostSourceAPITable* getInstance();

    // Basic Audio Functions
    IntFVoidFn audio_start_stream;
    IntFVoidFn audio_stop_stream;
    IntFVoidFn audio_suspend_stream;
    IntFVoidFn audio_stream_open;
    IntFVoidFn audio_stream_close;
    IntFVoidFn audio_stream_start;
    IntFVoidFn audio_stream_stop;
    IntFVoidFn audio_stream_suspend;

    AudioGetCodecConfigFn audio_get_codec_config;
    VoidFVoidFn audio_handoff_triggered;
    VoidFVoidFn clear_a2dpsuspend_flag;
    AudioGetNextCodecConfigFn audio_get_next_codec_config;
    IntFVoidFn audio_check_a2dp_ready;
    Uint16FVoidFn audio_get_a2dp_sink_latency;
    Uint16FVoidFn audio_sink_get_a2dp_latency;
    IntFUint8Fn wait_for_stack_response;
    BoolFVoidFn audio_is_scrambling_enabled;
    VoidFVoidFn bt_audio_pre_init;

    // Sink Variants
    IntFVoidFn audio_sink_start_stream;
    IntFVoidFn audio_sink_stop_stream;
    IntFVoidFn audio_sink_suspend_stream;
    IntFVoidFn audio_sink_stream_start;
    IntFVoidFn audio_sink_stream_stop;
    IntFVoidFn audio_sink_stream_suspend;

    // HIDL 2.2 APIs
    UpdateMetadataFn update_metadata;
    AudioStartStreamFn audio_start_stream_api;
    AudioStopStreamFn audio_stop_stream_api;
    AudioSuspendStreamFn audio_suspend_stream_api;
    AudioStreamOpenFn audio_stream_open_api;
    AudioStreamCloseFn audio_stream_close_api;
    AudioCheckA2dpReadyFn audio_check_a2dp_ready_api;
    AudioGetCodecConfigApiFn audio_get_codec_config_api;
    AudioSinkGetA2dpLatencyApiFn audio_sink_get_a2dp_latency_api;
    RegisterReconfigCbFn register_reconfig_cb;
    UnregisterReconfigCbFn unregister_reconfig_cb;
    AudioGetSupportedLatencyModesFn audio_stream_get_supported_latency_modes_api;
    AudioSetLatencyModeFn audio_stream_set_latency_mode_api;

  private:
    std::string mErrString;
    void* mLibHandle;
};
