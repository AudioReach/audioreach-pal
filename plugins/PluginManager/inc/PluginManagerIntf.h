/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _PLUGIN_MANAGER_INTF_H_
#define _PLUGIN_MANAGER_INTF_H_

#include "Stream.h"
#include "PalDefs.h"

#define MAKE_PLUGIN_INTF_VERSION(maj, min) ((((maj) & 0xff) << 8) | ((min) & 0xff))
#define PLUGIN_MANAGER_GET_MAJOR_VERSION(ver) (((ver) & 0xff00) >> 8)
#define PLUGIN_MANAGER_GET_MINOR_VERSION(ver) ((ver) & 0x00ff)

#define PLUGIN_MANAGER_MAJOR_VER 1
#define PLUGIN_MANAGER_MINOR_VER 2
#define PLUGIN_MANAGER_VERSION MAKE_PLUGIN_INTF_VERSION(PLUGIN_MANAGER_MAJOR_VER,PLUGIN_MANAGER_MINOR_VER)

/**
 * 1.Used for reconfig. e.g. devswitch
 * 2.There are frontends and backends in ReconfigPluginPayload
 * because reconfig plugin entries happen mostly in SessionAlsaUtils::
 * connectSessionDevice(...)
 * FE and BE has already been determined and been passed in, making
 * it impossible to know which getters(for rx or tx) to call in plugin.
 */
struct ReconfigPluginPayload {
    std::string config_ctrl;
    struct pal_device dAttr;
    std::vector<int> pcmDevIds;
    std::vector<std::pair<int32_t, std::string>> aifBackEnds;
    void* builder; //PayloadBuilder ref for AudioReach arch module configs
    void* payload;
    Session* session;
};

/**
 * 1.Used for SessionAR::setParameters() and setParamWithTag() in derived sessions.
 * 2.Session ptr needs to be passed in, not simply get from getter by streamhandle
 *  because of particular use cases.
 * e.g. In SVA/VUI, Session ptr is declared ONLY IN StreamTriggerEngineGsl,
 * NOT in StreamSoundTrigger and StreamTriggerEngineCapi.
 */
struct SetParamPluginPayload {
    uint32_t paramId;
    void* builder; //PayloadBuilder ref for AudioReach arch module configs
    void* payload;
    Session* session;
};

/**
 * 1.Used for general purpose. e.g. start() in derived session classes.
 */
struct PluginPayload {
    void* payload;
    void* builder; //PayloadBuilder ref for AudioReach arch module configs
    Session* session;
};

typedef enum {
    PAL_PLUGIN_MANAGER_STREAM,
    PAL_PLUGIN_MANAGER_SESSION,
    PAL_PLUGIN_MANAGER_DEVICE,
    PAL_PLUGIN_MANAGER_CONTROL,
    PAL_PLUGIN_MANAGER_CONFIG,
} pal_plugin_manager_t;

typedef enum {
    PAL_PLUGIN_CONFIG_START, //after pcm_open, before pcm_start. ->PAL_PLUGIN_CONFIG_OPEN
    PAL_PLUGIN_CONFIG_POST_START, //after pcm_start. post-start logic to retain the logic for plugin uses.
    PAL_PLUGIN_CONFIG_STOP, //pcm_stop
    PAL_PLUGIN_PRE_RECONFIG, //devswitch before stream are disconnceted;
    PAL_PLUGIN_RECONFIG, //devswitch before stream are re-connected;
    PAL_PLUGIN_POST_RECONFIG, //devswitch after stream are re-connected;
    PAL_PLUGIN_CONFIG_SETPARAM, //setParamWithTag.
} plugin_config_name_t;

typedef enum {
    PLUGIN_CONTROL_VOLUME,
    PLUGIN_CONTROL_VOLUME_BOOST,
    PLUGIN_CONTROL_HD_VOICE,
    PLUGIN_CONTROL_AUDIO_BUFFER,
    PLUGIN_CONTROL_AUDIO_LATENCY,
    PLUGIN_CONTROL_KV_PARAM,
    PLUGIN_CONTROL_MAX,
} plugin_control_name_t;

/*stream plugin entry function typedef*/
typedef Stream* (*StreamCreate)(const struct pal_stream_attributes *sattr, struct pal_device *dattr,
                const uint32_t no_of_devices,
                const struct modifier_kv *modifiers, const uint32_t no_of_modifiers,
                const std::shared_ptr<ResourceManager> rm);

typedef Stream* (*StreamDummyCreate)(const struct pal_stream_attributes *sattr, struct pal_device *dattr,
                               uint32_t instance_id, const std::shared_ptr<ResourceManager> rm);

/*session plugin entry function typedef*/
typedef Session* (*SessionCreate)(const std::shared_ptr<ResourceManager> rm);

/*device plugin entry function typedef*/
typedef void (*DeviceCreate)(struct pal_device *device,
                                const std::shared_ptr<ResourceManager> rm,
                                std::shared_ptr<Device> *dev);

/* control plugin entry point setter and getter*/
typedef int (*PluginSetControl) (Stream* s, plugin_control_name_t control,
                                void *payload, size_t payload_size);

typedef int (*PluginGetControl) (Stream* s, plugin_control_name_t control,
                                void **payload, size_t *payload_size);

/*config plugin entrypoint*/
typedef int (*PluginConfig) (Stream* stream, plugin_config_name_t config,
                            void *payload, size_t payload_size);
#endif /* _PLUGIN_MANAGER_INTF_H_ */