/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef _PLUGIN_MANAGER_INTF_H_
#define _PLUGIN_MANAGER_INTF_H_

#include "Stream.h"
//#include "Session.h"
#include "PalDefs.h"

#define MAKE_PLUGIN_INTF_VERSION(maj, min) ((((maj) & 0xff) << 8) | ((min) & 0xff))
#define PLUGIN_MANAGER_GET_MAJOR_VERSION(ver) (((ver) & 0xff00) >> 8)
#define PLUGIN_MANAGER_GET_MINOR_VERSION(ver) ((ver) & 0x00ff)

#define PLUGIN_MANAGER_MAJOR_VER 1
#define PLUGIN_MANAGER_MINOR_VER 2
#define PLUGIN_MANAGER_VERSION MAKE_PLUGIN_INTF_VERSION(PLUGIN_MANAGER_MAJOR_VER,PLUGIN_MANAGER_MINOR_VER)

typedef enum {
    PAL_PLUGIN_MANAGER_STREAM,
    PAL_PLUGIN_MANAGER_SESSION,
    PAL_PLUGIN_MANAGER_DEVICE,
    PAL_PLUGIN_MANAGER_CONTROL,
    PAL_PLUGIN_MANAGER_CONFIG,
} pal_plugin_manager_t;

typedef enum {
    PAL_PLUGIN_CONFIG,
    PAL_PLUGIN_RECONFIG,
} plugin_config_name_t;

typedef enum{
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

typedef Stream* (*StreamACDBCreate)(const struct pal_stream_attributes *sattr, struct pal_device *dattr,
                               uint32_t instance_id, const std::shared_ptr<ResourceManager> rm);

/*session plugin entry function typedef*/
typedef Session* (*SessionCreate)(const std::shared_ptr<ResourceManager> rm);

/*device plugin entry function typedef*/
typedef void (*DeviceCreate)(struct pal_device *device,
                                const std::shared_ptr<ResourceManager> rm,
                                pal_device_id_t id, bool createDevice,
                                std::shared_ptr<Device> *dev);

/* control plugin entry point setter and getter*/
typedef int (*pluginSetControl) (Stream* s, plugin_control_name_t control,
                                void *payload, size_t payload_size);

typedef int (*pluginGetControl) (Stream* s, plugin_control_name_t control,
                                void **payload, size_t *payload_size);

/*config plugin entrypoint*/
typedef int (*pluginConfig) (Session* s, plugin_config_name_t config,
                             void *payload, size_t payload_size);

#endif /* _PLUGIN_MANAGER_INTF_H_ */