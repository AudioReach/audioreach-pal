/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "STUtils.h"
#include "PalDefs.h"
#include <vector>
#include "ResourceManager.h"
#include <dlfcn.h>

void* vui_utils_dmgr_lib_handle = NULL;
vui_dmgr_init_t vui_utils_dmgr_init = NULL;
vui_dmgr_deinit_t vui_utils_dmgr_deinit = NULL;

void getMatchingStreams(std::vector<Stream*> &active_streams, std::vector<Stream*> &streams, vui_dmgr_uuid_t &uuid)
{
    int ret = 0;
    pal_param_payload *param_payload = nullptr;
    size_t payload_size = sizeof(pal_param_payload) + sizeof(struct st_uuid);

    for (auto s : active_streams) {
        if (NULL != s) {
            param_payload = (pal_param_payload *)calloc(1, payload_size);
            param_payload->payload_size = sizeof(struct st_uuid);
            ret = s->getParameters(PAL_PARAM_ID_VENDOR_UUID, (void**)&param_payload);
            if(ret){
                PAL_ERR(LOG_TAG, "failed to get UUID");
            } else {
                if (!memcmp(param_payload->payload, &uuid, sizeof(uuid))) {
                    PAL_INFO(LOG_TAG, "vendor uuid matched");
                    streams.push_back(s);
                }
            }
            free(param_payload);
        }
    }
}

int32_t voiceuiDmgrRestartUseCases(vui_dmgr_param_restart_usecases_t *uc_info)
{
    int status = 0;
    std::vector<Stream*> streams;
    std::vector<Stream*> activeStreams;
    pal_stream_type_t type;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    if(!rm){
        PAL_ERR(LOG_TAG, "unable to get resourcemanager instance");
        status = -EINVAL;
        goto exit;
    }

    for (int i = 0; i < uc_info->num_usecases; i++) {
        status = rm->getActiveStreamByType(activeStreams, (pal_stream_type_t)uc_info->usecases[i].stream_type);
        if(status || !activeStreams.size()){
            PAL_ERR(LOG_TAG, "failed to get active streams for stream type %d", uc_info->usecases[i].stream_type);
            break;
        }
        getMatchingStreams(activeStreams, streams, uc_info->usecases[i].vendor_uuid);
    }
    // Reuse SSR mechanism for stream teardown and bring up.
    PAL_INFO(LOG_TAG, "restart %d streams", streams.size());
    for (auto &s : streams) {
        s->getStreamType(&type);
        status = s->ssrDownHandler();
        if (status) {
            PAL_ERR(LOG_TAG, "stream teardown failed %d", type);
        }
        status = s->ssrUpHandler();
        if (status) {
            PAL_ERR(LOG_TAG, "strem bring up failed %d", type);
        }
    }
exit:
    return status;
}

int32_t voiceuiDmgrPalCallback(int32_t param_id, void *payload, size_t payload_size)
{
    int status = 0;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    PAL_DBG(LOG_TAG, "Enter param id: %d", param_id);
    if (!payload) {
        PAL_ERR(LOG_TAG, "Null payload");
        return -EINVAL;
    }
    if (!rm) {
        PAL_ERR(LOG_TAG, "null resource manager");
        return -EINVAL;
    }

    switch (param_id) {
        case VUI_DMGR_PARAM_ID_RESTART_USECASES:
        {
            vui_dmgr_param_restart_usecases_t *uc_info = (vui_dmgr_param_restart_usecases_t *)payload;
            if (payload_size != sizeof(vui_dmgr_param_restart_usecases_t)) {
                PAL_ERR(LOG_TAG, "Incorrect payload size %zu", payload_size);
                status = -EINVAL;
                break;
            }
            voiceuiDmgrRestartUseCases(uc_info);
        }
        break;
        default:
            PAL_ERR(LOG_TAG, "Unknown param id: %d", param_id);
            break;
    }

    PAL_DBG(LOG_TAG, "Exit status: %d", status);
    return status;
}

void voiceuiDmgrManagerInit()
{
    int status = 0;

    vui_utils_dmgr_lib_handle = dlopen(VUI_DMGR_LIB_PATH, RTLD_NOW);

    if (!vui_utils_dmgr_lib_handle) {
        PAL_ERR(LOG_TAG, "dlopen failed for voiceui dmgr %s", dlerror());
        return;
    }

    vui_utils_dmgr_init = (vui_dmgr_init_t)dlsym(vui_utils_dmgr_lib_handle, "vui_dmgr_init");
    if (!vui_utils_dmgr_init) {
        PAL_ERR(LOG_TAG, "dlsym for vui_utils_dmgr_init failed %s", dlerror());
        goto exit;
    }
    vui_utils_dmgr_deinit = (vui_dmgr_deinit_t)dlsym(vui_utils_dmgr_lib_handle, "vui_dmgr_deinit");
    if (!vui_utils_dmgr_deinit) {
        PAL_ERR(LOG_TAG, "dlsym for voiceui dmgr failed %s", dlerror());
        goto exit;
    }
    status = vui_utils_dmgr_init(voiceuiDmgrPalCallback);
    if (status) {
        PAL_DBG(LOG_TAG, "voiceui dmgr failed to initialize, status %d", status);
        goto exit;
    }
    PAL_INFO(LOG_TAG, "voiceui dgmr initialized");
    return;

exit:
    if (vui_utils_dmgr_lib_handle) {
        dlclose(vui_utils_dmgr_lib_handle);
        vui_utils_dmgr_lib_handle = NULL;
    }
    vui_utils_dmgr_init = NULL;
    vui_utils_dmgr_deinit = NULL;
}

void voiceuiDmgrManagerDeInit()
{
    if (vui_utils_dmgr_deinit)
        vui_utils_dmgr_deinit();

    if (vui_utils_dmgr_lib_handle) {
        dlclose(vui_utils_dmgr_lib_handle);
        vui_utils_dmgr_lib_handle = NULL;
    }
    vui_utils_dmgr_init = NULL;
    vui_utils_dmgr_deinit = NULL;
}