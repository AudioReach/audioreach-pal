/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
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
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "PAL: API_AR"

#include "Stream.h"
#include "ResourceManager.h"
#include "PalAR.h"
#include "mem_logger.h"
#include "MemLogBuilder.h"

int32_t pal_stream_get_tags_with_module_info(pal_stream_handle_t *stream_handle,
                           size_t *size, uint8_t *payload)
{
    int status = 0;
    Stream *s = NULL;
    std::shared_ptr<ResourceManager> rm = NULL;

    if (!stream_handle) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid stream handle status %d", status);
        return status;
    }

    rm = ResourceManager::getInstance();
    if (!rm) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid resource manager");
        return status;
    }

    PAL_DBG(LOG_TAG, "Enter. Stream handle :%pK", stream_handle);
    kpiEnqueue(__func__, true);

    s =  reinterpret_cast<Stream *>(stream_handle);
    status = s->getTagsWithModuleInfo(size, payload);

    PAL_DBG(LOG_TAG, "Exit. Stream handle: %pK, status %d", stream_handle, status);
    kpiEnqueue(__func__, false);
    return status;
}

int32_t pal_gef_rw_param(uint32_t param_id, void *param_payload,
                      size_t payload_size, pal_device_id_t pal_device_id,
                      pal_stream_type_t pal_stream_type, unsigned int dir)
{
    int status = 0;
    std::shared_ptr<ResourceManager> rm = NULL;

    rm = ResourceManager::getInstance();
    if (!rm) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid resource manager");
        return status;
    }

    PAL_DBG(LOG_TAG, "Enter.");

    if (rm) {
        kpiEnqueue(__func__, true);
        if (GEF_PARAM_WRITE == dir) {
            status = rm->setParameter(param_id, param_payload, payload_size,
                                        pal_device_id, pal_stream_type);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Failed to set global parameter %u, status %d",
                        param_id, status);
            }
        } else {
            status = rm->getParameter(param_id, param_payload, payload_size,
                                        pal_device_id, pal_stream_type);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Failed to set global parameter %u, status %d",
                        param_id, status);
            }
        }
        kpiEnqueue(__func__, false);
    } else {
        PAL_ERR(LOG_TAG, "Pal has not been initialized yet");
        status = -EINVAL;
    }
    PAL_DBG(LOG_TAG, "Exit:");
    return status;
}

int32_t pal_gef_rw_param_acdb(uint32_t param_id __unused, void *param_payload,
                      size_t payload_size __unused, pal_device_id_t pal_device_id,
                      pal_stream_type_t pal_stream_type, uint32_t sample_rate,
                      uint32_t instance_id, uint32_t dir, bool is_play )
{
    int status = 0;
    std::shared_ptr<ResourceManager> rm = NULL;
    rm = ResourceManager::getInstance();

    PAL_DBG(LOG_TAG, "Enter.");
    if (rm) {
        kpiEnqueue(__func__, true);
        status = rm->rwParameterACDB(param_id, param_payload, payload_size,
                                        pal_device_id, pal_stream_type,
                                        sample_rate, instance_id, dir, is_play);
        if (0 != status) {
            PAL_ERR(LOG_TAG, "Failed to rw global parameter %u, status %d",
                        param_id, status);
        }
        kpiEnqueue(__func__, false);
    } else {
        PAL_ERR(LOG_TAG, "Pal has not been initialized yet");
        status = -EINVAL;
    }
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}