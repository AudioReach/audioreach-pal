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

/** \file pal_api.h
 *  \brief Function prototypes for the PAL(Platform Audio
 *   Layer).
 *
 *  This contains the prototypes for the PAL(Platform Audio
 *  layer) and  any macros, constants, or global variables
 *  needed.
 */

#ifndef PAL_AR_H
#define PAL_AR_H

#include "PalDefs.h"

/** APIS*/

/**
  * Gets all the tags and associated module iid and module_id
  * mapping associated with a Pal session handle
  * \param[in] stream_handle - Valid stream handle obtained
  *       from pal_stream_open.
  * \param[in/out] size - size of the memory passed by the client. If it is not
                          enough to copy the tag_module_info a error is returned with
                          the size set to the expected size of the memory to be passed.
  * \param[out] payload - It is in the form of struct pal_tag_module_info
  *
  * \return - 0 on success, error code otherwise.
  */
int32_t pal_stream_get_tags_with_module_info(pal_stream_handle_t *stream_handle,
                                   size_t *size ,uint8_t *payload);

/**
  * \brief Set and get pal parameters for generic effect framework
  *
  * \param[in] param_id - param id whose parameters are to be
  *       set.
  * \param[in/out] param_payload - param data applicable to the
  *       param_id
  * \param[in] payload_size - size of payload
  *
  * \param[in] pal_stream_type - type of stream to apply the GEF param
  *
  * \param[in] pal_device_id - device id to apply the effect
  *
  * \param[i] pal_stream_type - stream type to apply the effect
  *
  * \param[in] dir - param read or write
  *
  * \return 0 on success, error code otherwise
  */
int32_t pal_gef_rw_param(uint32_t param_id, void *param_payload,
                      size_t payload_size, pal_device_id_t pal_device_id,
                      pal_stream_type_t pal_stream_type, unsigned int dir);

/**
  * \brief Set and get pal parameters for generic effect framework with ACDB
  *
  * \param[in] param_id - param id whose parameters are to be
  *       set.
  * \param[in/out] param_payload - param data applicable to the
  *       param_id
  * \param[in] payload_size - size of payload
  *
  * \param[in] pal_stream_type - type of stream to apply the GEF param
  *
  * \param[in] pal_device_id - device id to apply the effect
  *
  * \param[i] pal_stream_type - stream type to apply the effect
  *
  * \param[in] sample_rate - sample_rate value for CKV. 0 for default CKV
  *
  * \param[in] instance_id - instance id
  *
  * \param[in] dir - param read or write
  *
  * \param[in] is_play - stream direction. true for playback. false for recording.
  *
  * \return 0 on success, error code otherwise
  */

int32_t pal_gef_rw_param_acdb(uint32_t param_id, void *param_payload,
                      size_t payload_size, pal_device_id_t pal_device_id,
                      pal_stream_type_t pal_stream_type, uint32_t sample_rate,
                      uint32_t instance_id, uint32_t dir, bool is_play);

#endif /*PAL_AR_H*/
