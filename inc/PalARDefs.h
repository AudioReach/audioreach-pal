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
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef PAL_AR_DEFS_H
#define PAL_AR_DEFS_H


 /*
  * Description: used to get list of ar modules info associated to usecase
  * Payload For this custom param string is pal_tag_module_info
 */
#define PAL_CUSTOM_PARAM_AR_TAG_MODULE_INFO "PAL_CUSTOM_PARAM_AR_TAG_MODULE_INFO"

 /*
  * Description: used to set custom configurations to specifed modules
  * Payload For this custom param string is apm_module_param_data_t
 */
#define PAL_CUSTOM_PARAM_AR_TAG_MODULE_CONFIG "PAL_CUSTOM_PARAM_AR_TAG_MODULE_CONFIG"

 /*
  * Description: used to get/set the custom effect param to AR modules
  * Payload For this custom param string is gef_payload_s
 */
#define PAL_CUSTOM_PARAM_AR_UI_EFFECT "PAL_CUSTOM_PARAM_AR_UI_EFFECT"

 /*
  * Description: used to send custom params to AR modules
  * Payload For this custom param string is pal_cshm_msg_payload_t
 */
#define PAL_CUSTOM_PARAM_SEND_MSG_PARAM "PAL_CUSTOM_PARAM_SEND_MSG_PARAM"

 /**
  * Maps the modules instance id to module id for a single module
  */
struct module_info {
    uint32_t module_id; /**< module id */
    uint32_t module_iid; /**< globally unique module instance id */
};

/**
 * Structure mapping the tag_id to module info (mid and miid)
 */
struct pal_tag_module_mapping {
    uint32_t tag_id; /**< tag id of the module */
    uint32_t num_modules; /**< number of modules matching the tag_id */
    struct module_info mod_list[]; /**< module list */
};


/* Payload For param str: PAL_CUSTOM_PARAM_AR_TAG_MODULE_INFO
 * Description   : Used to return tags and module info data to client given a graph key vector
 */
struct pal_tag_module_info {
    /**< number of tags */
    uint32_t num_tags;
    /**< variable payload of type struct pal_tag_module_mapping*/
    uint8_t pal_tag_module_list[];
};


typedef struct pal_key_value_pair_s {
    uint32_t key; /**< key */
    uint32_t value; /**< value */
} pal_key_value_pair_t;

typedef struct pal_key_vector_s {
    size_t num_tkvs;  /**< number of key value pairs */
    pal_key_value_pair_t kvp[];  /**< vector of key value pairs */
} pal_key_vector_t;

typedef enum {
    PARAM_NONTKV,
    PARAM_TKV,
} pal_param_type_t;

typedef struct pal_effect_custom_payload_s {
    uint32_t paramId;
    uint32_t data[];
} pal_effect_custom_payload_t;

typedef struct effect_pal_payload_s {
    pal_param_type_t isTKV;      /* payload type: 0->non-tkv 1->tkv*/
    uint32_t tag;
    uint32_t  payloadSize;
    uint32_t  payload[]; /* TKV uses pal_key_vector_t, while nonTKV uses pal_effect_custom_payload_t */
} effect_pal_payload_t;

/* Payload for param str: PAL_CUSTOM_PARAM_AR_TAG_MODULE_INFO
 * Description   : Used to set/get custom effect parameters to Audioreach Specific UC definition/Modules
 */
typedef struct gef_payload_s {
    pal_key_vector_t *graph;
    bool persist;
    effect_pal_payload_t data;
} gef_payload_t;

typedef enum {
    GEF_PARAM_READ = 0,
    GEF_PARAM_WRITE,
} gef_param_rw_t;

#endif /*PAL_AR_DEFS_H*/