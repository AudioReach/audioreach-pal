/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef ST_UTILS_H
#define ST_UTILS_H

#include <vui_dmgr_audio_intf.h>
#include "Stream.h"
#include <stdio.h>
#include "SoundTriggerPlatformInfo.h"


void voiceuiDmgrManagerInit();
void voiceuiDmgrManagerDeInit();
int32_t voiceuiDmgrPalCallback(int32_t param_id, void *payload, size_t payload_size);
int32_t voiceuiDmgrRestartUseCases(vui_dmgr_param_restart_usecases_t *uc_info);

#endif
