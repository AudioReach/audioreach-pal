/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef UVVOICECUE_UTILS_H
#define UVVOICECUE_UTILS_H

#include <stdint.h>
#include <stddef.h>

#define VOICE_CUE_FILE_NAME "/data/vendor/audio/voice_cue.bin"

int32_t writeBufferToFile(const char *filePath, const uint8_t *data, size_t size);
int32_t readBufferFromFile(const char *filePath, uint8_t **data, size_t *size);

uint8_t *getVoiceCueDataPtr();
size_t getVoiceCueDataSize();
uint32_t getUvMaskUseCaseValues();

int32_t storeVoiceCueToFile(uint8_t *data, uint32_t size);
int32_t retrieveVoiceCueFromFile();

void cacheUvUseCaseMask(uint32_t usecaseMask);

/* New helper APIs to reduce ResourceManager switch-case code */
int32_t handleUvVoiceCueEnable(void *param_payload, size_t payload_size);
int32_t handleUvVoiceCueData(void *param_payload, size_t payload_size);

#endif // UVVOICECUE_UTILS_H