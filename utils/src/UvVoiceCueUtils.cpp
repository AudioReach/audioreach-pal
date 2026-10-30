/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "PAL: UVVoiceCueUtils"

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <mutex>

#include "ResourceManager.h"
#include "UvVoiceCueUtils.h"

namespace {
std::mutex gVoiceCueMutex;
uv_fluence_config_t cachedRmUvConfig = {};
}

uint8_t *getVoiceCueDataPtr()
{
    std::lock_guard<std::mutex> lock(gVoiceCueMutex);
    return cachedRmUvConfig.voice_cue_param;
}

size_t getVoiceCueDataSize()
{
    std::lock_guard<std::mutex> lock(gVoiceCueMutex);
    return cachedRmUvConfig.param_size;
}

uint32_t getUvMaskUseCaseValues()
{
    std::lock_guard<std::mutex> lock(gVoiceCueMutex);
    return cachedRmUvConfig.usecase_mask;
}

void cacheUvUseCaseMask(uint32_t usecaseMask)
{
    std::lock_guard<std::mutex> lock(gVoiceCueMutex);
    cachedRmUvConfig.usecase_mask = usecaseMask;
}

int32_t handleUvVoiceCueEnable(void *param_payload, size_t payload_size)
{
    pal_param_payload *palPayload = nullptr;
    uv_fluence_config_t *incoming = nullptr;

    if (!param_payload) {
        PAL_ERR(LOG_TAG, "param_payload is null");
        return -EINVAL;
    }

    if (payload_size < sizeof(pal_param_payload)) {
        PAL_ERR(LOG_TAG, "payload_size %zu is too small", payload_size);
        return -EINVAL;
    }

    palPayload = reinterpret_cast<pal_param_payload *>(param_payload);

    if (palPayload->payload_size < sizeof(uv_fluence_config_t)) {
        PAL_ERR(LOG_TAG, "Invalid UV fluence payload size %u",
                palPayload->payload_size);
        return -EINVAL;
    }

    incoming = reinterpret_cast<uv_fluence_config_t *>(palPayload->payload);

    PAL_DBG(LOG_TAG, "Enable Cmd -> usecase_mask: 0x%x",
            incoming->usecase_mask);

    {
        std::lock_guard<std::mutex> lock(gVoiceCueMutex);
        cachedRmUvConfig.usecase_mask = incoming->usecase_mask;
    }

    PAL_INFO(LOG_TAG, "Decoded UV config: audio=%d voice=%d voip=%d sva=%d",
             !!(incoming->usecase_mask & UV_FLUENCE_AUDIO_BIT),
             !!(incoming->usecase_mask & UV_FLUENCE_TELEPHONY_BIT),
             !!(incoming->usecase_mask & UV_FLUENCE_VOIP_BIT),
             !!(incoming->usecase_mask & UV_FLUENCE_SVA_BIT));

    return 0;
}

int32_t handleUvVoiceCueData(void *param_payload, size_t payload_size)
{
    pal_param_payload *palPayload = nullptr;
    uint8_t *incomingData = nullptr;
    size_t incomingSize = 0;
    uint8_t *newVoiceCue = nullptr;
    int32_t fileStatus = 0;

    if (!param_payload || payload_size < sizeof(pal_param_payload)) {
        PAL_ERR(LOG_TAG, "Invalid param payload for UV_VOICE_CUE_DATA_BYTE");
        return -EINVAL;
    }

    palPayload = reinterpret_cast<pal_param_payload *>(param_payload);
    incomingData = reinterpret_cast<uint8_t *>(palPayload->payload);
    incomingSize = palPayload->payload_size;

    PAL_DBG(LOG_TAG, "Received Voice Cue Data. Size: %zu bytes", incomingSize);

    if (!incomingData || incomingSize == 0) {
        PAL_ERR(LOG_TAG, "Invalid Voice Cue data received. data: %p size: %zu",
                incomingData, incomingSize);
        return -EINVAL;
    }

    newVoiceCue = static_cast<uint8_t *>(calloc(1, incomingSize));
    if (!newVoiceCue) {
        PAL_ERR(LOG_TAG, "Failed to allocate memory for Voice Cue Data");
        return -ENOMEM;
    }

    memcpy(newVoiceCue, incomingData, incomingSize);

    {
        std::lock_guard<std::mutex> lock(gVoiceCueMutex);

        if (cachedRmUvConfig.voice_cue_param != nullptr) {
            free(cachedRmUvConfig.voice_cue_param);
            cachedRmUvConfig.voice_cue_param = nullptr;
            cachedRmUvConfig.param_size = 0;
        }

        cachedRmUvConfig.voice_cue_param = newVoiceCue;
        cachedRmUvConfig.param_size = incomingSize;
    }

    newVoiceCue = nullptr; // ownership moved to cache

    PAL_DBG(LOG_TAG, "Cached %zu bytes of cue data successfully", incomingSize);

    fileStatus = storeVoiceCueToFile(getVoiceCueDataPtr(),
                                     static_cast<uint32_t>(incomingSize));
    if (fileStatus != 0) {
        PAL_ERR(LOG_TAG, "Failed to persist cue data to file! Status: %d",
                fileStatus);
    } else {
        PAL_DBG(LOG_TAG, "Successfully saved cue data to file.");
    }

    return 0;
}

int32_t storeVoiceCueToFile(uint8_t *data, uint32_t size)
{
    int32_t status = 0;

    PAL_INFO(LOG_TAG, "Enter, size %u", size);

    if (!data || size == 0) {
        PAL_ERR(LOG_TAG, "Invalid data pointer or size is 0");
        return -EINVAL;
    }

    status = writeBufferToFile(VOICE_CUE_FILE_NAME, data, size);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to store voice cue to file %s, status %d",
                VOICE_CUE_FILE_NAME, status);
        return status;
    }

    PAL_INFO(LOG_TAG, "Successfully stored %u bytes to %s",
             size, VOICE_CUE_FILE_NAME);
    return 0;
}

int32_t retrieveVoiceCueFromFile()
{
    uint8_t *tmpData = nullptr;
    size_t tmpSize = 0;
    int32_t status = 0;

    status = readBufferFromFile(VOICE_CUE_FILE_NAME, &tmpData, &tmpSize);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to retrieve voice cue from file %s, status %d",
                VOICE_CUE_FILE_NAME, status);
        return status;
    }

    if (!tmpData || tmpSize == 0) {
        PAL_DBG(LOG_TAG, "No valid voice cue data found in file");
        return 0;
    }

    {
        std::lock_guard<std::mutex> lock(gVoiceCueMutex);

        if (cachedRmUvConfig.voice_cue_param) {
            free(cachedRmUvConfig.voice_cue_param);
            cachedRmUvConfig.voice_cue_param = nullptr;
            cachedRmUvConfig.param_size = 0;
        }

        cachedRmUvConfig.voice_cue_param = tmpData;
        cachedRmUvConfig.param_size = tmpSize;
    }

    PAL_INFO(LOG_TAG, "Successfully restored %zu bytes of cue data", tmpSize);
    return 0;
}

int32_t writeBufferToFile(const char *filePath, const uint8_t *data, size_t size)
{
    int fd = -1;
    size_t totalWritten = 0;

    if (!filePath || !data || size == 0) {
        PAL_ERR(LOG_TAG, "Invalid input. filePath %p, data %p, size %zu",
                filePath, data, size);
        return -EINVAL;
    }

    fd = open(filePath, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        PAL_ERR(LOG_TAG, "Failed to open file %s for write, errno %d (%s)",
                filePath, errno, strerror(errno));
        return -errno;
    }

    while (totalWritten < size) {
        ssize_t written = write(fd, data + totalWritten, size - totalWritten);
        if (written < 0) {
            if (errno == EINTR)
                continue;

            PAL_ERR(LOG_TAG, "Write failed for file %s, errno %d (%s)",
                    filePath, errno, strerror(errno));
            close(fd);
            return -errno;
        }

        if (written == 0) {
            PAL_ERR(LOG_TAG, "Write returned 0 for file %s", filePath);
            close(fd);
            return -EIO;
        }

        totalWritten += static_cast<size_t>(written);
    }

    close(fd);
    PAL_DBG(LOG_TAG, "Successfully wrote %zu bytes to %s",
            totalWritten, filePath);

    return 0;
}

int32_t readBufferFromFile(const char *filePath, uint8_t **data, size_t *size)
{
    int fd = -1;
    struct stat st;
    uint8_t *buffer = nullptr;
    size_t totalRead = 0;

    if (!filePath || !data || !size) {
        PAL_ERR(LOG_TAG, "Invalid input. filePath %p, data %p, size %p",
                filePath, data, size);
        return -EINVAL;
    }

    *data = nullptr;
    *size = 0;

    if (stat(filePath, &st) != 0) {
        PAL_ERR(LOG_TAG, "stat failed for %s, errno %d (%s)",
                filePath, errno, strerror(errno));
        return -errno;
    }

    if (st.st_size <= 0) {
        PAL_ERR(LOG_TAG, "File %s is empty or invalid, size %lld",
                filePath, static_cast<long long>(st.st_size));
        return -EINVAL;
    }

    fd = open(filePath, O_RDONLY);
    if (fd < 0) {
        PAL_ERR(LOG_TAG, "Failed to open file %s for read, errno %d (%s)",
                filePath, errno, strerror(errno));
        return -errno;
    }

    buffer = static_cast<uint8_t *>(calloc(1, static_cast<size_t>(st.st_size)));
    if (!buffer) {
        PAL_ERR(LOG_TAG, "Failed to allocate %lld bytes for file read",
                static_cast<long long>(st.st_size));
        close(fd);
        return -ENOMEM;
    }

    while (totalRead < static_cast<size_t>(st.st_size)) {
        ssize_t bytesRead = read(fd, buffer + totalRead,
                                 static_cast<size_t>(st.st_size) - totalRead);
        if (bytesRead < 0) {
            if (errno == EINTR)
                continue;

            PAL_ERR(LOG_TAG, "Read failed for file %s, errno %d (%s)",
                    filePath, errno, strerror(errno));
            free(buffer);
            close(fd);
            return -errno;
        }

        if (bytesRead == 0) {
            break;
        }

        totalRead += static_cast<size_t>(bytesRead);
    }

    close(fd);

    if (totalRead != static_cast<size_t>(st.st_size)) {
        PAL_ERR(LOG_TAG, "Partial read from %s. expected %zu actual %zu",
                filePath, static_cast<size_t>(st.st_size), totalRead);
        free(buffer);
        return -EIO;
    }

    *data = buffer;
    *size = totalRead;

    PAL_DBG(LOG_TAG, "Successfully read %zu bytes from %s", totalRead, filePath);
    return 0;
}