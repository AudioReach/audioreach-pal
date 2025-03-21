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
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2022-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define LOG_TAG "PAL: Device"

#include "Device.h"
#include <tinyalsa/asoundlib.h>
#include "ResourceManager.h"
#include "Device.h"
#include <dlfcn.h>
#include <agm/agm_api.h>
#include <sound/asound.h>
#include "PalAudioRoute.h"
#ifdef FEATURE_IPQ_OPENWRT
#include <sstream>
#endif
#ifdef LINUX_ENABLED
#include <sstream>
#endif

#define MAX_CHANNEL_SUPPORTED 2
#define DEFAULT_OUTPUT_SAMPLING_RATE 48000
#define DEFAULT_OUTPUT_CHANNEL 2

typedef void (*write_qmp_mode)(const char *hdr_custom_key);
std::shared_ptr<PluginManager> Device::pm = nullptr;

std::shared_ptr<Device> Device::getInstance(struct pal_device *device,
                                            std::shared_ptr<ResourceManager> Rm)
{
    uint32_t status;
    std::string deviceName;
    std::shared_ptr<Device> devPtr = nullptr;
    void* plugin = nullptr;
    DeviceCreate deviceCreate = NULL;

    if (!device || !Rm) {
        PAL_ERR(LOG_TAG, "Invalid input parameters");
        return NULL;
    }
    if (device->id == PAL_DEVICE_NONE) {
        PAL_DBG(LOG_TAG," Device None");
        return nullptr;
    }

    pm = PluginManager::getInstance();
    if (!pm) {
        PAL_ERR(LOG_TAG, "Unable to get plugin manager instance");
        return NULL;
    }

    PAL_VERBOSE(LOG_TAG, "Enter device id %d", device->id);

    deviceName = (std::string) deviceNameLUT.at(device->id);
    try {
        status = pm->openPlugin(PAL_PLUGIN_MANAGER_DEVICE, deviceName, plugin);
        if (plugin && !status) {
            deviceCreate = reinterpret_cast<DeviceCreate>(plugin);
            deviceCreate(device, Rm, device->id, true, &devPtr);
            if (devPtr == nullptr) {
                PAL_ERR(LOG_TAG, "Device create failed for type %s",
                    deviceNameLUT.at(device->id).c_str());
            }
            return devPtr;
        }
        else {
            PAL_ERR(LOG_TAG, "unable to get plugin for device type %s",
                    deviceName.c_str());
        }
    }
    catch (const std::exception& e) {
        PAL_ERR(LOG_TAG, "Device create failed for type %s",
            deviceNameLUT.at(device->id).c_str());
    }

    return nullptr;
}

std::shared_ptr<Device> Device::getObject(pal_device_id_t dev_id)
{
    uint32_t status;
    std::string deviceName;
    std::shared_ptr<Device> devPtr = nullptr;
    void* plugin = nullptr;
    DeviceCreate deviceCreate = NULL;

    //There is no registered plugin for device type none, so return null
    if (dev_id == PAL_DEVICE_NONE) {
        return nullptr;
    }

    pm = PluginManager::getInstance();
    if (!pm) {
        PAL_ERR(LOG_TAG, "Unable to get plugin manager instance");
        return NULL;
    }

    PAL_VERBOSE(LOG_TAG, "Enter device id %d", dev_id);

    deviceName = (std::string) deviceNameLUT.at(dev_id);
    try {
        status = pm->openPlugin(PAL_PLUGIN_MANAGER_DEVICE, deviceName,
                                plugin);
        if (plugin) {
            deviceCreate = reinterpret_cast<DeviceCreate>(plugin);
            deviceCreate(nullptr, nullptr, dev_id, false, &devPtr);
            return devPtr;
        }
        else {
            PAL_ERR(LOG_TAG, "unable to get plugin for device type %s",
                    deviceName.c_str());
        }
    }
    catch (const std::exception& e) {
        PAL_ERR(LOG_TAG, "exception: Device getObject failed for type %s",
            deviceNameLUT.at(dev_id).c_str());
    }

    return nullptr;
}

int32_t Device::initHdrRoutine(const char *hdr_custom_key)
{
    void *handle = NULL;
    handle = dlopen("vendor/lib64/libqmp.so", RTLD_NOW);
    if (!handle) {
        PAL_ERR(LOG_TAG, "Failed to open libqmp.so");
    }

   write_qmp_mode write = reinterpret_cast<write_qmp_mode>(dlsym(handle, "write_qmp_mode"));
   if (!write) {
       dlclose(handle);
       return -1;
   }
   write(hdr_custom_key);
   dlclose(handle);
   return 0;
}

Device::Device(struct pal_device *device, std::shared_ptr<ResourceManager> Rm)
{
    rm = Rm;
    rm->getHwAudioMixer(&hwMixerHandle);
    rm->getVirtualAudioMixer(&virtualMixerHandle);
    memset(&deviceAttr, 0, sizeof(struct pal_device));
    ar_mem_cpy(&deviceAttr, sizeof(struct pal_device), device,
                     sizeof(struct pal_device));

    mPALDeviceName.clear();
    customPayload = NULL;
    customPayloadSize = 0;
    strlcpy(mSndDeviceName, "", DEVICE_NAME_MAX_SIZE);
    mCurrentPriority = MIN_USECASE_PRIORITY;
    PAL_DBG(LOG_TAG,"device instance for id %d created", device->id);

}

Device::Device()
{
    strlcpy(mSndDeviceName, "", DEVICE_NAME_MAX_SIZE);
    mCurrentPriority = MIN_USECASE_PRIORITY;
    mPALDeviceName.clear();
}

Device::~Device()
{
    if (customPayload)
        free(customPayload);

    customPayload = NULL;
    customPayloadSize = 0;
    mCurrentPriority = MIN_USECASE_PRIORITY;
    PAL_DBG(LOG_TAG,"device instance for id %d destroyed", deviceAttr.id);
}

int Device::getDeviceAttributes(struct pal_device *dattr, Stream* streamHandle)
{
    struct pal_device *strDevAttr;

    if (!dattr) {
        PAL_ERR(LOG_TAG, "Invalid device attributes");
        return  -EINVAL;
    }

    ar_mem_cpy(dattr, sizeof(struct pal_device),
            &deviceAttr, sizeof(struct pal_device));

    /* overwrite custom key if stream is specified */
    mDeviceMutex.lock();
    if (streamHandle != NULL) {
        if (mStreamDevAttr.empty()) {
            PAL_DBG(LOG_TAG, "no device attr for associated streams for dev %d", getSndDeviceId());
            mDeviceMutex.unlock();
            return 0;
        }
        for (auto it = mStreamDevAttr.begin(); it != mStreamDevAttr.end(); ++it) {
            Stream* curStream = (*it).second.first;
            if (curStream == streamHandle) {
                PAL_DBG(LOG_TAG,"found entry for stream: %pK", streamHandle);
                strDevAttr = (*it).second.second;
                strlcpy(dattr->custom_config.custom_key, strDevAttr->custom_config.custom_key,
                        PAL_MAX_CUSTOM_KEY_SIZE);
                break;
            }
        }
    }
    mDeviceMutex.unlock();

    return 0;
}

int Device::getCodecConfig(struct pal_media_config *config __unused)
{
    return 0;
}


int Device::getDefaultConfig(pal_param_device_capability_t capability __unused)
{
    return 0;
}

int Device::setDeviceAttributes(struct pal_device &dattr)
{
    int status = 0;

    mDeviceMutex.lock();
    PAL_INFO(LOG_TAG,"DeviceAttributes for Device Id %d updated", dattr.id);
    ar_mem_cpy(&deviceAttr, sizeof(struct pal_device), &dattr,
                     sizeof(struct pal_device));

    mDeviceMutex.unlock();
    return status;
}

int Device::freeCustomPayload(uint8_t **payload, size_t *payloadSize)
{
    if (*payload) {
        free(*payload);
        *payload = NULL;
        *payloadSize = 0;
    }
    return 0;
}

int Device::updateCustomPayload(void *payload, size_t size)
{
    if (!customPayloadSize) {
        customPayload = calloc(1, size);
    } else {
        customPayload = realloc(customPayload, customPayloadSize + size);
    }

    if (!customPayload) {
        PAL_ERR(LOG_TAG, "failed to allocate memory for custom payload");
        return -ENOMEM;
    }

    memcpy((uint8_t *)customPayload + customPayloadSize, payload, size);
    customPayloadSize += size;
    PAL_INFO(LOG_TAG, "customPayloadSize = %zu", customPayloadSize);
    return 0;
}

void* Device::getCustomPayload()
{
    return customPayload;
}

size_t Device::getCustomPayloadSize()
{
    return customPayloadSize;
}

int Device::getSndDeviceId()
{
    PAL_VERBOSE(LOG_TAG,"Device Id %d acquired", deviceAttr.id);
    return deviceAttr.id;
}

void Device::getCurrentSndDevName(char *name){
    strlcpy(name, mSndDeviceName, DEVICE_NAME_MAX_SIZE);
}

std::string Device::getPALDeviceName()
{
    PAL_VERBOSE(LOG_TAG, "Device name %s acquired", mPALDeviceName.c_str());
    return mPALDeviceName;
}

int Device::init(pal_param_device_connection_t device_conn __unused)
{
    return 0;
}

int Device::deinit(pal_param_device_connection_t device_conn __unused)
{
    return 0;
}

int Device::open()
{
    int status = 0;

    mDeviceMutex.lock();
    mPALDeviceName = rm->getPALDeviceName(this->deviceAttr.id);
    PAL_INFO(LOG_TAG, "Enter. deviceCount %d for device id %d (%s)", deviceCount,
            this->deviceAttr.id, mPALDeviceName.c_str());

    devObj = Device::getInstance(&deviceAttr, rm);

    if (deviceCount == 0) {
        std::string backEndName;
        rm->getBackendName(this->deviceAttr.id, backEndName);
        if (strlen(backEndName.c_str())) {
            devObj->setMediaConfig(rm, backEndName, &(this->deviceAttr));
        }

        status = rm->getAudioRoute(&audioRoute);
        if (0 != status) {
            PAL_ERR(LOG_TAG, "Failed to get the audio_route address status %d", status);
            goto exit;
        }
        status = rm->getSndDeviceName(deviceAttr.id , mSndDeviceName); //getsndName

        if (!UpdatedSndName.empty()) {
            PAL_DBG(LOG_TAG,"Update sndName %s, currently %s",
                    UpdatedSndName.c_str(), mSndDeviceName);
            strlcpy(mSndDeviceName, UpdatedSndName.c_str(), DEVICE_NAME_MAX_SIZE);
        }

        PAL_DBG(LOG_TAG, "audio_route %pK SND device name %s", audioRoute, mSndDeviceName);
        if (0 != status) {
            PAL_ERR(LOG_TAG, "Failed to obtain the device name from ResourceManager status %d", status);
            goto exit;
        }

        if (rm->IsQmpEnabled()) {
            if (strstr(this->deviceAttr.custom_config.custom_key, "unprocessed-hdr-mic")){
                if (Device::initHdrRoutine(this->deviceAttr.custom_config.custom_key))
                    PAL_ERR(LOG_TAG, "Failed to set QMP hdr config");
            }
        }
        enableDevice(audioRoute, mSndDeviceName);
    }
    ++deviceCount;

exit:
    PAL_INFO(LOG_TAG, "Exit. deviceCount %d for device id %d (%s), exit status: %d", deviceCount,
            this->deviceAttr.id, mPALDeviceName.c_str(), status);
    mDeviceMutex.unlock();
    return status;
}

int Device::close()
{
    int status = 0;
    mDeviceMutex.lock();
    PAL_INFO(LOG_TAG, "Enter. deviceCount %d for device id %d (%s)", deviceCount,
            this->deviceAttr.id, mPALDeviceName.c_str());
    if (deviceCount > 0) {
        --deviceCount;

       if (deviceCount == 0) {
           PAL_DBG(LOG_TAG, "Disabling device %d with snd dev %s", deviceAttr.id, mSndDeviceName);
           disableDevice(audioRoute, mSndDeviceName);
           mCurrentPriority = MIN_USECASE_PRIORITY;
           deviceStartStopCount = 0;
           if(rm->getProxyChannels() != 0)
               rm->setProxyChannels(0);
           if (customPayload) {
               free(customPayload);
               customPayload = NULL;
               customPayloadSize = 0;
           }
       }
    }
    PAL_INFO(LOG_TAG, "Exit. deviceCount %d for device id %d (%s), exit status %d", deviceCount,
            this->deviceAttr.id, mPALDeviceName.c_str(), status);
    mDeviceMutex.unlock();
    return status;
}

int Device::prepare()
{
    return 0;
}

int Device::start()
{
    int status = 0;

    mDeviceMutex.lock();
    status = start_l();
    mDeviceMutex.unlock();

    return status;
}

// must be called with mDeviceMutex held
int Device::start_l()
{
    int status = 0;
    std::string backEndName;
    std::shared_ptr<Device> dev = nullptr;

    PAL_DBG(LOG_TAG, "Enter. deviceCount %d deviceStartStopCount %d"
        " for device id %d (%s)", deviceCount, deviceStartStopCount,
            this->deviceAttr.id, mPALDeviceName.c_str());
    if (0 == deviceStartStopCount) {
        rm->getBackendName(this->deviceAttr.id, backEndName);
        if (!strlen(backEndName.c_str())) {
            PAL_ERR(LOG_TAG, "Error: Backend name not defined for %d in xml file\n", this->deviceAttr.id);
            status = -EINVAL;
            goto exit;
        }

        devObj = Device::getInstance(&deviceAttr, rm);
        if (devObj->isPluginPlaybackDevice(this->deviceAttr.id) || devObj->isDpDevice(this->deviceAttr.id)) {
            /* avoid setting invalid device attribute and the failure of starting device
             * when plugin device disconnects. Audio Policy Manager will go on finishing device switch.
             */
            if (this->deviceAttr.config.sample_rate == 0) {
                PAL_DBG(LOG_TAG, "overwrite samplerate to default value");
                this->deviceAttr.config.sample_rate = DEFAULT_OUTPUT_SAMPLING_RATE;
            }
            if (this->deviceAttr.config.bit_width == 0) {
                PAL_DBG(LOG_TAG, "overwrite bit width to default value");
                this->deviceAttr.config.bit_width = 16;
            }
            if (this->deviceAttr.config.ch_info.channels == 0) {
                PAL_DBG(LOG_TAG, "overwrite channel to default value");
                this->deviceAttr.config.ch_info.channels = DEFAULT_OUTPUT_CHANNEL;
                this->deviceAttr.config.ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FL;
                this->deviceAttr.config.ch_info.ch_map[1] = PAL_CHMAP_CHANNEL_FR;
            }
            if (this->deviceAttr.config.aud_fmt_id == 0) {
                PAL_DBG(LOG_TAG, "overwrite aud_fmt_id to default value");
                this->deviceAttr.config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S16_LE;
            }
        }
        devObj->setMediaConfig(rm, backEndName, &(this->deviceAttr));

        if (customPayloadSize) {
            status = devObj->setCustomPayload(rm, backEndName,
                                        customPayload, customPayloadSize);
            if (status)
                 PAL_ERR(LOG_TAG, "Error: Dev setParam failed for %d\n",
                                   this->deviceAttr.id);
        }
    }
    deviceStartStopCount++;
exit :
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int Device::stop()
{
    int status = 0;

    mDeviceMutex.lock();
    status = stop_l();
    mDeviceMutex.unlock();

    return status;
}

// must be called with mDeviceMutex held
int Device::stop_l()
{
    PAL_DBG(LOG_TAG, "Enter. deviceCount %d deviceStartStopCount %d"
        " for device id %d (%s)", deviceCount, deviceStartStopCount,
            this->deviceAttr.id, mPALDeviceName.c_str());

    if (deviceStartStopCount > 0) {
        --deviceStartStopCount;
    }

    return 0;
}

int32_t Device::setDeviceParameter(uint32_t param_id __unused, void *param __unused)
{
    return 0;
}

int32_t Device::getDeviceParameter(uint32_t param_id __unused, void **param __unused)
{
    return 0;
}

int32_t Device::setParameter(uint32_t param_id __unused, void *param __unused)
{
    return 0;
}

int32_t Device::getParameter(uint32_t param_id __unused, void **param __unused)
{
    return 0;
}

int32_t Device::checkAndUpdateSampleRate(uint32_t *sampleRate)
{
    return 0;
}

int32_t Device::checkAndUpdateBitWidth(uint32_t *bitWidth)
{
    return 0;
}

int Device::selectBestConfig(struct pal_device *dattr,
                             struct pal_stream_attributes *sattr,
                             bool is_playback, struct pal_device_info *devinfo)
{
    return 0;
}

int Device::getMaxChannel()
{
    return 0;
}

bool Device::isSupportedSR(int sr)
{
    return 0;
}

int Device::getHighestSupportedSR()
{
    return 0;
}

int32_t Device::isBitWidthSupported(uint32_t bitWidth)
{
    return 0;
}

int Device::getHighestSupportedBps()
{
    return 0;
}

bool Device::isDeviceConnected(struct pal_usb_device_address addr)
{
    return 0;
}

int32_t Device::checkDeviceStatus() {
    return 0;
}

int32_t Device::getDeviceConfig(struct pal_device *deviceattr,
                                struct pal_stream_attributes *sAttr) {
    return 0;
}

int32_t Device::configureDeviceClockSrc(char const *mixerStrClockSrc, const uint32_t clockSrc)
{
    struct mixer *hwMixerHandle = NULL;
    struct mixer_ctl *clockSrcCtrl = NULL;
    int32_t ret = 0;

    if (!mixerStrClockSrc) {
        PAL_ERR(LOG_TAG, "Invalid argument - mixerStrClockSrc");
        ret = -EINVAL;
        goto exit;
    }

    ret = rm->getHwAudioMixer(&hwMixerHandle);
    if (ret) {
        PAL_ERR(LOG_TAG, "getHwAudioMixer() failed %d", ret);
        goto exit;
    }

    /* Hw mixer control registration is optional in case
     * clock source selection is not required
     */
    clockSrcCtrl = mixer_get_ctl_by_name(hwMixerHandle, mixerStrClockSrc);
    if (!clockSrcCtrl) {
        PAL_DBG(LOG_TAG, "%s hw mixer control not identified", mixerStrClockSrc);
        goto exit;
    }

    PAL_DBG(LOG_TAG, "HwMixer set %s = %d", mixerStrClockSrc, clockSrc);
    ret = mixer_ctl_set_value(clockSrcCtrl, 0, clockSrc);
    if (ret)
        PAL_ERR(LOG_TAG, "HwMixer set %s = %d failed", mixerStrClockSrc, clockSrc);

exit:
    return ret;
}

/* insert inDevAttr if incoming device has higher priority */
bool Device::compareStreamDevAttr(const struct pal_device *inDevAttr,
                            const struct pal_device_info *inDevInfo,
                            struct pal_device *curDevAttr,
                            const struct pal_device_info *curDevInfo)
{
    bool insert = false;

    if (!inDevAttr || !inDevInfo || !curDevAttr || !curDevInfo) {
        PAL_ERR(LOG_TAG, "invalid pointer cannot update attr");
        goto exit;
    }

     /* check snd device name */
    if (inDevInfo->sndDevName_overwrite && !curDevInfo->sndDevName_overwrite) {
        PAL_DBG(LOG_TAG, "snd overwrite found");
        insert = true;
        goto exit;
    }


    /* check channels */
    if (inDevInfo->channels_overwrite && !curDevInfo->channels_overwrite) {
        PAL_DBG(LOG_TAG, "ch overwrite found");
        insert = true;
        goto exit;
    } else if ((inDevInfo->channels_overwrite && curDevInfo->channels_overwrite) ||
               (!inDevInfo->channels_overwrite && !curDevInfo->channels_overwrite)) {
        if (inDevAttr->config.ch_info.channels > curDevAttr->config.ch_info.channels) {
            PAL_DBG(LOG_TAG, "incoming dev has higher ch count, in ch: %d, cur ch: %d",
                            inDevAttr->config.ch_info.channels, curDevAttr->config.ch_info.channels);
            insert = true;
            goto exit;
        }
    }

    /* check sample rate */
    if (inDevInfo->samplerate_overwrite && !curDevInfo->samplerate_overwrite) {
        PAL_DBG(LOG_TAG, "sample rate overwrite found");
        insert = true;
        goto exit;
    } else if ((inDevInfo->samplerate_overwrite && curDevInfo->samplerate_overwrite) &&
               (inDevAttr->config.sample_rate > curDevAttr->config.sample_rate)) {
        PAL_DBG(LOG_TAG, "both have sr overwrite set, incoming dev has higher sr: %d, cur sr: %d",
                        inDevAttr->config.sample_rate, curDevAttr->config.sample_rate);
        insert = true;
        goto exit;
    } else if (!inDevInfo->samplerate_overwrite && !curDevInfo->samplerate_overwrite) {
        if ((inDevAttr->config.sample_rate % 44100 == 0) &&
            (curDevAttr->config.sample_rate % 44100 != 0)) {
            PAL_DBG(LOG_TAG, "incoming sample rate is 44.1K");
            insert = true;
            goto exit;
        } else if (inDevAttr->config.sample_rate > curDevAttr->config.sample_rate) {
            if (curDevAttr->config.sample_rate % 44100 == 0 &&
                inDevAttr->config.sample_rate % 48000 == 0) {
                PAL_DBG(LOG_TAG, "current stream is running at 44.1KHz");
                insert = false;
            } else {
                PAL_DBG(LOG_TAG, "incoming dev has higher sr: %d, cur sr: %d",
                            inDevAttr->config.sample_rate, curDevAttr->config.sample_rate);
                insert = true;
                goto exit;
            }
        }
    }

    /* check streams bit width */
    if (inDevInfo->bit_width_overwrite && !curDevInfo->bit_width_overwrite) {
        if (isPalPCMFormat(inDevAttr->config.aud_fmt_id)) {
            PAL_DBG(LOG_TAG, "bit width overwrite found");
            insert = true;
            goto exit;
        }
    } else if ((inDevInfo->bit_width_overwrite && curDevInfo->bit_width_overwrite) ||
               (!inDevInfo->bit_width_overwrite && !curDevInfo->bit_width_overwrite)) {
        if (isPalPCMFormat(inDevAttr->config.aud_fmt_id) &&
            (inDevAttr->config.bit_width > curDevAttr->config.bit_width)) {
            PAL_DBG(LOG_TAG, "incoming dev has higher bw: %d, cur bw: %d",
                            inDevAttr->config.bit_width, curDevAttr->config.bit_width);
            insert = true;
            goto exit;
        }
    }

exit:
    return insert;
}

int Device::insertStreamDeviceAttr(struct pal_device *inDevAttr,
                                 Stream* streamHandle)
{
    pal_device_info inDevInfo, curDevInfo;
    struct pal_device *curDevAttr, *newDevAttr;
    std::string key = "";
    pal_stream_attributes strAttr;

    if (!streamHandle) {
        PAL_ERR(LOG_TAG, "invalid stream handle");
        return -EINVAL;
    }
    if (!inDevAttr) {
        PAL_ERR(LOG_TAG, "invalid dev cannot get device attr");
        return -EINVAL;
    }

    streamHandle->getStreamAttributes(&strAttr);

    newDevAttr = (struct pal_device *) calloc(1, sizeof(struct pal_device));
    if (!newDevAttr) {
        PAL_ERR(LOG_TAG, "failed to allocate memory for pal device");
        return -ENOMEM;
    }

    key = inDevAttr->custom_config.custom_key;

    /* get the incoming stream dev info */
    rm->getDeviceInfo(inDevAttr->id, strAttr.type, key, &inDevInfo);

    ar_mem_cpy(newDevAttr, sizeof(struct pal_device), inDevAttr,
                 sizeof(struct pal_device));

    mDeviceMutex.lock();
    if (mStreamDevAttr.empty()) {
        mStreamDevAttr.insert(std::make_pair(inDevInfo.priority, std::make_pair(streamHandle, newDevAttr)));
        PAL_DBG(LOG_TAG, "insert the first device attribute");
        goto exit;
    }

    /*
     * this map is sorted with stream priority, and the top one will always
     * be with highest stream priority.
     * <priority(it.first):<stream_attr(it.second.first):pal_device(it.second.second)>>
     * If stream priority is the same, new attributes will be inserted to the map with:
     *   1. device attr with snd name overwrite flag set
     *   2. device attr with channel overwrite set, or a higher channel count
     *   3. device attr with sample rate overwrite set, or a higher sampling rate
     *   4. device attr with bit depth overwrite set, or a higher bit depth
     */
    for (auto it = mStreamDevAttr.begin(); ; it++) {
        /* get the current stream dev info to be compared with incoming device */
        struct pal_stream_attributes curStrAttr;
        if (it != mStreamDevAttr.end()) {
            (*it).second.first->getStreamAttributes(&curStrAttr);
            curDevAttr = (*it).second.second;
            rm->getDeviceInfo(curDevAttr->id, curStrAttr.type,
                            curDevAttr->custom_config.custom_key, &curDevInfo);
        }

        if (it == mStreamDevAttr.end()) {
            /* if reaches to the end, then the new dev attr will be inserted to the end */
            PAL_DBG(LOG_TAG, "incoming stream: %d has lowest priority, insert to the end", strAttr.type);
            mStreamDevAttr.insert(std::make_pair(inDevInfo.priority, std::make_pair(streamHandle, newDevAttr)));
            break;
        } else if (inDevInfo.priority < (*it).first) {
            /* insert if incoming stream has higher priority than current */
            mStreamDevAttr.insert(it, std::make_pair(inDevInfo.priority, std::make_pair(streamHandle, newDevAttr)));
            break;
        } else if (inDevInfo.priority == (*it).first) {
            /* if stream priority is the same, check attributes priority */
            if (compareStreamDevAttr(inDevAttr, &inDevInfo, curDevAttr, &curDevInfo)) {
                PAL_DBG(LOG_TAG, "incoming stream: %d has higher priority than cur stream %d",
                                strAttr.type, curStrAttr.type);
                mStreamDevAttr.insert(it, std::make_pair(inDevInfo.priority, std::make_pair(streamHandle, newDevAttr)));
                break;
            }
        }
    }

exit:
    PAL_DBG(LOG_TAG, "dev: %d attr inserted are: priority: 0x%x, stream type: %d, ch: %d,"
                     " sr: %d, bit_width: %d, fmt: %d, sndDev: %s, custom_key: %s",
                    getSndDeviceId(), inDevInfo.priority, strAttr.type,
                    newDevAttr->config.ch_info.channels,
                    newDevAttr->config.sample_rate,
                    newDevAttr->config.bit_width,
                    newDevAttr->config.aud_fmt_id,
                    newDevAttr->sndDevName,
                    newDevAttr->custom_config.custom_key);

#if DUMP_DEV_ATTR
    PAL_DBG(LOG_TAG, "======dump StreamDevAttr Inserted dev: %d ======", getSndDeviceId());
    int i = 0;
    for (auto it = mStreamDevAttr.begin(); it != mStreamDevAttr.end(); it++) {
        pal_stream_attributes dumpstrAttr;
        uint32_t dumpPriority = (*it).first;
        (*it).second.first->getStreamAttributes(&dumpstrAttr);
        struct pal_device *dumpDevAttr = (*it).second.second;
        PAL_DBG(LOG_TAG, "entry: %d", i);
        PAL_DBG(LOG_TAG, "str pri: 0x%x, str type: %d, ch %d, sr %d, bit_width %d,"
                         " fmt %d, sndDev: %s, custom_key: %s",
                         dumpPriority, dumpstrAttr.type,
                         dumpDevAttr->config.ch_info.channels,
                         dumpDevAttr->config.sample_rate,
                         dumpDevAttr->config.bit_width,
                         dumpDevAttr->config.aud_fmt_id,
                         dumpDevAttr->sndDevName,
                         dumpDevAttr->custom_config.custom_key);
        i++;
    }
#endif

    mDeviceMutex.unlock();
    return 0;
}

void Device::removeStreamDeviceAttr(Stream* streamHandle)
{
    mDeviceMutex.lock();
    if (mStreamDevAttr.empty()) {
        PAL_ERR(LOG_TAG, "empty device attr for device %d", getSndDeviceId());
        mDeviceMutex.unlock();
        return;
    }

    for (auto it = mStreamDevAttr.begin(); it != mStreamDevAttr.end(); it++) {
        Stream* curStream = (*it).second.first;
        if (curStream == streamHandle) {
            uint32_t priority = (*it).first;
            pal_stream_attributes strAttr;
            (*it).second.first->getStreamAttributes(&strAttr);
            pal_device *devAttr = (*it).second.second;
            PAL_DBG(LOG_TAG, "found entry for stream:%d", strAttr.type);
            PAL_DBG(LOG_TAG, "dev: %d attr removed are: priority: 0x%x, stream type: %d, ch: %d,"
                             " sr: %d, bit_width: %d, fmt: %d, sndDev: %s, custom_key: %s",
                            getSndDeviceId(), priority, strAttr.type,
                            devAttr->config.ch_info.channels,
                            devAttr->config.sample_rate,
                            devAttr->config.bit_width,
                            devAttr->config.aud_fmt_id,
                            devAttr->sndDevName,
                            devAttr->custom_config.custom_key);
            free((*it).second.second);
            mStreamDevAttr.erase(it);
            break;
        }
    }

#if DUMP_DEV_ATTR
    PAL_DBG(LOG_TAG, "=====dump StreamDevAttr after removing dev: %d ======", getSndDeviceId());
    int i = 0;
    for (auto it = mStreamDevAttr.begin(); it != mStreamDevAttr.end(); it++) {
        pal_stream_attributes dumpstrAttr;
        uint32_t dumpPriority = (*it).first;
        (*it).second.first->getStreamAttributes(&dumpstrAttr);
        struct pal_device *dumpDevAttr = (*it).second.second;
        PAL_DBG(LOG_TAG, "entry: %d", i);
        PAL_DBG(LOG_TAG, "str pri: 0x%x, str type: %d, ch %d, sr %d, bit_width %d,"
                         " fmt %d, sndDev: %s, custom_key: %s",
                         dumpPriority, dumpstrAttr.type,
                         dumpDevAttr->config.ch_info.channels,
                         dumpDevAttr->config.sample_rate,
                         dumpDevAttr->config.bit_width,
                         dumpDevAttr->config.aud_fmt_id,
                         dumpDevAttr->sndDevName,
                         dumpDevAttr->custom_config.custom_key);
        i++;
    }
#endif
    mDeviceMutex.unlock();
}

int Device::getTopPriorityDeviceAttr(struct pal_device *deviceAttr, uint32_t *streamPrio)
{
    mDeviceMutex.lock();
    if (mStreamDevAttr.empty()) {
        PAL_ERR(LOG_TAG, "empty device attr for device %d", getSndDeviceId());
        mDeviceMutex.unlock();
        return -EINVAL;
    }

    auto it = mStreamDevAttr.begin();
    *streamPrio = (*it).first;
    ar_mem_cpy(deviceAttr, sizeof(struct pal_device),
            (*it).second.second, sizeof(struct pal_device));
    /* update snd dev name */
    std::string sndDevName(deviceAttr->sndDevName);
    rm->updateSndName(deviceAttr->id, sndDevName);
    /* update sample rate if it's valid */
    if (mSampleRate)
        deviceAttr->config.sample_rate = mSampleRate;
    if (mBitWidth) {
        deviceAttr->config.bit_width = mBitWidth;
        deviceAttr->config.aud_fmt_id = rm->getAudioFmt(mBitWidth);
    }
#if DUMP_DEV_ATTR
    pal_stream_attributes dumpstrAttr;
    (*it).second.first->getStreamAttributes(&dumpstrAttr);
    PAL_DBG(LOG_TAG, "======dump StreamDevAttr Retrieved dev: %d ======", getSndDeviceId());
    PAL_DBG(LOG_TAG, "str pri: 0x%x, str type: %d, ch %d, sr %d, bit_width %d,"
                     " fmt %d, sndDev: %s, custom_key: %s",
                     (*it).first, dumpstrAttr.type,
                     deviceAttr->config.ch_info.channels,
                     deviceAttr->config.sample_rate,
                     deviceAttr->config.bit_width,
                     deviceAttr->config.aud_fmt_id,
                     deviceAttr->sndDevName,
                     deviceAttr->custom_config.custom_key);
#endif

    mDeviceMutex.unlock();
    return 0;
}

unsigned int Device::palToSndDriverFormat(uint32_t fmt_id)
{
    switch (fmt_id) {
        case PAL_AUDIO_FMT_PCM_S32_LE:
            return SNDRV_PCM_FORMAT_S32_LE;
        case PAL_AUDIO_FMT_PCM_S8:
            return SNDRV_PCM_FORMAT_S8;
        case PAL_AUDIO_FMT_PCM_S24_3LE:
            return SNDRV_PCM_FORMAT_S24_3LE;
        case PAL_AUDIO_FMT_PCM_S24_LE:
            return SNDRV_PCM_FORMAT_S24_LE;
        default:
        case PAL_AUDIO_FMT_PCM_S16_LE:
            return SNDRV_PCM_FORMAT_S16_LE;
    };
}

unsigned int Device::bitsToAlsaFormat(unsigned int bits)
{
    switch (bits) {
        case 32:
            return SNDRV_PCM_FORMAT_S32_LE;
        case 8:
            return SNDRV_PCM_FORMAT_S8;
        case 24:
            return SNDRV_PCM_FORMAT_S24_3LE;
        default:
        case 16:
            return SNDRV_PCM_FORMAT_S16_LE;
    };
}

struct mixer_ctl *Device::getBeMixerControl(struct mixer *am, std::string beName,
        uint32_t idx)
{
    std::ostringstream cntrlName;

    cntrlName << beName << beCtrlNames[idx];
    PAL_DBG(LOG_TAG, "mixer control %s", cntrlName.str().data());
    return mixer_get_ctl_by_name(am, cntrlName.str().data());
}

int Device::setCustomPayload(std::shared_ptr<ResourceManager> rmHandle,
                                std::string backEndName, void *payload, size_t size)
{
    struct mixer_ctl *ctl = NULL;
    struct mixer *mixerHandle = NULL;
    int status = 0;

    status = rmHandle->getVirtualAudioMixer(&mixerHandle);
    if (status) {
        PAL_ERR(LOG_TAG, "Error: Failed to get mixer handle\n");
        return status;
    }

    ctl = getBeMixerControl(mixerHandle, backEndName, BE_SETPARAM);
    if (!ctl) {
        PAL_ERR(LOG_TAG, "invalid mixer control: %s %s", backEndName.c_str(),
                beCtrlNames[BE_SETPARAM]);
        return -EINVAL;
    }

    return mixer_ctl_set_array(ctl, payload, size);
}

int Device::setMediaConfig(std::shared_ptr<ResourceManager> rmHandle,
                            std::string backEndName, struct pal_device *dAttr)
{
    struct mixer_ctl *ctl = NULL;
    long aif_media_config[4];
    long aif_group_atrr_config[5];
    struct mixer *mixerHandle = NULL;
    int status = 0;
    std::shared_ptr<group_dev_config_t> groupDevConfig;
    group_dev_config_t currentGroupDevConfig;

    status = rmHandle->getVirtualAudioMixer(&mixerHandle);
    if (status) {
        PAL_ERR(LOG_TAG, "Error: Failed to get mixer handle\n");
        return status;
    }

    aif_media_config[0] = dAttr->config.sample_rate;
    aif_media_config[1] = dAttr->config.ch_info.channels;

    if (!isPalPCMFormat((uint32_t)dAttr->config.aud_fmt_id)) {
        /*
         *Only for configuring the BT A2DP device backend we use
         *bitwidth instead of aud_fmt_id
         */
        aif_media_config[2] = bitsToAlsaFormat(dAttr->config.bit_width);
        aif_media_config[3] = AGM_DATA_FORMAT_COMPR_OVER_PCM_PACKETIZED;
    } else {
        aif_media_config[2] = palToSndDriverFormat((uint32_t)dAttr->config.aud_fmt_id);
        aif_media_config[3] = AGM_DATA_FORMAT_FIXED_POINT;
    }

    // if it's virtual port, need to set group attribute as well
    groupDevConfig = rmHandle->getActiveGroupDevConfig();
    if (groupDevConfig && (dAttr->id == PAL_DEVICE_OUT_SPEAKER ||
        dAttr->id == PAL_DEVICE_OUT_HANDSET ||
        dAttr->id == PAL_DEVICE_OUT_ULTRASOUND)) {
        std::string truncatedBeName = backEndName;
        // remove "-VIRT-x" which length is 7
        truncatedBeName.erase(truncatedBeName.end() - 7, truncatedBeName.end());
        ctl = getBeMixerControl(mixerHandle, truncatedBeName , BE_GROUP_ATTR);
        if (!ctl) {
        PAL_ERR(LOG_TAG, "invalid mixer control: %s %s", truncatedBeName.c_str(),
                beCtrlNames[BE_GROUP_ATTR]);
        return -EINVAL;
        }
        if (groupDevConfig->grp_dev_hwep_cfg.sample_rate)
            aif_group_atrr_config[0] = groupDevConfig->grp_dev_hwep_cfg.sample_rate;
        else
            aif_group_atrr_config[0] = dAttr->config.sample_rate;
        if (groupDevConfig->grp_dev_hwep_cfg.channels)
            aif_group_atrr_config[1] = groupDevConfig->grp_dev_hwep_cfg.channels;
        else
            aif_group_atrr_config[1] = dAttr->config.ch_info.channels;
        aif_group_atrr_config[2] = palToSndDriverFormat(
                                    groupDevConfig->grp_dev_hwep_cfg.aud_fmt_id);
        aif_group_atrr_config[3] = AGM_DATA_FORMAT_FIXED_POINT;
        aif_group_atrr_config[4] = groupDevConfig->grp_dev_hwep_cfg.slot_mask;

        mixer_ctl_set_array(ctl, &aif_group_atrr_config,
                               sizeof(aif_group_atrr_config)/sizeof(aif_group_atrr_config[0]));
        PAL_INFO(LOG_TAG, "%s rate ch fmt data_fmt slot_mask %ld %ld %ld %ld %ld\n", truncatedBeName.c_str(),
                aif_group_atrr_config[0], aif_group_atrr_config[1], aif_group_atrr_config[2],
                aif_group_atrr_config[3], aif_group_atrr_config[4]);
        rmHandle->setCurrentGroupDevConfig(groupDevConfig, aif_group_atrr_config[0], aif_group_atrr_config[1]);
    }
    ctl = getBeMixerControl(mixerHandle, backEndName , BE_MEDIAFMT);
    if (!ctl) {
        PAL_ERR(LOG_TAG, "invalid mixer control: %s %s", backEndName.c_str(),
                beCtrlNames[BE_MEDIAFMT]);
        return -EINVAL;
    }

    PAL_INFO(LOG_TAG, "%s rate ch fmt data_fmt %ld %ld %ld %ld\n", backEndName.c_str(),
                     aif_media_config[0], aif_media_config[1],
                     aif_media_config[2], aif_media_config[3]);

    return mixer_ctl_set_array(ctl, &aif_media_config,
                               sizeof(aif_media_config)/sizeof(aif_media_config[0]));
}
