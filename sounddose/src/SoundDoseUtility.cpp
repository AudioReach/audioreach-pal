/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "PAL: SoundDoseUtility"

#include "SoundDoseUtility.h"

#include <agm/agm_api.h>

#include <chrono>
#include <ctime>

#include "PalMappings.h"
#include "PayloadBuilder.h"
#include "ResourceManager.h"
#include "SessionAgm.h"
#include "SessionAlsaUtils.h"
#include "kvh2xml.h"
#include "sound_dose_api.h"

#define DEFAULT_PERIOD_SIZE 256
#define DEFAULT_PERIOD_COUNT 4

bool SoundDoseUtility::isEnabled(pal_device_id_t deviceId) {
    if (!ResourceManager::IsSoundDoseEnabled()) {
        PAL_DBG(LOG_TAG, "sounddose is disabled for device %d", deviceId);
        return false;
    }
    return supportsSoundDose(deviceId);
}

bool SoundDoseUtility::supportsSoundDose(pal_device_id_t deviceId) {
    switch (deviceId) {
        case PAL_DEVICE_OUT_WIRED_HEADSET:
        case PAL_DEVICE_OUT_WIRED_HEADPHONE:
        case PAL_DEVICE_OUT_BLUETOOTH_A2DP:
        case PAL_DEVICE_OUT_USB_HEADSET:
        case PAL_DEVICE_OUT_BLUETOOTH_BLE:
        case PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST:
            return true;
        default:
            return false;
    }
}

pal_global_callback SoundDoseUtility::getCallback() {
    return mResourceManager ? mResourceManager->getCallback() : nullptr;
}

uint64_t SoundDoseUtility::getCookie() {
    return mResourceManager ? mResourceManager->getCookie() : 0;
}

SoundDoseUtility::SoundDoseUtility(Device *devObj, const struct pal_device device) {
    pal_device_id_t deviceId = device.id;
    int status = 0;
    mPalDevice = device;
    mDevObj = devObj;

    mResourceManager = ResourceManager::getInstance();
    if (!mResourceManager) {
        PAL_ERR(LOG_TAG, "Resource manager instance unavailable, disable sounddose");
        return;
    }

    status = mResourceManager->getVirtualAudioMixer(&virtMixer);
    if (status) {
        PAL_ERR(LOG_TAG, "getVirtualAudioMixer failed %d", status);
        return;
    }

    mIsEnabled = isEnabled(deviceId);
    PAL_INFO(LOG_TAG, "device %s mIsEnabled %d", toString(&mPalDevice).c_str(), mIsEnabled);
}

SoundDoseUtility::~SoundDoseUtility() {
    PAL_DBG(LOG_TAG, "device %s mIsEnabled %d", toString(&mPalDevice).c_str(), mIsEnabled);
}

void SoundDoseUtility::startComputation() {
    if (mIsEnabled) {
        startComputationInternal();
    } else {
        startSimulation();
    }
}

void SoundDoseUtility::stopComputation() {
    // Logic to stop dosage computation for the device
    if (mIsEnabled) {
        stopComputationInternal();
    } else {
        stopSimulation();
    }
}

void SoundDoseUtility::startComputationInternal() {
    // Logic to start dosage computation for the device
    // Call pcm_open, register for event and pcm_start.
    PAL_INFO(LOG_TAG, "%p startComputation for device %s ", this, toString(&mPalDevice).c_str());
    int ret = 0, dir = RX_HOSTLESS;
    char mSndDeviceName[128] = {0};
    uint32_t devicePropId[] = {0x08000010, 1, 0x2};
    struct pal_device device;
    std::string deviceName;

    struct pal_device_info deviceSoundDose;
    struct pal_stream_attributes sAttr;
    struct pal_channel_info ch_info;
    struct pcm_config config;
    std::vector<std::pair<int, int>> keyVector;
    std::vector<std::pair<int, int>> calVector;
    session_callback sessionCb;
    std::string backEndName;
    struct agmMetaData deviceMetaData(nullptr, 0);
    struct mixer_ctl *beMetaDataMixerCtrl = nullptr;
    struct mixer_ctl *connectCtrl = NULL;
    struct audio_route *audioRoute = NULL;
    unsigned int flags;
    size_t payloadSize = 0;
    std::ostringstream connectCtrlName;
    std::ostringstream connectCtrlNameBe;
    struct agm_event_reg_cfg event_cfg;

    sessionCb = handleSoundDoseCallback;
    memset(&device, 0, sizeof(device));
    memset(&sAttr, 0, sizeof(sAttr));
    memset(&config, 0, sizeof(config));

    // setup dummy dev tx graph with sound dose module.
    device.id = PAL_DEVICE_OUT_SOUND_DOSE;
    deviceName = (std::string)deviceNameLUT.at(device.id);

    strlcpy(mSndDeviceName, "dummy-dev", DEVICE_NAME_MAX_SIZE);
    /* Configure device attributes TODO: check the config? */
    ch_info.channels = CHANNELS_1;
    ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FL;
    device.config.ch_info = ch_info;
    device.config.sample_rate = 48000;
    device.config.bit_width = BITWIDTH_16;
    device.config.aud_fmt_id = PAL_AUDIO_FMT_DEFAULT_COMPRESSED;

    ret = mResourceManager->getAudioRoute(&audioRoute);
    if (0 != ret) {
        PAL_ERR(LOG_TAG, "Failed in getAudioRoute %d", ret);
        goto exit;
    }

    mResourceManager->getBackendName(device.id, backEndName);
    if (!strlen(backEndName.c_str())) {
        PAL_ERR(LOG_TAG, "getBackendName failed for id %d device %s", device.id,
                deviceName.c_str());
        goto exit;
    }

    ret = PayloadBuilder::getDeviceKV(device.id, keyVector);
    if (0 != ret) {
        PAL_ERR(LOG_TAG, "getDeviceKV for id %d device %s", device.id, deviceName.c_str());
        goto exit;
    }

    SessionAlsaUtils::getAgmMetaData(keyVector, calVector, (struct prop_data *)devicePropId,
                                     deviceMetaData);
    if (!deviceMetaData.size) {
        PAL_ERR(LOG_TAG, "SoundDose device metadata is zero");
        ret = -ENOMEM;
        goto exit;
    }
    connectCtrlNameBe << backEndName << " metadata";
    beMetaDataMixerCtrl = mixer_get_ctl_by_name(virtMixer, connectCtrlNameBe.str().data());
    if (!beMetaDataMixerCtrl) {
        PAL_ERR(LOG_TAG, "invalid mixer control for SoundDose : %s", backEndName.c_str());
        ret = -EINVAL;
        goto exit;
    }

    if (deviceMetaData.size) {
        ret = mixer_ctl_set_array(beMetaDataMixerCtrl, (void *)deviceMetaData.buf,
                                  deviceMetaData.size);
        free(deviceMetaData.buf);
        deviceMetaData.buf = nullptr;
    } else {
        PAL_ERR(LOG_TAG, "Device Metadata not set for RX path");
        ret = -EINVAL;
        goto exit;
    }

    ret = mDevObj->setMediaConfig(mResourceManager, backEndName, &device);
    if (ret) {
        PAL_ERR(LOG_TAG, "setDeviceMediaConfig for sound dose device failed");
        goto exit;
    }

    /* Retrieve Hostless PCM device id */
    sAttr.type = PAL_STREAM_LOW_LATENCY;
    sAttr.direction = PAL_AUDIO_OUTPUT;
    dir = RX_HOSTLESS;
    flags = PCM_OUT;
    pcmDevIdRx = mResourceManager->allocateFrontEndIds(sAttr, dir);
    if (pcmDevIdRx.size() == 0) {
        PAL_ERR(LOG_TAG, "allocateFrontEndIds failed");
        ret = -ENOSYS;
        goto exit;
    }

    connectCtrlName << "PCM" << pcmDevIdRx.at(0) << " connect";
    connectCtrl = mixer_get_ctl_by_name(virtMixer, connectCtrlName.str().data());
    if (!connectCtrl) {
        PAL_ERR(LOG_TAG, "invalid mixer control: %s", connectCtrlName.str().data());
        goto free_fe;
    }
    ret = mixer_ctl_set_enum_by_string(connectCtrl, backEndName.c_str());
    if (ret) {
        PAL_ERR(LOG_TAG, "Mixer control %s set with %s failed: %d", connectCtrlName.str().data(),
                backEndName.c_str(), ret);
        goto free_fe;
    }

    // TODO: set PARAM_ID_SOUND_DOSE_MEL_EVENTS_CONFIG for time between 2 mel values
    // TODO: set PARAM_ID_SOUND_DOSE_RS2_UPPER_BOUND if configured from fwk.
    config.rate = 48000;
    config.format = PCM_FORMAT_S16_LE;
    config.channels = CHANNELS_1;
    config.period_size = DEFAULT_PERIOD_SIZE;
    config.period_count = DEFAULT_PERIOD_COUNT;
    config.start_threshold = 0;
    config.stop_threshold = INT_MAX;
    config.silence_threshold = 0;

    rxPcm = pcm_open(mResourceManager->getVirtualSndCard(), pcmDevIdRx.at(0), flags, &config);
    if (!rxPcm) {
        PAL_ERR(LOG_TAG, "pcm_open failed");
        goto free_fe;
    }

    if (!pcm_is_ready(rxPcm)) {
        PAL_ERR(LOG_TAG, "pcm_open not ready");
        goto err_pcm_open;
    }

    PAL_DBG(LOG_TAG, "registering events for SoundDose module");
    payloadSize = sizeof(struct agm_event_reg_cfg);

    /* Register for EVENT_ID_SOUND_DOSE_MEL_VALUES. */
    event_cfg.event_id = EVENT_ID_SOUND_DOSE_MEL_VALUES;
    event_cfg.event_config_payload_size = 0;
    event_cfg.is_register = 1;

    ret = SessionAlsaUtils::registerMixerEvent(virtMixer, pcmDevIdRx.at(0), backEndName.c_str(),
                                               MODULE_SOUND_DOSE, (void *)&event_cfg, payloadSize);
    if (ret) {
        PAL_ERR(LOG_TAG, "registerMixerEvent failed");
        // Fatal Error. Sound Dose Computation Won't work
        goto err_pcm_open;
    }

    // Register to mixtureControlEvents and wait for the MEL values
    ret = mResourceManager->registerMixerEventCallback(pcmDevIdRx, sessionCb, (uint64_t)this, true);
    if (ret != 0) {
        PAL_ERR(LOG_TAG, "registerMixerEventCallback failed");
        // Fatal Error. Sound Dose Computation Won't work
        goto err_pcm_open;
    }

    PAL_DBG(LOG_TAG, " enable device SoundDose module");
    enableDevice(audioRoute, mSndDeviceName);

    PAL_DBG(LOG_TAG, "pcm start for Sound Dose graph");
    if (pcm_start(rxPcm) < 0) {
        PAL_ERR(LOG_TAG, "pcm start failed for Sound Dose graph");
        ret = -ENOSYS;
        goto err_pcm_open;
    }

    goto exit;
err_pcm_open:
    if (rxPcm) {
        pcm_close(rxPcm);
        disableDevice(audioRoute, mSndDeviceName);
        rxPcm = NULL;
    }

free_fe:
    mResourceManager->freeFrontEndIds(pcmDevIdRx, sAttr, dir);
exit:

    PAL_DBG(LOG_TAG, "Exit");
}

void SoundDoseUtility::stopComputationInternal() {
    PAL_INFO(LOG_TAG, "%p stopComputation for device %s", this, toString(&mPalDevice).c_str());
    struct pal_stream_attributes sAttr;
    struct pal_channel_info ch_info;
    int ret = 0, dir;
    struct pal_device device;
    std::string backEndName;
    struct agm_event_reg_cfg event_cfg;
    size_t payloadSize = 0;
    // setup dummy dev tx graph with sound dose module.
    device.id = PAL_DEVICE_OUT_SOUND_DOSE;

    /* Configure device attributes TODO: check the config? */
    ch_info.channels = CHANNELS_1;
    ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FL;
    device.config.ch_info = ch_info;
    device.config.bit_width = BITWIDTH_16;
    device.config.aud_fmt_id = PAL_AUDIO_FMT_DEFAULT_COMPRESSED;

    mResourceManager->getBackendName(device.id, backEndName);
    if (!strlen(backEndName.c_str())) {
        PAL_ERR(LOG_TAG, "getBackendName failed for id %d", device.id);
        goto exit;
    }

    // add mutex if required

    if (!rxPcm) {
        PAL_ERR(LOG_TAG, "rxPcm is null");
        return;
    }

    memset(&sAttr, 0, sizeof(sAttr));
    sAttr.type = PAL_STREAM_LOW_LATENCY;
    sAttr.direction = PAL_AUDIO_OUTPUT;
    dir = RX_HOSTLESS;

    pcm_stop(rxPcm);

    /* Add getParameter for sounddose for remaining data.
     * parse the data received and
     * invoke callback to hal wih the data */
    getSoundDoseMelValues();

    mResourceManager->freeFrontEndIds(pcmDevIdRx, sAttr, dir);
    payloadSize = sizeof(struct agm_event_reg_cfg);

    event_cfg.event_id = EVENT_ID_SOUND_DOSE_MEL_VALUES;
    event_cfg.event_config_payload_size = 0;

    /* De-register for the sound dose events. */
    event_cfg.is_register = 0;
    ret = SessionAlsaUtils::registerMixerEvent(virtMixer, pcmDevIdRx.at(0), backEndName.c_str(),
                                               MODULE_SOUND_DOSE, (void *)&event_cfg, payloadSize);
    if (ret) {
        PAL_ERR(LOG_TAG, "registerMixerEvent failed");
    }
    pcm_close(rxPcm);
    rxPcm = NULL;

exit:
    PAL_DBG(LOG_TAG, "Exit");
}

void SoundDoseUtility::getSoundDoseMelValues() {
    PAL_DBG(LOG_TAG, "%p getSoundDoseMelValues for %d", this, toString(&mPalDevice).c_str());
    int ret = 0;
    uint32_t miid = 0;
    const char *getParamControl = "getParam";
    char *pcmDeviceName = NULL;
    uint8_t *payload = NULL;
    size_t payloadSize = 0;
    struct mixer_ctl *ctl;
    std::ostringstream cntrlName;
    std::string backendName;
    param_id_sounddose_flush_mel_values remainingMels;
    PayloadBuilder *builder = new PayloadBuilder();
    param_id_sounddose_flush_mel_values *remainingMelValues;
    uint32_t *eventData = nullptr;
    pal_global_callback_event_t event;

    PAL_DBG(LOG_TAG, " Enter %s", __func__);
    pcmDeviceName = mResourceManager->getDeviceNameFromID(pcmDevIdRx.at(0));
    if (pcmDeviceName) {
        cntrlName << pcmDeviceName << " " << getParamControl;
    } else {
        PAL_ERR(LOG_TAG, "Error: %d Unable to get Device name\n", -EINVAL);
        goto exit;
    }

    ctl = mixer_get_ctl_by_name(virtMixer, cntrlName.str().data());
    if (!ctl) {
        ret = -ENOENT;
        PAL_ERR(LOG_TAG, "Error: %d Invalid mixer control: %s\n", ret, cntrlName.str().data());
        goto exit;
    }

    mResourceManager->getBackendName(PAL_DEVICE_OUT_SOUND_DOSE, backendName);
    if (!strlen(backendName.c_str())) {
        ret = -ENOENT;
        PAL_ERR(LOG_TAG, "Error: %d Failed to get SoundDose backend name\n", ret);
        goto exit;
    }

    ret = SessionAlsaUtils::getModuleInstanceId(virtMixer, pcmDevIdRx.at(0), backendName.c_str(),
                                                MODULE_SOUND_DOSE, &miid);

    if (ret != 0) {
        PAL_ERR(LOG_TAG, "Error: %d Failed to get tag info %x", ret, MODULE_SOUND_DOSE);
        goto exit;
    }

    ret = builder->payloadSoundDoseInfo(&payload, &payloadSize, miid);
    if (ret || !payload) {
        PAL_ERR(LOG_TAG, "payloadSoundDoseInfo failed, ret = %d", ret);
        ret = -ENOMEM;
        goto exit;
    }

    ret = mixer_ctl_set_array(ctl, payload, payloadSize);
    if (ret != 0) {
        PAL_ERR(LOG_TAG, "Set failed ret = %d", ret);
        goto exit;
    }

    memset(payload, 0, payloadSize);

    ret = mixer_ctl_get_array(ctl, payload, payloadSize);
    if (ret != 0) {
        PAL_ERR(LOG_TAG, "Get failed ret = %d", ret);
    } else {
        // Parse payload
        PAL_DBG(LOG_TAG, "%s Received Sound Dose getParam", __func__);

        remainingMelValues =
                (param_id_sounddose_flush_mel_values *)(payload +
                                                        sizeof(struct apm_module_param_data_t));

        std::unique_ptr<pal_sound_dose_info_t> palSoundDoseInfo =
                std::make_unique<pal_sound_dose_info_t>();
        if (!palSoundDoseInfo) {
            PAL_ERR(LOG_TAG, "Memory allocation failed!\n");
            return;
        }
        palSoundDoseInfo->num_mel_values = remainingMelValues->num_mel_values;
        palSoundDoseInfo->device = getPalDevice();

        uint32_t offset_to_mel_values = sizeof(uint32_t);
        uint32_t offset_to_timestamp_values =
                offset_to_mel_values + sizeof(float) * palSoundDoseInfo->num_mel_values;
        float *mel_values_base = reinterpret_cast<float *>(
                reinterpret_cast<uint8_t *>(remainingMelValues) + offset_to_mel_values);
        uint32_t *timestamp_base = reinterpret_cast<uint32_t *>(
                reinterpret_cast<uint8_t *>(remainingMelValues) + offset_to_timestamp_values);

        struct timespec currentTime;
        clock_gettime(CLOCK_MONOTONIC, &currentTime);
        uint64_t currentTimeMicroseconds =
                currentTime.tv_sec * 1000000LL + currentTime.tv_nsec / 1000;

        // Calculate the time delta
        uint64_t timeDeltaMicroseconds =
                currentTimeMicroseconds - timestamp_base[palSoundDoseInfo->num_mel_values - 1];
        for (int i = 0; i < remainingMels.num_mel_values; i++) {
            palSoundDoseInfo->mel_values[i] = mel_values_base[i];
            uint32_t lsb_time = timestamp_base[2 * i];
            uint32_t msb_time = timestamp_base[2 * i + 1];
            palSoundDoseInfo->timestamp[i] = (static_cast<uint64_t>(msb_time) << 32) | lsb_time;
            palSoundDoseInfo->timestamp[i] =
                    (palSoundDoseInfo->timestamp[i] + timeDeltaMicroseconds) / 1000000LL;
        }

        auto callback = mResourceManager->getCallback();
        if (callback) {
            PAL_DBG(LOG_TAG, "Notifying client about sound dose global cb %pK", callback);
            eventData = reinterpret_cast<uint32_t *>(palSoundDoseInfo.get());
            event = PAL_SOUND_DOSE_INFO;
            callback(event, eventData, mResourceManager->getCookie());
        }
    }

    if (payload) {
        delete payload;
    }

exit:
    if (builder) {
        delete builder;
    }

    PAL_DBG(LOG_TAG, " Exit %s", __func__);
}

void SoundDoseUtility::handleSoundDoseCallback(uint64_t hdl, uint32_t event_id, void *data,
                                               uint32_t event_size) {
    uint32_t *eventData = nullptr;
    pal_global_callback_event_t event;
    event_id_sound_dose_mel_values *event_data = nullptr;

    PAL_DBG(LOG_TAG, "Got event from DSP %x", event_id);

    if (event_id != EVENT_ID_SOUND_DOSE_MEL_VALUES) {
        PAL_ERR(LOG_TAG, "Not sound dose mel event");
        return;
    }

    if (data == nullptr) {
        PAL_ERR(LOG_TAG, "data is null");
        return;
    }

    SoundDoseUtility *sounddoseInstance = reinterpret_cast<SoundDoseUtility *>(hdl);
    event_data = static_cast<event_id_sound_dose_mel_values *>(data);
    // use unique_ptr for pal_info
    std::unique_ptr<pal_sound_dose_info_t> palSoundDoseInfo =
            std::make_unique<pal_sound_dose_info_t>();
    if (!palSoundDoseInfo) {
        PAL_ERR(LOG_TAG, "Memory allocation failed for palSoundDoseInfo");
        return;
    }

    palSoundDoseInfo->is_momentary_exposure_warning = event_data->is_momentary_exposure_raised;
    palSoundDoseInfo->num_mel_values = event_data->num_mel_values;
    palSoundDoseInfo->device = sounddoseInstance->getPalDevice();

    uint32_t offset_to_mel_values = sizeof(uint32_t) + sizeof(uint32_t);
    uint32_t offset_to_timestamp_values =
            offset_to_mel_values + sizeof(float) * palSoundDoseInfo->num_mel_values;
    float *mel_values_base =
            reinterpret_cast<float *>(reinterpret_cast<uint8_t *>(data) + offset_to_mel_values);
    uint32_t *timestamp_base = reinterpret_cast<uint32_t *>(reinterpret_cast<uint8_t *>(data) +
                                                            offset_to_timestamp_values);

    /*Parse payload based on momentary exposure warning or not. */
    if (palSoundDoseInfo->is_momentary_exposure_warning) {
        // if momentary exposure warning then only 1 MEL value is raised
        if (event_data->num_mel_values > 0) {
            palSoundDoseInfo->mel_values[0] = mel_values_base[0];
        } else {
            PAL_ERR(LOG_TAG, "Momentary warning raised, but num mel values = 0");
        }
    } else {
        PAL_DBG(LOG_TAG, "Mel values num =%d", palSoundDoseInfo->num_mel_values);
        struct timespec currentTime;
        clock_gettime(CLOCK_MONOTONIC, &currentTime);
        uint64_t currentTimeMicroseconds =
                currentTime.tv_sec * 1000000LL + currentTime.tv_nsec / 1000;

        // Calculate the time delta
        uint64_t timeDeltaMicroseconds =
                currentTimeMicroseconds - timestamp_base[event_data->num_mel_values - 1];
        for (uint32_t i = 0; i < event_data->num_mel_values && i < 10; i++) {
            palSoundDoseInfo->mel_values[i] = mel_values_base[i];
            PAL_VERBOSE(LOG_TAG, "MEL value: %f %s", palSoundDoseInfo->mel_values[i], __func__);
            uint32_t lsb_time = timestamp_base[2 * i];
            uint32_t msb_time = timestamp_base[2 * i + 1];
            palSoundDoseInfo->timestamp[i] = (static_cast<uint64_t>(msb_time) << 32) | lsb_time;
            palSoundDoseInfo->timestamp[i] =
                    (palSoundDoseInfo->timestamp[i] + timeDeltaMicroseconds) / 1000000LL;
            PAL_VERBOSE(LOG_TAG, "TimeStamp value: %llu %s", palSoundDoseInfo->timestamp[i],
                        __func__);
        }
        // TODO: if mel values > 10 then error.
    }

    // Invoke global cb
    auto callback = sounddoseInstance->getCallback();
    if (callback) {
        PAL_DBG(LOG_TAG, "Notifying client about sound dose global cb %pK", callback);
        eventData = reinterpret_cast<uint32_t *>(palSoundDoseInfo.get());
        event = PAL_SOUND_DOSE_INFO;
        callback(event, eventData, sounddoseInstance->getCookie());
    }
    PAL_DBG(LOG_TAG, " Exit %s", __func__);
}

void SoundDoseUtility::startSimulation() {
    if (!mUseSimulation) {
        return; // don't use Simulation
    }

    PAL_INFO(LOG_TAG, " Enter: %p refcount %d", this, mRefCount.load());

    if (mRefCount.fetch_add(1) == 0) {
        mSimulationRunning = true;
        PAL_INFO(LOG_TAG, " create simulationThreadLoop %p ", this);
        mSimulationThread = std::thread(&SoundDoseUtility::simulationThreadLoop, this);
    } else {
        PAL_INFO(LOG_TAG, " thread is already running %p refCount %d ", this, mRefCount.load());
    }
}

void SoundDoseUtility::stopSimulation() {
    if (!mSimulationRunning) {
        return;
    }

    PAL_INFO(LOG_TAG, " Enter: %p refcount %d", this, mRefCount.load());

    if (mRefCount.fetch_sub(1) == 1) {
        mSimulationRunning = false;
        if (mSimulationThread.joinable()) {
            mSimulationThread.join();
        }
    } else {
        PAL_INFO(LOG_TAG, " refcount %d is not zero yet %p", mRefCount.load(), this);
    }
}

void SoundDoseUtility::simulationThreadLoop() {
    int count = 0;
    while (mSimulationRunning) {
        pal_sound_dose_info_t palSoundDoseInfo;
        palSoundDoseInfo.device = mPalDevice;
        // random value each 5th value set exposure warning
        palSoundDoseInfo.is_momentary_exposure_warning = (count % 5 == 0);
        count++;
        palSoundDoseInfo.num_mel_values = PAL_MAX_SOUND_DOSE_VALUES;

        for (uint32_t i = 0; i < PAL_MAX_SOUND_DOSE_VALUES; ++i) {
            palSoundDoseInfo.mel_values[i] = static_cast<float>(i + 20);
            palSoundDoseInfo.timestamp[i] =
                    std::chrono::system_clock::now().time_since_epoch().count();
        }

        // Simulate event generation (replace with actual callback if needed)
        auto callback = mResourceManager->getCallback();

        if (callback) {
            PAL_INFO(LOG_TAG, "Notifying client about sounddose global cb %pK device %s", callback,
                     toString(&mPalDevice).c_str());
            uint32_t *eventData = reinterpret_cast<uint32_t *>(&palSoundDoseInfo);
            pal_global_callback_event_t event = PAL_SOUND_DOSE_INFO;
            callback(event, eventData, mResourceManager->getCookie());
        } else {
            PAL_ERR(LOG_TAG, "no valid callback found");
        }

        // generate event every 2 seconds for test
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}
