/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

//File with Sound Dose related support in PAL.
#include "SoundDoseUtility.h"
#include "ResourceManager.h"
#include "PayloadBuilder.h"
#include "sound_dose_api.h"
#include "SessionAgm.h"
#include "SessionAlsaUtils.h"
#include "kvh2xml.h"
#include <agm/agm_api.h>
#include <ctime>

#define DEFAULT_PERIOD_SIZE 256
#define DEFAULT_PERIOD_COUNT 4

std::shared_ptr<ResourceManager> SoundDoseUtility::rm = NULL;

SoundDoseUtility::SoundDoseUtility(Device *devObj, pal_device_id_t deviceId) {
    //TODO: add appropriate initialization.
    int status = 0;
    mAssociatedDevId = deviceId;
    mDevObj = devObj;

    rm = ResourceManager::getInstance();
    if (!rm) {
        PAL_ERR(LOG_TAG,"Resource manager instance unavailable");
        return;
    }
    status = rm->getVirtualAudioMixer(&virtMixer);
    if (status) {
        PAL_ERR(LOG_TAG,"virt mixer error %d", status);
    }
}

SoundDoseUtility::~SoundDoseUtility() {
    //TODO:handling for dynamic memory
}

int SoundDoseUtility::startSoundDoseComputation() {
    // Logic to start dosage computation for the device
    //Call pcm_open, register for event and pcm_start.
    int ret = 0, dir = RX_HOSTLESS;
    char mSndDeviceName[128] = {0};
    uint32_t devicePropId[] = {0x08000010, 1, 0x2};
    struct pal_device device;
    struct pal_device_info deviceSoundDose;
    struct pal_stream_attributes sAttr;
    struct pal_channel_info ch_info;
    struct pcm_config config;
    std::vector <std::pair<int, int>> keyVector;
    std::vector <std::pair<int, int>> calVector;
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

    //setup dummy dev tx graph with sound dose module.
    device.id = PAL_DEVICE_OUT_SOUND_DOSE;
	strlcpy(mSndDeviceName, "dummy-dev", DEVICE_NAME_MAX_SIZE);
    /* Configure device attributes TODO: check the config? */
    ch_info.channels = CHANNELS_1;
    ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FL;
    device.config.ch_info = ch_info;
    device.config.sample_rate = 48000;
    device.config.bit_width = BITWIDTH_16;
    device.config.aud_fmt_id = PAL_AUDIO_FMT_DEFAULT_COMPRESSED;

    ret = rm->getAudioRoute(&audioRoute);
    if (0 != ret) {
        PAL_ERR(LOG_TAG, "Failed to get the audio_route address status %d", ret);
        goto exit;
    }

    rm->getBackendName(device.id, backEndName);
    if (!strlen(backEndName.c_str())) {
        PAL_ERR(LOG_TAG, "Failed to obtain tx backend name for %d", device.id);
        goto exit;
    }

    ret = PayloadBuilder::getDeviceKV(device.id, keyVector);
    if (0 != ret) {
        PAL_ERR(LOG_TAG, "Failed to obtain device KV for %d", device.id);
        goto exit;
    }

    SessionAlsaUtils::getAgmMetaData(keyVector, calVector,
                (struct prop_data *)devicePropId, deviceMetaData);
    if (!deviceMetaData.size) {
       PAL_ERR(LOG_TAG, "SoundDose device metadata is zero");
       ret = -ENOMEM;
       goto exit;
    }
    connectCtrlNameBe<< backEndName << " metadata";
    beMetaDataMixerCtrl = mixer_get_ctl_by_name(virtMixer,
                                    connectCtrlNameBe.str().data());
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
    }
    else {
        PAL_ERR(LOG_TAG, "Device Metadata not set for RX path");
        ret = -EINVAL;
        goto exit;
    }

    ret = mDevObj->setMediaConfig(rm, backEndName, &device);
    if (ret) {
        PAL_ERR(LOG_TAG, "setDeviceMediaConfig for sound dose device failed");
        goto exit;
    }

    /* Retrieve Hostless PCM device id */
    sAttr.type = PAL_STREAM_LOW_LATENCY;
    sAttr.direction = PAL_AUDIO_OUTPUT;
    dir = RX_HOSTLESS;
    flags = PCM_OUT;
    pcmDevIdRx = rm->allocateFrontEndIds(sAttr, dir);
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
        PAL_ERR(LOG_TAG, "Mixer control %s set with %s failed: %d",
        connectCtrlName.str().data(), backEndName.c_str(), ret);
        goto free_fe;
    }


    //TODO: set PARAM_ID_SOUND_DOSE_MEL_EVENTS_CONFIG for time between 2 mel values
    //TODO: set PARAM_ID_SOUND_DOSE_RS2_UPPER_BOUND if configured from fwk.
    config.rate = 48000;
    config.format = PCM_FORMAT_S16_LE;
    config.channels = CHANNELS_1;
    config.period_size = DEFAULT_PERIOD_SIZE;
    config.period_count = DEFAULT_PERIOD_COUNT;
    config.start_threshold = 0;
    config.stop_threshold = INT_MAX;
    config.silence_threshold = 0;

    rxPcm = pcm_open(rm->getVirtualSndCard(), pcmDevIdRx.at(0), flags, &config);
    if (!rxPcm) {
        PAL_ERR(LOG_TAG, "rxPcm open failed");
        goto free_fe;
    }

    if (!pcm_is_ready(rxPcm)) {
        PAL_ERR(LOG_TAG, "rxPcm open not ready");
        goto err_pcm_open;
    }

    PAL_DBG(LOG_TAG, "registering events for SoundDose module");
    payloadSize = sizeof(struct agm_event_reg_cfg);

    /* Register for EVENT_ID_SOUND_DOSE_MEL_VALUES. */
    event_cfg.event_id = EVENT_ID_SOUND_DOSE_MEL_VALUES;
    event_cfg.event_config_payload_size = 0;
    event_cfg.is_register = 1;

    ret = SessionAlsaUtils::registerMixerEvent(virtMixer, pcmDevIdRx.at(0),
                      backEndName.c_str(), MODULE_SOUND_DOSE, (void *)&event_cfg,
                      payloadSize);
    if (ret) {
        PAL_ERR(LOG_TAG, "Unable to register event to DSP, for Sound Dose event");
        // Fatal Error. Sound Dose Computation Won't work
        goto err_pcm_open;
    }

    // Register to mixtureControlEvents and wait for the MEL values
    ret = rm->registerMixerEventCallback(pcmDevIdRx, sessionCb, (uint64_t)this, true);
    if (ret != 0) {
        PAL_ERR(LOG_TAG, "Failed to register callback to rm");
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
err_pcm_open :
    if (rxPcm) {
        pcm_close(rxPcm);
        disableDevice(audioRoute, mSndDeviceName);
        rxPcm = NULL;
    }

free_fe:
    rm->freeFrontEndIds(pcmDevIdRx, sAttr, dir);
exit:

    PAL_DBG(LOG_TAG,"Exit");
    return ret;

}

void SoundDoseUtility::getSoundDoseMelValues()
{
    int ret = 0;
    uint32_t miid = 0;
    const char *getParamControl = "getParam";
    char *pcmDeviceName = NULL;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    struct mixer_ctl *ctl;
    std::ostringstream cntrlName;
    std::string backendName;
    param_id_sounddose_flush_mel_values remainingMels;
    PayloadBuilder* builder = new PayloadBuilder();
    param_id_sounddose_flush_mel_values* remainingMelValues;
    uint32_t* eventData = nullptr;
    pal_global_callback_event_t event;

    PAL_DBG(LOG_TAG, " Enter %s",__func__);
    pcmDeviceName = rm->getDeviceNameFromID(pcmDevIdRx.at(0));
    if (pcmDeviceName) {
        cntrlName<<pcmDeviceName<<" "<<getParamControl;
    } else {
        PAL_ERR(LOG_TAG, "Error: %d Unable to get Device name\n", -EINVAL);
        goto exit;
    }

    ctl = mixer_get_ctl_by_name(virtMixer, cntrlName.str().data());
    if (!ctl) {
        ret = -ENOENT;
        PAL_ERR(LOG_TAG, "Error: %d Invalid mixer control: %s\n", ret,cntrlName.str().data());
        goto exit;
    }

    rm->getBackendName(PAL_DEVICE_OUT_SOUND_DOSE, backendName);
    if (!strlen(backendName.c_str())) {
        ret = -ENOENT;
        PAL_ERR(LOG_TAG, "Error: %d Failed to get SoundDose backend name\n", ret);
        goto exit;
    }

    ret =  SessionAlsaUtils::getModuleInstanceId(virtMixer, pcmDevIdRx.at(0),
                    backendName.c_str(), MODULE_SOUND_DOSE, &miid);

    if (ret != 0) {
        PAL_ERR(LOG_TAG, "Error: %d Failed to get tag info %x", ret, MODULE_SOUND_DOSE);
        goto exit;
    }

    ret = builder->payloadSoundDoseInfo(&payload,&payloadSize, miid);
    if (ret || !payload) {
        PAL_ERR(LOG_TAG, "Failed to construct SoundDose payload, ret = %d", ret);
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
        //Parse payload
        PAL_DBG(LOG_TAG, "%s Received Sound Dose getParam",__func__);

        remainingMelValues = (param_id_sounddose_flush_mel_values *) (payload+
                             sizeof(struct apm_module_param_data_t));

        std::unique_ptr<pal_sound_dose_info_t> palSoundDoseInfo = std::make_unique<pal_sound_dose_info_t>();
        if (!palSoundDoseInfo) {
            PAL_ERR(LOG_TAG,"Memory allocation failed!\n");
            return;
        }
        palSoundDoseInfo->num_mel_values = remainingMelValues->num_mel_values;
        palSoundDoseInfo->id = this->mAssociatedDevId;
        uint32_t offset_to_mel_values = sizeof(uint32_t);
        uint32_t offset_to_timestamp_values = offset_to_mel_values + sizeof(float) * palSoundDoseInfo->num_mel_values;
        float* mel_values_base = reinterpret_cast<float*>(reinterpret_cast<uint8_t*>(remainingMelValues) + offset_to_mel_values);
        uint32_t* timestamp_base = reinterpret_cast<uint32_t*>(reinterpret_cast<uint8_t*>(remainingMelValues) + offset_to_timestamp_values);

        struct timespec currentTime;
        clock_gettime(CLOCK_MONOTONIC, &currentTime);
        uint64_t currentTimeMicroseconds = currentTime.tv_sec * 1000000LL + currentTime.tv_nsec / 1000;

        // Calculate the time delta
        uint64_t timeDeltaMicroseconds = currentTimeMicroseconds - timestamp_base[palSoundDoseInfo->num_mel_values - 1];
        for (int i = 0; i < remainingMels.num_mel_values; i++) {
            palSoundDoseInfo->mel_values[i] =  mel_values_base[i];
            uint32_t lsb_time = timestamp_base[2*i];
            uint32_t msb_time = timestamp_base[2*i+1];
            palSoundDoseInfo->timestamp[i] = (static_cast<uint64_t>(msb_time) << 32) | lsb_time;
            palSoundDoseInfo->timestamp[i] = (palSoundDoseInfo->timestamp[i]+ timeDeltaMicroseconds)  / 1000000LL;
        }

        if (rm->globalCb) {
            PAL_DBG(LOG_TAG, "Notifying client about sound dose global cb %pK",
                              rm->globalCb);
            eventData = reinterpret_cast<uint32_t*>(palSoundDoseInfo.get());
            event = PAL_SOUND_DOSE_INFO;
            rm->globalCb(event, eventData, rm->cookie);
        }
    }

    if (payload) {
        delete payload;
        payloadSize = 0;
        payload = NULL;
    }

exit:
    if (builder) {
        delete builder;
        builder = NULL;
    }

    PAL_DBG(LOG_TAG, " Exit %s",__func__);
}

void SoundDoseUtility::stopSoundDoseComputation() {
    // Logic to stop dosage computation for the device

    struct pal_stream_attributes sAttr;
    struct pal_channel_info ch_info;
    int ret = 0, dir;
    struct pal_device device;
    std::string backEndName;
    struct agm_event_reg_cfg event_cfg;
    size_t payloadSize = 0;
    //setup dummy dev tx graph with sound dose module.
    device.id = PAL_DEVICE_OUT_SOUND_DOSE;

    /* Configure device attributes TODO: check the config? */
    ch_info.channels = CHANNELS_1;
    ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FL;
    device.config.ch_info = ch_info;
    device.config.bit_width = BITWIDTH_16;
    device.config.aud_fmt_id = PAL_AUDIO_FMT_DEFAULT_COMPRESSED;

    rm->getBackendName(device.id, backEndName);
    if (!strlen(backEndName.c_str())) {
        PAL_ERR(LOG_TAG, "Failed to obtain tx backend name for %d", device.id);
        goto exit;
    }

    //add mutex if required

    if (!rxPcm) {
        PAL_ERR(LOG_TAG,"rxPcm is null");
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

    rm->freeFrontEndIds(pcmDevIdRx, sAttr,dir);
    payloadSize = sizeof(struct agm_event_reg_cfg);

    event_cfg.event_id = EVENT_ID_SOUND_DOSE_MEL_VALUES;
    event_cfg.event_config_payload_size = 0;

    /* De-register for the sound dose events. */
    event_cfg.is_register = 0;
    ret = SessionAlsaUtils::registerMixerEvent(virtMixer, pcmDevIdRx.at(0),
                        backEndName.c_str(), MODULE_SOUND_DOSE, (void *)&event_cfg,
                        payloadSize);
    if (ret) {
            PAL_ERR(LOG_TAG, "Unable to deregister event to DSP");
    }
    pcm_close(rxPcm);
    rxPcm = NULL;

exit:
    PAL_DBG(LOG_TAG, "Exit");
}

void SoundDoseUtility::handleSoundDoseCallback(uint64_t hdl, uint32_t event_id,
                                        void *data, uint32_t event_size)
{

    uint32_t* eventData = nullptr;
    pal_global_callback_event_t event;
    event_id_sound_dose_mel_values *event_data = nullptr;
    SoundDoseUtility *sd = nullptr;

    PAL_DBG(LOG_TAG, "Got event from DSP %x", event_id);

    if (event_id != EVENT_ID_SOUND_DOSE_MEL_VALUES) {
        PAL_ERR(LOG_TAG,"Not sound dose mel event!\n");
        return;
    }

    if (data == nullptr) {
        PAL_ERR(LOG_TAG,"data is null!\n");
        return;
    }

    sd = reinterpret_cast<SoundDoseUtility*>(hdl);
    event_data = static_cast<event_id_sound_dose_mel_values*>(data);
    //use unique_ptr for pal_info
    std::unique_ptr<pal_sound_dose_info_t> palSoundDoseInfo = std::make_unique<pal_sound_dose_info_t>();
    if (!palSoundDoseInfo) {
        PAL_ERR(LOG_TAG,"Memory allocation failed!\n");
        return;
    }

    palSoundDoseInfo->is_momentary_exposure_warning = event_data->is_momentary_exposure_raised;
    palSoundDoseInfo->num_mel_values = event_data->num_mel_values;
    palSoundDoseInfo->id = sd->mAssociatedDevId;

    uint32_t offset_to_mel_values = sizeof(uint32_t) + sizeof(uint32_t);
    uint32_t offset_to_timestamp_values = offset_to_mel_values + sizeof(float) * palSoundDoseInfo->num_mel_values;
    float* mel_values_base = reinterpret_cast<float*>(reinterpret_cast<uint8_t*>(data) + offset_to_mel_values);
    uint32_t* timestamp_base = reinterpret_cast<uint32_t*>(reinterpret_cast<uint8_t*>(data) + offset_to_timestamp_values);

    /*Parse payload based on momentary exposure warning or not. */
    if (palSoundDoseInfo->is_momentary_exposure_warning) {
        //if momentary exposure warning then only 1 MEL value is raised
        if (event_data->num_mel_values > 0) {
            palSoundDoseInfo->mel_values[0] = mel_values_base[0];
        } else {
            PAL_ERR(LOG_TAG,"Momentary warning raised, but num mel values = 0 \n");
        }
    } else {
        PAL_DBG(LOG_TAG, " Mel values num =%d",palSoundDoseInfo->num_mel_values);
        struct timespec currentTime;
        clock_gettime(CLOCK_MONOTONIC, &currentTime);
        uint64_t currentTimeMicroseconds = currentTime.tv_sec * 1000000LL + currentTime.tv_nsec / 1000;

        // Calculate the time delta
        uint64_t timeDeltaMicroseconds = currentTimeMicroseconds - timestamp_base[event_data->num_mel_values - 1];
        for (uint32_t i = 0; i<event_data->num_mel_values && i < 10; i++) {
            palSoundDoseInfo->mel_values[i] =  mel_values_base[i];
            PAL_DBG(LOG_TAG, "MEL Value: %f %s", palSoundDoseInfo->mel_values[i] , __func__);
            uint32_t lsb_time = timestamp_base[2*i];
            uint32_t msb_time = timestamp_base[2*i+1];
            palSoundDoseInfo->timestamp[i] = (static_cast<uint64_t>(msb_time) << 32) | lsb_time;
            palSoundDoseInfo->timestamp[i] = (palSoundDoseInfo->timestamp[i]+ timeDeltaMicroseconds)  / 1000000LL;
            PAL_DBG(LOG_TAG, "TimeStamp Value: %llu %s", palSoundDoseInfo->timestamp[i] , __func__);
        }
        //TODO: if mel values > 10 then error.
    }

    //Invoke global cb
    if (rm->globalCb) {
        PAL_DBG(LOG_TAG, "Notifying client about sound dose global cb %pK",
                              rm->globalCb);
        eventData = reinterpret_cast<uint32_t*>(palSoundDoseInfo.get());
        event = PAL_SOUND_DOSE_INFO;
        rm->globalCb(event, eventData, rm->cookie);
    }
    PAL_DBG(LOG_TAG, " Exit %s",__func__);
}


