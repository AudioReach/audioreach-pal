/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
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
 *
 * Copyright (c) 2022-2025, Qualcomm Innovation Center, Inc. All rights reserved.
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

#define LOG_TAG "PAL: SpeakerProtection"


#include "SpeakerProtection.h"
#include "PalAudioRoute.h"
#include "ResourceManager.h"
#include "SessionAlsaUtils.h"
#include "kvh2xml.h"
#include <agm/agm_api.h>
#include "SessionAR.h"

#include<fstream>
#include<sstream>

std::thread SpeakerProtection::mCalThread;
std::condition_variable SpeakerProtection::cv;
std::mutex SpeakerProtection::cvMutex;

bool SpeakerProtection::isSpkrInUse;
bool SpeakerProtection::calThrdCreated;
bool SpeakerProtection::isDynamicCalTriggered = false;
struct timespec SpeakerProtection::spkrLastTimeUsed;
struct mixer *SpeakerProtection::virtMixer;
struct mixer *SpeakerProtection::hwMixer;
speaker_prot_cal_state SpeakerProtection::spkrCalState;
struct pcm * SpeakerProtection::rxPcm = NULL;
struct pcm * SpeakerProtection::txPcm = NULL;
struct pcm * SpeakerProtection::cpsPcm = NULL;
struct param_id_sp_th_vi_calib_res_per_spkr_cfg_param_t * SpeakerProtection::callback_data;
int SpeakerProtection::numberOfChannels;
struct pal_device_info SpeakerProtection::vi_device;
struct pal_device_info SpeakerProtection::cps_device;
int SpeakerProtection::calibrationCallbackStatus;
int SpeakerProtection::numberOfRequest;
bool SpeakerProtection::mDspCallbackRcvd;
std::shared_ptr<Device> SpeakerFeedback::obj = nullptr;
int SpeakerFeedback::numSpeaker;

std::string getDefaultSpkrTempCtrl(uint8_t spkr_pos)
{
    switch(spkr_pos)
    {
        case SPKR_LEFT:
            return std::string(SPKR_LEFT_WSA_TEMP);
        break;
        case SPKR_RIGHT:
            [[fallthrough]];
        default:
            return std::string(SPKR_RIGHT_WSA_TEMP);
    }
}

int SpeakerProtection::updateVICustomPayload(void *payload, size_t size)
{
    if (!viCustomPayloadSize) {
        viCustomPayload = calloc(1, size);
    } else {
        viCustomPayload = realloc(viCustomPayload, viCustomPayloadSize + size);
    }

    if (!viCustomPayload) {
        PAL_ERR(LOG_TAG, "failed to allocate memory for custom payload for VI");
        return -ENOMEM;
    }

    memcpy((uint8_t *)viCustomPayload + viCustomPayloadSize, payload, size);
    viCustomPayloadSize += size;
    PAL_INFO(LOG_TAG, "viCustomPayloadSize = %zu", viCustomPayloadSize);
    return 0;
}

/* Function to check if Speaker is in use or not.
 * It returns the time as well for which speaker is not in use.
 */
bool SpeakerProtection::isSpeakerInUse(unsigned long *sec)
{
    struct timespec temp;
    PAL_DBG(LOG_TAG, "Enter");

    if (!sec) {
        PAL_ERR(LOG_TAG, "Improper argument");
        return false;
    }

    if (isSpkrInUse) {
        PAL_INFO(LOG_TAG, "Speaker in use");
        *sec = 0;
        return true;
    } else {
        PAL_INFO(LOG_TAG, "Speaker not in use");
        clock_gettime(CLOCK_BOOTTIME, &temp);
        *sec = temp.tv_sec - spkrLastTimeUsed.tv_sec;
    }

    PAL_DBG(LOG_TAG, "Idle time %ld", *sec);

    return false;
}

/* Function to set status of speaker */
void SpeakerProtection::spkrProtSetSpkrStatus(bool enable)
{
    PAL_DBG(LOG_TAG, "Enter");

    if (enable)
        isSpkrInUse = true;
    else {
        isSpkrInUse = false;
        clock_gettime(CLOCK_BOOTTIME, &spkrLastTimeUsed);
        PAL_INFO(LOG_TAG, "Speaker used last time %ld", spkrLastTimeUsed.tv_sec);
    }

    PAL_DBG(LOG_TAG, "Exit");
}

/* Wait function for WAKEUP_MIN_IDLE_CHECK  */
void SpeakerProtection::spkrCalibrateWait()
{
    std::unique_lock<std::mutex> lock(cvMutex);
    cv.wait_for(lock,
            std::chrono::milliseconds(WAKEUP_MIN_IDLE_CHECK));
}

// Callback from DSP for Ressistance value
void SpeakerProtection::handleSPCallback (uint64_t hdl __unused, uint32_t event_id,
                                            void *event_data, uint32_t event_size)
{
    param_id_sp_th_vi_calib_res_per_spkr_cfg_param_t *param_data = nullptr;
    param_id_sp_vi_spkr_diag_getpkt_param_t *diag_data = nullptr;
    bool calSuccess = true, calFailure = false;

    PAL_DBG(LOG_TAG, "Got event from DSP %x", event_id);

    switch(event_id) {
    case EVENT_ID_VI_PER_SPKR_CALIBRATION:
        // Received callback for Calibration state
        param_data = (param_id_sp_th_vi_calib_res_per_spkr_cfg_param_t *) event_data;

        for (int i = 0; i < param_data->num_ch; i++) {
            PAL_DBG(LOG_TAG, "Calibration state %d for Spkr %d", param_data->cali_param[i].state, i+1);
            if (param_data->cali_param[i].state != CALIBRATION_STATUS_SUCCESS)
                calSuccess = false;
            if (param_data->cali_param[i].state == CALIBRATION_STATUS_FAILURE ||
                param_data->cali_param[i].state == CALIBRATION_STATUS_IVLOW) {
                PAL_ERR(LOG_TAG, "Calibration failed for Speaker no %d", i+1);
                calFailure = true;
            }
        }

        if (calSuccess) {
            PAL_DBG(LOG_TAG, "Calibration is successful");
            callback_data = (param_id_sp_th_vi_calib_res_per_spkr_cfg_param_t *) calloc(1, event_size);
            if (!callback_data) {
                PAL_ERR(LOG_TAG, "Unable to allocate memory");
            }
            else {
                callback_data->num_ch = param_data->num_ch;
                calibrationCallbackStatus = CALIBRATION_STATUS_SUCCESS;
                for (int i = 0; i < param_data->num_ch; i++)
                    callback_data->cali_param[i].r0_cali_q24 = param_data->cali_param[i].r0_cali_q24;
            }
            mDspCallbackRcvd = true;
            cv.notify_all();
        } else {
                // Restart the calibration and abort current run.
                if (calFailure) {
                    mDspCallbackRcvd = true;
                    calibrationCallbackStatus = CALIBRATION_STATUS_FAILURE;
                    cv.notify_all();
                }
        }
        break;
    case EVENT_ID_SPv5_SPEAKER_DIAGNOSTICS:
        struct mixer_ctl *ctl;

        ctl = mixer_get_ctl_by_name(hwMixer, SPKR_LEFT_WSA_DC_DET);
        diag_data = (param_id_sp_vi_spkr_diag_getpkt_param_t *) event_data;
        if (diag_data->num_ch == 1) {
                PAL_DBG(LOG_TAG, "Calibration state %d", diag_data->spkr_cond[0]);

                if (diag_data->spkr_cond[0] == SPKR_OVERTEMP)
                    PAL_ERR(LOG_TAG, "OVERTEMP detected on Spkr Right");

                if (diag_data->spkr_cond[0] == SPKR_DC) {
                    ctl = mixer_get_ctl_by_name(hwMixer, SPKR_RIGHT_WSA_DC_DET);
                    if (!ctl) {
                         PAL_ERR(LOG_TAG, "invalid mixer control for DC : %s", SPKR_RIGHT_WSA_DC_DET);
                         return;
                    }
                    mixer_ctl_set_value(ctl, 0, 1);
                    mixer_ctl_set_value(ctl, 0, 0);
                }
        } else {
                PAL_DBG(LOG_TAG, "Calibration state left %d, right %d", diag_data->spkr_cond[0],
                                  diag_data->spkr_cond[1]);

                 if (diag_data->spkr_cond[0] == SPKR_OVERTEMP)
                     PAL_ERR(LOG_TAG, "OVERTEMP detected on Spkr Left");

                 if (diag_data->spkr_cond[0] == SPKR_DC) {
                     ctl = mixer_get_ctl_by_name(hwMixer, SPKR_LEFT_WSA_DC_DET);
                     if (!ctl) {
                         PAL_ERR(LOG_TAG, "invalid mixer control for DC : %s", SPKR_LEFT_WSA_DC_DET);
                         goto spkr_right;
                     }
                     mixer_ctl_set_value(ctl, 0, 1);
                     mixer_ctl_set_value(ctl, 0, 0);
                 }
spkr_right:
                 if (diag_data->spkr_cond[1] == SPKR_OVERTEMP)
                     PAL_ERR(LOG_TAG, "OVERTEMP detected on Spkr Right");

                 if (diag_data->spkr_cond[1] == SPKR_DC) {
                     ctl = mixer_get_ctl_by_name(hwMixer, SPKR_RIGHT_WSA_DC_DET);
                     if (!ctl) {
                         PAL_ERR(LOG_TAG, "invalid mixer control for DC : %s", SPKR_RIGHT_WSA_DC_DET);
                         return;
                     }
                     mixer_ctl_set_value(ctl, 0, 1);
                     mixer_ctl_set_value(ctl, 0, 0);
                 }
        }
        break;
    default:
        PAL_ERR(LOG_TAG, "Unsupported event %x", event_id);
        break;
    }
}

int SpeakerProtection::getSpeakerTemperature(int spkr_pos)
{
    struct mixer_ctl *ctl;
    std::string mixer_ctl_name;
    int status = 0;
    /**
     * It is assumed that for Mono speakers only right speaker will be there.
     * Thus we will get the Temperature just for right speaker.
     * TODO: Get the channel from RM.xml
     */
    PAL_DBG(LOG_TAG, "Enter Speaker Get Temperature %d", spkr_pos);
    mixer_ctl_name = rm->getSpkrTempCtrl(spkr_pos);
    if (mixer_ctl_name.empty()) {
        PAL_DBG(LOG_TAG, "Using default mixer control");
        mixer_ctl_name = getDefaultSpkrTempCtrl(spkr_pos);
    }

    PAL_DBG(LOG_TAG, "audio_mixer %pK", hwMixer);

    ctl = mixer_get_ctl_by_name(hwMixer, mixer_ctl_name.c_str());
    if (!ctl) {
        PAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", mixer_ctl_name.c_str());
        status = -EINVAL;
        return status;
    }

    status = mixer_ctl_get_value(ctl, 0);

    PAL_DBG(LOG_TAG, "Exiting Speaker Get Temperature %d", status);

    return status;
}

void SpeakerProtection::disconnectFeandBe(std::vector<int> pcmDevIds,
                                         std::string backEndName) {

    std::ostringstream disconnectCtrlName;
    std::ostringstream disconnectCtrlNameBe;
    struct mixer_ctl *disconnectCtrl = NULL;
    struct mixer_ctl *beMetaDataMixerCtrl = nullptr;
    struct agmMetaData deviceMetaData(nullptr, 0);
    uint32_t devicePropId[] = { 0x08000010, 2, 0x2, 0x5 };
    std::vector <std::pair<int, int>> emptyKV;
    int ret = 0;

    emptyKV.clear();

    SessionAlsaUtils::getAgmMetaData(emptyKV, emptyKV,
                    (struct prop_data *)devicePropId, deviceMetaData);
    if (!deviceMetaData.size) {
        ret = -ENOMEM;
        PAL_ERR(LOG_TAG, "Error: %d, Device metadata is zero", ret);
        goto exit;
    }

    disconnectCtrlNameBe<< backEndName << " metadata";
    beMetaDataMixerCtrl = mixer_get_ctl_by_name(virtMixer, disconnectCtrlNameBe.str().data());
    if (!beMetaDataMixerCtrl) {
        ret = -EINVAL;
        PAL_ERR(LOG_TAG, "Error: %d, invalid mixer control %s", ret, backEndName.c_str());
        goto exit;
    }

    disconnectCtrlName << "PCM" << pcmDevIds.at(0) << " disconnect";
    disconnectCtrl = mixer_get_ctl_by_name(virtMixer, disconnectCtrlName.str().data());
    if (!disconnectCtrl) {
        ret = -EINVAL;
        PAL_ERR(LOG_TAG, "Error: %d, invalid mixer control: %s", ret, disconnectCtrlName.str().data());
        goto exit;
    }
    ret = mixer_ctl_set_enum_by_string(disconnectCtrl, backEndName.c_str());
    if (ret) {
        PAL_ERR(LOG_TAG, "Error: %d, Mixer control %s set with %s failed", ret,
        disconnectCtrlName.str().data(), backEndName.c_str());
    }

    if (deviceMetaData.size) {
        ret = mixer_ctl_set_array(beMetaDataMixerCtrl, (void *)deviceMetaData.buf,
                    deviceMetaData.size);
        free(deviceMetaData.buf);
        deviceMetaData.buf = nullptr;
    } else {
        PAL_ERR(LOG_TAG, "Error: %d, Device Metadata not cleaned up", ret);
        goto exit;
    }

exit:
    return;
}

/**
  * This function sets the temperature of each speakers.
  * Currently values are supported like:
  * spkerTempList[0] - Right Speaker Temperature
  * spkerTempList[1] - Left Speaker Temperature
  */
void SpeakerProtection::getSpeakerTemperatureList()
{
    int i = 0;
    int value;
    PAL_DBG(LOG_TAG, "Enter Speaker Get Temperature List");

    for(i = 0; i < numberOfChannels; i++) {
         value = getSpeakerTemperature(i);
         PAL_DBG(LOG_TAG, "Temperature %d ", value);
         spkerTempList[i] = value;
    }
    PAL_DBG(LOG_TAG, "Exit Speaker Get Temperature List");
}

void SpeakerProtection::spkrCalibrationThread()
{
    unsigned long sec = 0;
    bool proceed = false;
    int i;
    int retryCount = 0;

    while (!threadExit) {
        PAL_DBG(LOG_TAG, "Inside calibration while loop");
        proceed = false;
        if (isSpeakerInUse(&sec)) {
            PAL_DBG(LOG_TAG, "Speaker in use. Wait for proper time");
            spkrCalibrateWait();
            PAL_DBG(LOG_TAG, "Waiting done");
            continue;
        }
        else {
            PAL_DBG(LOG_TAG, "Speaker not in use");
            if (isDynamicCalTriggered) {
                PAL_DBG(LOG_TAG, "Dynamic Calibration triggered");
            }
            else if (sec < minIdleTime) {
                PAL_DBG(LOG_TAG, "Speaker not idle for minimum time. %lu", sec);
                spkrCalibrateWait();
                PAL_DBG(LOG_TAG, "Waited for speaker to be idle for min time");
                continue;
            }
            proceed = true;
        }
retry:
        if (proceed) {
            PAL_DBG(LOG_TAG, "Getting temperature of speakers");
            getSpeakerTemperatureList();

            for (i = 0; i < numberOfChannels; i++) {
                if ((spkerTempList[i] != -EINVAL) &&
                    (spkerTempList[i] < TZ_TEMP_MIN_THRESHOLD ||
                     spkerTempList[i] > TZ_TEMP_MAX_THRESHOLD)) {
                    PAL_ERR(LOG_TAG, "Temperature out of range. Retry");
                    spkrCalibrateWait();
                    if (retryCount < MAX_RETRY) {
                        retryCount++;
                        goto retry;
                    }
                    else
                        continue;
                }
            }
            for (i = 0; i < numberOfChannels; i++) {
                // Converting to Q6 format
                spkerTempList[i] = (spkerTempList[i]*(1<<6));
            }
        }
        else {
            continue;
        }

        // Check whether speaker was in use in the meantime when temperature
        // was being read.
        proceed = false;
        if (isSpeakerInUse(&sec)) {
            PAL_DBG(LOG_TAG, "Speaker in use. Wait for proper time");
            spkrCalibrateWait();
            PAL_DBG(LOG_TAG, "Waiting done");
            continue;
        }
        else {
            PAL_DBG(LOG_TAG, "Speaker not in use");
            if (isDynamicCalTriggered) {
                PAL_DBG(LOG_TAG, "Dynamic calibration triggered");
            }
            else if (sec < minIdleTime) {
                PAL_DBG(LOG_TAG, "Speaker not idle for minimum time. %lu", sec);
                spkrCalibrateWait();
                PAL_DBG(LOG_TAG, "Waited for speaker to be idle for min time");
                continue;
            }
            proceed = true;
        }

        if (proceed) {
            // Start calibrating the speakers.
            PAL_DBG(LOG_TAG, "Speaker not in use, start calibration");
            spkrStartCalibration();
            if (spkrCalState == SPKR_CALIBRATED) {
                threadExit = true;
            }
        }
        else {
            continue;
        }
    }
    isDynamicCalTriggered = false;
    calThrdCreated = false;
    PAL_DBG(LOG_TAG, "Calibration done, exiting the thread");
}

SpeakerProtection::SpeakerProtection(struct pal_device *device,
                        std::shared_ptr<ResourceManager> Rm):Device(device, Rm)
{
    int status = 0;
    struct pal_device_info devinfo = {};
    FILE *fp = NULL;

    spkerTempList = NULL;

    if (rm->getSpQuickCalTime() > 0 &&
        rm->getSpQuickCalTime() < MIN_SPKR_IDLE_SEC)
        minIdleTime = rm->getSpQuickCalTime();
    else
        minIdleTime = MIN_SPKR_IDLE_SEC;

    rm = Rm;

    memset(&mDeviceAttr, 0, sizeof(struct pal_device));
    memcpy(&mDeviceAttr, device, sizeof(struct pal_device));

    threadExit = false;
    calThrdCreated = false;
    triggerCal = false;
    spkrCalState = SPKR_NOT_CALIBRATED;
    spkrProcessingState = SPKR_PROCESSING_IN_IDLE;

    isSpkrInUse = false;

    calibrationCallbackStatus = 0;
    mDspCallbackRcvd = false;

    rm->getDeviceInfo(device->id, PAL_STREAM_PROXY, "", &devinfo);
    numberOfChannels = devinfo.channels;
    PAL_DBG(LOG_TAG, "Number of Channels %d", numberOfChannels);

    rm->getDeviceInfo(PAL_DEVICE_IN_VI_FEEDBACK, PAL_STREAM_PROXY, "", &vi_device);
    PAL_DBG(LOG_TAG, "Number of Channels for VI path is %d", vi_device.channels);

    rm->getDeviceInfo(PAL_DEVICE_IN_CPS_FEEDBACK, PAL_STREAM_PROXY, "", &cps_device);
    PAL_DBG(LOG_TAG, "Number of Channels for CPS path is %d", cps_device.channels);

    spkerTempList = new int [numberOfChannels];
    // Get current time
    clock_gettime(CLOCK_BOOTTIME, &spkrLastTimeUsed);

    // Getting mixture controls from Resource Manager
    status = rm->getVirtualAudioMixer(&virtMixer);
    if (status) {
        PAL_ERR(LOG_TAG,"virt mixer error %d", status);
    }
    status = rm->getHwAudioMixer(&hwMixer);
    if (status) {
        PAL_ERR(LOG_TAG,"hw mixer error %d", status);
    }

    if (device->id == PAL_DEVICE_OUT_HANDSET) {
        vi_device.channels = 1;
        cps_device.channels = 1;
        PAL_DBG(LOG_TAG, "Device id: %d vi_device.channels: %d cps_device.channels: %d",
                              device->id, vi_device.channels, cps_device.channels);
        goto exit;
    }

    fp = fopen(PAL_SP_TEMP_PATH, "rb");
    if (fp) {
        PAL_DBG(LOG_TAG, "Cal File exists. Reading from it");
        spkrCalState = SPKR_CALIBRATED;
    }
    else {
        PAL_DBG(LOG_TAG, "Calibration Not done");
        mCalThread = std::thread(&SpeakerProtection::spkrCalibrationThread,
                            this);
        calThrdCreated = true;
    }
exit:
    PAL_DBG(LOG_TAG, "exit. calThrdCreated :%d", calThrdCreated);
}

SpeakerProtection::~SpeakerProtection()
{
    if (customPayload)
        free(customPayload);

    customPayload = NULL;
    customPayloadSize = 0;
}

void SpeakerProtection::updateSPcustomPayload()
{
    PayloadBuilder* builder = new PayloadBuilder();
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    std::string backEndName;
    std::shared_ptr<Device> dev = nullptr;
    Stream *stream = NULL;
    Session *session = NULL;
    std::vector<Stream*> activeStreams;
    uint32_t miid = 0, ret;
    param_id_sp_op_mode_t spModeConfg;

    rm->getBackendName(mDeviceAttr.id, backEndName);
    dev = Device::getInstance(&mDeviceAttr, rm);
    ret = rm->getActiveStream_l(activeStreams, dev);
    if ((0 != ret) || (activeStreams.size() == 0)) {
        PAL_ERR(LOG_TAG, " no active stream available");
        goto exit;
    }
    stream = static_cast<Stream *>(activeStreams[0]);
    stream->getAssociatedSession(&session);
    ret = dynamic_cast<SessionAR*>(session)->getMIID(backEndName.c_str(), MODULE_SP, &miid);
    if (ret) {
        PAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", MODULE_SP, ret);
        goto exit;
    }

    if (customPayloadSize) {
        free(customPayload);
        customPayloadSize = 0;
    }

    spModeConfg.operation_mode = NORMAL_MODE;
    payloadSize = 0;
    builder->payloadSPConfig(&payload, &payloadSize, miid,
                    PARAM_ID_SP_OP_MODE,(void *)&spModeConfg);
    if (payloadSize) {
        ret = updateCustomPayload(payload, payloadSize);
        free(payload);
        if (0 != ret) {
            PAL_ERR(LOG_TAG," updateCustomPayload Failed\n");
        }
    }

exit:
    if(builder) {
       delete builder;
       builder = NULL;
    }
    return;
}


int SpeakerProtection::speakerProtectionDynamicCal()
{
    int ret = 0;

    PAL_DBG(LOG_TAG, "Enter");

    if (calThrdCreated) {
        PAL_DBG(LOG_TAG, "Calibration already triggered Thread State %d",
                        calThrdCreated);
        return ret;
    }

    threadExit = false;
    spkrCalState = SPKR_NOT_CALIBRATED;

    calibrationCallbackStatus = 0;
    mDspCallbackRcvd = false;

    calThrdCreated = true;
    isDynamicCalTriggered = true;

    std::thread dynamicCalThread(&SpeakerProtection::spkrCalibrationThread, this);

    dynamicCalThread.detach();

    PAL_DBG(LOG_TAG, "Exit");

    return ret;
}

int SpeakerProtection::start()
{
    PAL_DBG(LOG_TAG, "Enter");

    if (rm->IsVIRecordStarted()) {
        PAL_DBG(LOG_TAG, "record running so just update SP payload");
        updateSPcustomPayload();
    }
    else {
        spkrProtProcessingMode(true);
    }

    PAL_DBG(LOG_TAG, "Calling Device start");
    Device::start();
    return 0;
}

int SpeakerProtection::stop()
{
    PAL_DBG(LOG_TAG, "Inside Speaker Protection stop");
    Device::stop();
    if (rm->IsVIRecordStarted()) {
        PAL_DBG(LOG_TAG, "record running so no need to proceed");
        rm->setVIRecordState(false);
        return 0;
    }
    spkrProtProcessingMode(false);
    return 0;
}


int32_t SpeakerProtection::setParameter(uint32_t param_id, void *param)
{
    PAL_DBG(LOG_TAG, "Inside Speaker Protection Set parameters");
    (void ) param;
    if (param_id == PAL_SP_MODE_DYNAMIC_CAL)
        speakerProtectionDynamicCal();
    return 0;
}

int32_t SpeakerProtection::getFTMParameter(void **param)
{
    int size = 0, status = 0 ;
    int spkr1_status = 0;
    int spkr2_status = 0;
    uint32_t miid = 0;
    const char *getParamControl = "getParam";
    char *pcmDeviceName = NULL;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    struct mixer_ctl *ctl;
    std::ostringstream cntrlName;
    std::ostringstream resString;
    std::string backendName;
    param_id_sp_th_vi_ftm_params_t ftm;
    param_id_sp_ex_vi_ftm_params_t exFtm;
    PayloadBuilder* builder = new PayloadBuilder();
    vi_th_ftm_params_t ftm_ret[numberOfChannels];
    vi_ex_ftm_params_t exFtm_ret[numberOfChannels];
    param_id_sp_th_vi_ftm_params_t *ftmValue;
    param_id_sp_ex_vi_ftm_params_t *exFtmValue;
    pal_spkr_prot_payload spkrProtPayload;

    memset(&ftm_ret, 0,sizeof(vi_th_ftm_params_t) * numberOfChannels);
    memset(&exFtm_ret, 0,sizeof(vi_ex_ftm_params_t) * numberOfChannels);

    pcmDeviceName = rm->getDeviceNameFromID(pcmDevIdTx.at(0));
    if (pcmDeviceName) {
        cntrlName<<pcmDeviceName<<" "<<getParamControl;
    }
    else {
        PAL_ERR(LOG_TAG, "Error: %d Unable to get Device name\n", -EINVAL);
        goto exit;
    }

    ctl = mixer_get_ctl_by_name(virtMixer, cntrlName.str().data());
    if (!ctl) {
        status = -ENOENT;
        PAL_ERR(LOG_TAG, "Error: %d Invalid mixer control: %s\n", status,cntrlName.str().data());
        goto exit;
    }
    rm->getBackendName(PAL_DEVICE_IN_VI_FEEDBACK, backendName);
    if (!strlen(backendName.c_str())) {
        status = -ENOENT;
        PAL_ERR(LOG_TAG, "Error: %d Failed to obtain VI backend name", status);
        goto exit;
    }

    status = SessionAlsaUtils::getModuleInstanceId(virtMixer, pcmDevIdTx.at(0),
                        backendName.c_str(), MODULE_VI, &miid);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Error: %d Failed to get tag info %x", status, MODULE_VI);
        goto exit;
    }

    ftm.num_ch = numberOfChannels;
    builder->payloadSPConfig (&payload, &payloadSize, miid,
            PARAM_ID_SP_TH_VI_FTM_PARAMS, &ftm);

    status = mixer_ctl_set_array(ctl, payload, payloadSize);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Set failed status = %d", status);
        goto exit;
    }

    memset(payload, 0, payloadSize);

    status = mixer_ctl_get_array(ctl, payload, payloadSize);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Get failed status = %d", status);
    }
    else {

        ftmValue = (param_id_sp_th_vi_ftm_params_t *) (payload +
                        sizeof(struct apm_module_param_data_t));

        for (int i = 0; i < numberOfChannels; i++) {
            ftm_ret[i].ftm_rDC_q24 = ftmValue->vi_th_ftm_params[i].ftm_rDC_q24;
            ftm_ret[i].ftm_temp_q22 = ftmValue->vi_th_ftm_params[i].ftm_temp_q22;
            ftm_ret[i].status = ftmValue->vi_th_ftm_params[i].status;
        }
    }

    PAL_DBG(LOG_TAG, "Got FTM value with status %d", ftm_ret[0].status);

    if (payload) {
        delete payload;
        payloadSize = 0;
        payload = NULL;
    }

    exFtm.num_ch = numberOfChannels;
    builder->payloadSPConfig (&payload, &payloadSize, miid,
            PARAM_ID_SP_EX_VI_FTM_PARAMS, &exFtm);

    status = mixer_ctl_set_array(ctl, payload, payloadSize);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Error: %d Mixer cntrl Set failed", status);
        goto exit;
    }

    memset(payload, 0, payloadSize);

    status = mixer_ctl_get_array(ctl, payload, payloadSize);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Error: %d Get failed ", status);
    }
    else {
        exFtmValue = (param_id_sp_ex_vi_ftm_params_t *) (payload +
                                sizeof(struct apm_module_param_data_t));
        for (int i = 0; i < numberOfChannels; i++) {
            exFtm_ret[i].ftm_Re_q24 = exFtmValue->fbsp_ex_vi_ftm_get_param[i].ftm_Re_q24;
            exFtm_ret[i].ftm_Bl_q24 = exFtmValue->fbsp_ex_vi_ftm_get_param[i].ftm_Bl_q24;
            exFtm_ret[i].ftm_Kms_q24 = exFtmValue->fbsp_ex_vi_ftm_get_param[i].ftm_Kms_q24;
            exFtm_ret[i].ftm_Fres_q20 = exFtmValue->fbsp_ex_vi_ftm_get_param[i].ftm_Fres_q20;
            exFtm_ret[i].ftm_Qms_q24 = exFtmValue->fbsp_ex_vi_ftm_get_param[i].ftm_Qms_q24;
            exFtm_ret[i].status = exFtmValue->fbsp_ex_vi_ftm_get_param[i].status;
        }
    }
    PAL_DBG(LOG_TAG, "Got FTM Excursion value with status %d", exFtm_ret[0].status);

    if (payload) {
        delete payload;
        payloadSize = 0;
        payload = NULL;
    }

    switch(numberOfChannels) {
        case 1 :
            if (exFtm_ret[0].status == 4 && ftm_ret[0].status == 4)
                spkr1_status = 1;
            resString << "SpkrParamStatus: " << spkr1_status << "; Rdc: "
                    << ((ftm_ret[0].ftm_rDC_q24)/(1<<24)) << "; Temp: "
                    << ((ftm_ret[0].ftm_temp_q22)/(1<<22)) << "; Res: "
                    << ((exFtm_ret[0].ftm_Re_q24)/(1<<24)) << "; Bl: "
                    << ((exFtm_ret[0].ftm_Bl_q24)/(1<<24)) << "; Rms: "
                    << ((exFtm_ret[0].ftm_Rms_q24)/(1<<24)) << "; Kms: "
                    << ((exFtm_ret[0].ftm_Kms_q24)/(1<<24)) << "; Fres: "
                    << ((exFtm_ret[0].ftm_Fres_q20)/(1<<20)) << "; Qms: "
                    << ((exFtm_ret[0].ftm_Qms_q24)/(1<<24));
        break;
        case 2 :
            if (exFtm_ret[0].status == 4 && ftm_ret[0].status == 4)
                spkr1_status = 1;
            if (exFtm_ret[1].status == 4 && ftm_ret[1].status == 4)
                spkr2_status = 1;
            resString << "SpkrParamStatus: " << spkr1_status <<", "<< spkr2_status
                    << "; Rdc: " << (((float)ftm_ret[0].ftm_rDC_q24)/(1<<24)) << ", "
                    << (((float)ftm_ret[1].ftm_rDC_q24)/(1<<24)) << "; Temp: "
                    << ((ftm_ret[0].ftm_temp_q22)/(1<<22)) << ", "
                    << ((ftm_ret[1].ftm_temp_q22)/(1<<22)) <<"; Res: "
                    << ((exFtm_ret[0].ftm_Re_q24)/(1<<24)) << ", "
                    << ((exFtm_ret[1].ftm_Re_q24)/(1<<24)) << "; Bl: "
                    << ((exFtm_ret[0].ftm_Bl_q24)/(1<<24)) << ", "
                    << ((exFtm_ret[1].ftm_Bl_q24)/(1<<24)) << "; Rms: "
                    << ((exFtm_ret[0].ftm_Rms_q24)/(1<<24)) << ", "
                    << ((exFtm_ret[1].ftm_Rms_q24)/(1<<24)) << "; Kms: "
                    << ((exFtm_ret[0].ftm_Kms_q24)/(1<<24)) << ", "
                    << ((exFtm_ret[1].ftm_Kms_q24)/(1<<24)) << "; Fres: "
                    << (((float)exFtm_ret[0].ftm_Fres_q20)/(1<<20)) << ", "
                    << (((float)exFtm_ret[1].ftm_Fres_q20)/(1<<20)) << "; Qms: "
                    << ((exFtm_ret[0].ftm_Qms_q24)/(1<<24)) << ", "
                    << ((exFtm_ret[1].ftm_Qms_q24)/(1<<24));
        break;
        default :
            PAL_ERR(LOG_TAG, "No support for Speakers > 2");
            goto exit;
    }

    PAL_DBG(LOG_TAG, "Get param value %s", resString.str().c_str());
    if (resString.str().length() > 0) {
        memcpy((char *) (param), resString.str().c_str(),
                resString.str().length());
        size = resString.str().length();

        // Get is done now, we will clear up the stored mode now
        rm->setSpkrProtModeValue(0);
    }

exit :
    if(builder) {
       delete builder;
       builder = NULL;
    }
    if(!status)
       return size;
    else
      return status;

}

int32_t SpeakerProtection::getCalibrationData(void **param)
{
    int i, status = 0;
    struct vi_r0t0_cfg_t r0t0Array[numberOfChannels];
    double dr0[numberOfChannels];
    double dt0[numberOfChannels];
    std::ostringstream resString;

    memset(r0t0Array, 0, sizeof(vi_r0t0_cfg_t) * numberOfChannels);
    memset(dr0, 0, sizeof(double) * numberOfChannels);
    memset(dt0, 0, sizeof(double) * numberOfChannels);

    FILE *fp = fopen(PAL_SP_TEMP_PATH, "rb");
    if (fp) {
        for (i = 0; i < numberOfChannels; i++) {
            fread(&r0t0Array[i].r0_cali_q24,
                    sizeof(r0t0Array[i].r0_cali_q24), 1, fp);
            fread(&r0t0Array[i].t0_cali_q6,
                    sizeof(r0t0Array[i].t0_cali_q6), 1, fp);
            // Convert to readable format
            dr0[i] = ((double)r0t0Array[i].r0_cali_q24)/(1 << 24);
            dt0[i] = ((double)r0t0Array[i].t0_cali_q6)/(1 << 6);
        }
        PAL_DBG(LOG_TAG, "R0= %lf, %lf, T0= %lf, %lf", dr0[0], dr0[1], dt0[0], dt0[1]);
        fclose(fp);
    }
    else {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "No cal file present");
    }
    resString << "SpkrCalStatus: " << status << "; R0: " << dr0[0] << ", "
              << dr0[1] << "; T0: "<< dt0[0] << ", " << dt0[1] << ";";

    PAL_DBG(LOG_TAG, "Calibration value %s", resString.str().c_str());

    memcpy((char *) (param), resString.str().c_str(), resString.str().length());

    if(!status)
       return resString.str().length();
    else
    return status;

}

int32_t SpeakerProtection::getParameter(uint32_t param_id, void **param)
{
    int32_t status = 0;
    switch(param_id) {
        case PAL_PARAM_ID_SP_GET_CAL:
            status = getCalibrationData(param);
        break;
        case PAL_PARAM_ID_SP_MODE:
            status = getFTMParameter(param);
        break;
        default :
            PAL_ERR(LOG_TAG, "Unsupported operation");
            status = -EINVAL;
        break;
    }
    return status;
}

/*
 * VI feedack related functionalities
 */

extern "C" void CreateFeedbackDevice(struct pal_device *device,
                                        const std::shared_ptr<ResourceManager> rm,
                                        pal_device_id_t id, bool createDevice,
                                        std::shared_ptr<Device> *dev) {
    if (createDevice)
        *dev = SpeakerFeedback::getInstance(device, rm);
    else
        *dev = SpeakerFeedback::getObject();

}

void SpeakerFeedback::updateVIcustomPayload()
{
    PayloadBuilder* builder = new PayloadBuilder();
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    std::string backEndName;
    std::shared_ptr<Device> dev = nullptr;
    Stream *stream = NULL;
    Session *session = NULL;
    std::vector<Stream*> activeStreams;
    uint32_t miid = 0, ret = 0;
    struct vi_r0t0_cfg_t r0t0Array[numSpeaker];
    FILE *fp = NULL;
    param_id_sp_th_vi_r0t0_cfg_t *spR0T0confg;
    param_id_sp_vi_op_mode_cfg_t modeConfg;
    param_id_sp_vi_channel_map_cfg_t viChannelMapConfg;
    param_id_sp_ex_vi_mode_cfg_t viExModeConfg;

    rm->getBackendName(mDeviceAttr.id, backEndName);
    dev = Device::getInstance(&mDeviceAttr, rm);
    ret = rm->getActiveStream_l(activeStreams, dev);
    if ((0 != ret) || (activeStreams.size() == 0)) {
        PAL_ERR(LOG_TAG, " no active stream available");
        goto exit;
    }
    stream = static_cast<Stream *>(activeStreams[0]);
    stream->getAssociatedSession(&session);
    ret = dynamic_cast<SessionAR*>(session)->getMIID(backEndName.c_str(), MODULE_VI, &miid);
    if (ret) {
        PAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", MODULE_VI, ret);
        goto exit;
    }

    if (customPayloadSize) {
        free(customPayload);
        customPayloadSize = 0;
    }

    memset(&modeConfg, 0, sizeof(modeConfg));
    memset(&viChannelMapConfg, 0, sizeof(viChannelMapConfg));
    memset(&viExModeConfg, 0, sizeof(viExModeConfg));
    memset(&r0t0Array, 0, sizeof(struct vi_r0t0_cfg_t) * numSpeaker);

    // Setting the mode of VI module
    modeConfg.num_speakers = numSpeaker;
    modeConfg.th_operation_mode = NORMAL_MODE;
    modeConfg.th_quick_calib_flag = 0;
    builder->payloadSPConfig(&payload, &payloadSize, miid,
                             PARAM_ID_SP_VI_OP_MODE_CFG,(void *)&modeConfg);
    if (payloadSize) {
        ret = updateCustomPayload(payload, payloadSize);
        free(payload);
        if (0 != ret) {
            PAL_ERR(LOG_TAG," updateCustomPayload Failed for VI_OP_MODE_CFG\n");
        }
    }

    // Setting Channel Map configuration for VI module
    viChannelMapConfg.num_ch = numSpeaker * 2;
    payloadSize = 0;

    builder->payloadSPConfig(&payload, &payloadSize, miid,
                    PARAM_ID_SP_VI_CHANNEL_MAP_CFG,(void *)&viChannelMapConfg);
    if (payloadSize) {
        ret = updateCustomPayload(payload, payloadSize);
        free(payload);
        if (0 != ret) {
            PAL_ERR(LOG_TAG," updateCustomPayload Failed for CHANNEL_MAP_CFG\n");
        }
    }

    fp = fopen(PAL_SP_TEMP_PATH, "rb");
    if (fp) {
        PAL_DBG(LOG_TAG, "Speaker calibrated. Send calibrated value");
        for (int i = 0; i < numSpeaker; i++) {
            fread(&r0t0Array[i].r0_cali_q24,
                    sizeof(r0t0Array[i].r0_cali_q24), 1, fp);
            fread(&r0t0Array[i].t0_cali_q6,
                    sizeof(r0t0Array[i].t0_cali_q6), 1, fp);
        }
    }
    else {
        PAL_DBG(LOG_TAG, "Speaker not calibrated. Send safe value");
        for (int i = 0; i < numSpeaker; i++) {
            r0t0Array[i].r0_cali_q24 = MIN_RESISTANCE_SPKR_Q24;
            r0t0Array[i].t0_cali_q6 = SAFE_SPKR_TEMP_Q6;
        }
    }
    spR0T0confg = (param_id_sp_th_vi_r0t0_cfg_t *)calloc(1,
                        sizeof(param_id_sp_th_vi_r0t0_cfg_t) +
                        sizeof(vi_r0t0_cfg_t) * numSpeaker);
    if (!spR0T0confg) {
        PAL_ERR(LOG_TAG," updateCustomPayload Failed\n");
        return;
    }
    spR0T0confg->num_ch = numSpeaker;

    memcpy(spR0T0confg->r0t0_cfg, r0t0Array, sizeof(vi_r0t0_cfg_t) *
            numSpeaker);

    payloadSize = 0;
    builder->payloadSPConfig(&payload, &payloadSize, miid,
                    PARAM_ID_SP_TH_VI_R0T0_CFG,(void *)spR0T0confg);
    if (payloadSize) {
        ret = updateCustomPayload(payload, payloadSize);
        free(payload);
        free(spR0T0confg);
        if (0 != ret) {
            PAL_ERR(LOG_TAG," updateCustomPayload Failed\n");
        }
    }
exit:
    if(builder) {
       delete builder;
       builder = NULL;
    }
    return;
}

SpeakerFeedback::SpeakerFeedback(struct pal_device *device,
                                std::shared_ptr<ResourceManager> Rm):Device(device, Rm)
{
    struct pal_device_info devinfo = {};

    memset(&mDeviceAttr, 0, sizeof(struct pal_device));
    memcpy(&mDeviceAttr, device, sizeof(struct pal_device));
    rm = Rm;


    rm->getDeviceInfo(mDeviceAttr.id, PAL_STREAM_PROXY, mDeviceAttr.custom_config.custom_key, &devinfo);
    numSpeaker = devinfo.channels;
}

SpeakerFeedback::~SpeakerFeedback()
{
}

int32_t SpeakerFeedback::start()
{
    rm->setVIRecordState(true);
    // Do the customPayload configuration for VI path and call the Device::start
    PAL_DBG(LOG_TAG," Feedback start\n");
    if (rm->IsSpeakerProtectionEnabled())
        updateVIcustomPayload();

    Device::start();

    return 0;
}

int32_t SpeakerFeedback::stop()
{
    rm->setVIRecordState(false);
    PAL_DBG(LOG_TAG," Feedback stop\n");
    Device::stop();

    return 0;
}

std::shared_ptr<Device> SpeakerFeedback::getInstance(struct pal_device *device,
                                                     std::shared_ptr<ResourceManager> Rm)
{
    PAL_DBG(LOG_TAG," Feedback getInstance\n");
    if (!obj) {
        std::shared_ptr<Device> sp(new SpeakerFeedback(device, Rm));
        obj = sp;
    }
    return obj;
}

std::shared_ptr<Device> SpeakerFeedback::getObject()
{
    return obj;
}

