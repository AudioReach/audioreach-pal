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

#define LOG_TAG "PAL: SessionAR"

#include "SessionAR.h"
#include "Stream.h"
#include "ResourceManager.h"
#include "SessionAlsaUtils.h"
#include "SessionAgm.h"
#include "SessionGsl.h"
#include "SessionAlsaPcm.h"
#include "SessionAlsaCompress.h"
#include "SessionAlsaUtils.h"
#include "SessionAlsaVoice.h"

#include <agm/agm_api.h>
#include "apm_api.h"
#include <sstream>

struct pcm *SessionAR::pcmEcTx = NULL;
std::vector<int> SessionAR::pcmDevEcTxIds = {0};
int SessionAR::extECRefCnt = 0;
std::mutex SessionAR::extECMutex;

Session* SessionAR::makeARSession(const std::shared_ptr<ResourceManager>& rm, const struct pal_stream_attributes *sAttr)
{
    if (!rm || !sAttr) {
        PAL_ERR(LOG_TAG, "Invalid parameters passed");
        return nullptr;
    }

    Session* s = (Session*) nullptr;

    switch (sAttr->type) {
        //create compressed if the stream type is compressed
        case PAL_STREAM_COMPRESSED:
            s =  new SessionAlsaCompress(rm);
            break;
        case PAL_STREAM_VOICE_CALL:
            s = new SessionAlsaVoice(rm);
            break;
        case PAL_STREAM_NON_TUNNEL:
            s = new SessionAgm(rm);
            break;
        default:
            s = new SessionAlsaPcm(rm);
            break;
    }
    return s;
}

void SessionAR::setPmQosMixerCtl(pmQosVote vote)
{
    int status = 0;
    struct audio_route *audioRoute;

    status = rm->getAudioRoute(&audioRoute);
    if (!status) {
        if (vote == PM_QOS_VOTE_DISABLE) {
            audio_route_reset_and_update_path(audioRoute, "PM_QOS_Vote");
            PAL_DBG(LOG_TAG,"mixer control disabled for PM_QOS Vote \n");
        } else if (vote == PM_QOS_VOTE_ENABLE) {
            audio_route_apply_and_update_path(audioRoute, "PM_QOS_Vote");
            PAL_DBG(LOG_TAG,"mixer control enabled for PM_QOS Vote \n");
        }
    } else {
        PAL_ERR(LOG_TAG,"could not get audioRoute, not setting mixer control for PM_QOS \n");
    }
}

uint32_t SessionAR::getModuleInfo(const char *control, uint32_t tagId, uint32_t *miid, struct mixer_ctl **ctl, int *device)
{
    int status = 0;
    int dev = 0;
    struct mixer_ctl *mixer_ctl = NULL;

    if (!rxAifBackEnds.empty()) { /** search in RX GKV */
        mixer_ctl = getFEMixerCtl(control, &dev, PAL_AUDIO_OUTPUT);
        if (!mixer_ctl) {
            PAL_ERR(LOG_TAG, "Invalid mixer control\n");
            status = -ENOENT;
            goto exit;
        }
        status = SessionAlsaUtils::getModuleInstanceId(mixer, dev, rxAifBackEnds[0].second.data(), tagId, miid);
        if (status) /** if not found, reset miid to 0 again */
            *miid = 0;
    }

    if (!txAifBackEnds.empty() && !(*miid)) { /** search in TX GKV */
        mixer_ctl = getFEMixerCtl(control, &dev, PAL_AUDIO_INPUT);
        if (!mixer_ctl) {
            PAL_ERR(LOG_TAG, "Invalid mixer control\n");
            status = -ENOENT;
            goto exit;
        }
        status = SessionAlsaUtils::getModuleInstanceId(mixer, dev, txAifBackEnds[0].second.data(), tagId, miid);
        if (status)
            *miid = 0;
    }

    if (*miid == 0) {
        PAL_ERR(LOG_TAG, "failed to look for module with tagID 0x%x", tagId);
        status = -EINVAL;
        goto exit;
    }

    if (device)
        *device = dev;

    if (ctl)
        *ctl = mixer_ctl;

    PAL_DBG(LOG_TAG, "got miid = 0x%04x, device = %d", *miid, dev);
exit:
    if (status) {
        if (device)
            *device = 0;
        if (ctl)
            *ctl = NULL;
        *miid = 0;
        PAL_ERR(LOG_TAG, "Exit. status %d", status);
    }
    return status;
}

int SessionAR::setEffectParametersTKV(Stream *s __unused, effect_pal_payload_t *effectPayload)
{
    int status = 0;
    int device = 0;
    uint32_t tag;
    uint32_t nTkvs;
    uint32_t tagConfigSize;
    struct mixer_ctl *ctl;
    pal_key_vector_t *palKVPair;
    struct agm_tag_config* tagConfig = NULL;
    std::vector <std::pair<int, int>> tkv;
    const char *control = "setParamTag";

    PAL_DBG(LOG_TAG, "Enter.");

    palKVPair = (pal_key_vector_t *)effectPayload->payload;
    nTkvs =  palKVPair->num_tkvs;
    tkv.clear();
    for (int i = 0; i < nTkvs; i++) {
        tkv.push_back(std::make_pair(palKVPair->kvp[i].key, palKVPair->kvp[i].value));
    }
    if (tkv.size() == 0) {
        status = -EINVAL;
        goto exit;
    }

    tagConfigSize = sizeof(struct agm_tag_config) + (tkv.size() * sizeof(agm_key_value));
    tagConfig = (struct agm_tag_config *) malloc(tagConfigSize);
    if(!tagConfig) {
        status = -ENOMEM;
        goto exit;
    }

    tag = effectPayload->tag;
    status = SessionAlsaUtils::getTagMetadata(tag, tkv, tagConfig);
    if (0 != status) {
        goto exit;
    }

    /* Prepare mixer control */
    ctl = getFEMixerCtl(control, &device, PAL_AUDIO_OUTPUT);
    if (!ctl) {
        PAL_ERR(LOG_TAG, "Invalid mixer control\n");
        status = -ENOENT;
        goto exit;
    }

    status = mixer_ctl_set_array(ctl, tagConfig, tagConfigSize);
    if (status != 0) {
        PAL_DBG(LOG_TAG, "Unable to set TKV in Rx path, trying in Tx\n");
        /* Rx set failed, Try on Tx path we well */
        ctl = getFEMixerCtl(control, &device, PAL_AUDIO_INPUT);
        if (!ctl) {
            PAL_ERR(LOG_TAG, "Invalid mixer control\n");
            status = -ENOENT;
            goto exit;
        }

        status = mixer_ctl_set_array(ctl, tagConfig, tagConfigSize);
        if (status != 0) {
            PAL_ERR(LOG_TAG, "failed to set the param %d", status);
            goto exit;
        }
    }

exit:
    ctl = NULL;

    if (tagConfig) {
        free(tagConfig);
        tagConfig = NULL;
    }
    PAL_INFO(LOG_TAG, "mixer set tkv status = %d\n", status);
    return status;
}

int SessionAR::setEffectParametersNonTKV(Stream *s __unused, effect_pal_payload_t *effectPayload)
{
    int status = 0;
    int device = 0;
    PayloadBuilder builder;

    uint32_t miid = 0;
    const char *control = "setParam";
    size_t payloadSize = 0;
    uint8_t *payloadData = NULL;
    pal_effect_custom_payload_t *effectCustomPayload = nullptr;

    PAL_DBG(LOG_TAG, "Enter.");

    if (!effectPayload) {
        PAL_ERR(LOG_TAG, "Invalid effectPayload address.\n");
        return -EINVAL;
    }

    /* This is set param call, find out miid first */
    status = getModuleInfo(control, effectPayload->tag, &miid, NULL, &device);
    if (status || !miid) {
        PAL_ERR(LOG_TAG, "failed to look for module with tagID 0x%x, status = %d",
                    effectPayload->tag, status);
        return -EINVAL;
    }

    /* Now we got the miid, build set param payload */
    effectCustomPayload = (pal_effect_custom_payload_t *)effectPayload->payload;
    status = builder.payloadCustomParam(&payloadData, &payloadSize,
            effectCustomPayload->data,
            effectPayload->payloadSize - sizeof(uint32_t),
            miid, effectCustomPayload->paramId);
    if (status != 0) {
        PAL_ERR(LOG_TAG, "payloadCustomParam failed. status = %d",
                status);
        goto exit;
    }
    /* set param through set mixer param */
    status = SessionAlsaUtils::setMixerParameter(mixer,
            device,
            payloadData,
            payloadSize);
    PAL_INFO(LOG_TAG, "mixer set param status = %d\n", status);

exit:
    if (payloadData) {
        free(payloadData);
        payloadData = NULL;
    }
    if (status && effectCustomPayload) {
        PAL_ERR(LOG_TAG, "setEffectParameters for param_id %d failed, status = %d",
                effectCustomPayload->paramId, status);
    }
    return status;

}

int SessionAR::setEffectParameters(Stream *s, effect_pal_payload_t *effectPayload)
{
    int status = 0;

    PAL_DBG(LOG_TAG, "Enter.");

    /* Identify whether this is tkv or set param call */
    if (effectPayload->isTKV) {
        /* This is tkv set call */
        status = setEffectParametersTKV(s, effectPayload);
        if (0 != status) {
            PAL_ERR(LOG_TAG, "Get setEffectParameters with TKV payload failed"
                                ", status = %d", status);
            goto exit;
        }
    } else {
        status = setEffectParametersNonTKV(s, effectPayload);
        if (0 != status) {
            PAL_ERR(LOG_TAG, "Get setEffectParameters with non TKV payload failed"
                                ", status = %d", status);
            goto exit;
        }
    }

exit:
    if (status)
        PAL_ERR(LOG_TAG, "Exit. status %d", status);

    return status;
}

int SessionAR::getEffectParameters(Stream *s __unused, effect_pal_payload_t *effectPayload)
{
    int status = 0;
    uint8_t *ptr = NULL;

    uint8_t *payloadData = NULL;
    size_t payloadSize = 0;
    uint32_t miid = 0;
    const char *control = "getParam";
    struct mixer_ctl *ctl = NULL;
    pal_effect_custom_payload_t *effectCustomPayload = nullptr;
    PayloadBuilder builder;

    PAL_DBG(LOG_TAG, "Enter.");

    status = getModuleInfo(control, effectPayload->tag, &miid, &ctl, NULL);
    if (status || !miid) {
        PAL_ERR(LOG_TAG, "failed to look for module with tagID 0x%x, status = %d",
                    effectPayload->tag, status);
        status = -EINVAL;
        goto exit;
    }

    effectCustomPayload = (pal_effect_custom_payload_t *)(effectPayload->payload);
    if (effectPayload->payloadSize < sizeof(pal_effect_custom_payload_t)) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "memory for retrieved data is too small");
        goto exit;
    }

    builder.payloadQuery(&payloadData, &payloadSize,
                            miid, effectCustomPayload->paramId,
                            effectPayload->payloadSize - sizeof(uint32_t));
    status = mixer_ctl_set_array(ctl, payloadData, payloadSize);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Set custom config failed, status = %d", status);
        goto exit;
    }

    status = mixer_ctl_get_array(ctl, payloadData, payloadSize);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Get custom config failed, status = %d", status);
        goto exit;
    }

    ptr = (uint8_t *)payloadData + sizeof(struct apm_module_param_data_t);
    ar_mem_cpy(effectCustomPayload->data, effectPayload->payloadSize,
                        ptr, effectPayload->payloadSize);

exit:
    ctl = NULL;
    if (payloadData)
        free(payloadData);
    PAL_ERR(LOG_TAG, "Exit. status %d", status);
    return status;
}

int SessionAR::checkAndSetExtEC(const std::shared_ptr<ResourceManager>& rm,
                              Stream *s, bool is_enable)
{
    struct pcm_config config;
    struct pal_stream_attributes sAttr = {};
    int32_t status = 0;
    std::shared_ptr<Device> dev = nullptr;
    std::vector <std::shared_ptr<Device>> extEcTxDeviceList;
    int32_t extEcbackendId;
    std::vector <std::string> extEcbackendNames;
    struct pal_device device = {};

    PAL_DBG(LOG_TAG, "Enter.");

    extECMutex.lock();
    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG,"stream get attributes failed");
        status = -EINVAL;
        goto exit;
    }

    device.id = PAL_DEVICE_IN_EXT_EC_REF;
    rm->getDeviceConfig(&device, &sAttr);
    dev = Device::getInstance(&device, rm);
    if (!dev) {
        PAL_ERR(LOG_TAG, "dev get instance failed");
        status = -EINVAL;
        goto exit;
    }

    if(!is_enable) {
        if (extECRefCnt > 0)
            extECRefCnt --;
        if (extECRefCnt == 0) {
            if (pcmEcTx) {
                status = pcm_stop(pcmEcTx);
                if (status) {
                    PAL_ERR(LOG_TAG, "pcm_stop - ec_tx failed %d", status);
                }
                dev->stop();

                status = pcm_close(pcmEcTx);
                if (status) {
                    PAL_ERR(LOG_TAG, "pcm_close - ec_tx failed %d", status);
                }
                dev->close();

                rm->freeFrontEndEcTxIds(pcmDevEcTxIds);
                pcmEcTx = NULL;
                ecRefDevId = PAL_DEVICE_OUT_MIN;
                extECMutex.unlock();
                rm->restoreInternalECRefs();
                extECMutex.lock();
            }
        }
    } else {
        extECRefCnt ++;
        if (extECRefCnt == 1) {
            extECMutex.unlock();
            rm->disableInternalECRefs(s);
            extECMutex.lock();
            extEcTxDeviceList.push_back(dev);
            pcmDevEcTxIds = rm->allocateFrontEndExtEcIds();
            if (pcmDevEcTxIds.size() == 0) {
                PAL_ERR(LOG_TAG, "ResourceManger::getBackEndNames returned no EXT_EC device Ids");
                status = -EINVAL;
                goto exit;
            }
            status = dev->open();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "dev open failed");
                status = -EINVAL;
                rm->freeFrontEndEcTxIds(pcmDevEcTxIds);
                goto exit;
            }
            status = dev->start();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "dev start failed");
                dev->close();
                rm->freeFrontEndEcTxIds(pcmDevEcTxIds);
                status = -EINVAL;
                goto exit;
            }

            extEcbackendId = extEcTxDeviceList[0]->getSndDeviceId();
            extEcbackendNames = rm->getBackEndNames(extEcTxDeviceList);
            status = SessionAlsaUtils::openDev(rm, pcmDevEcTxIds, extEcbackendId,
                extEcbackendNames.at(0).c_str());
            if (0 != status) {
                PAL_ERR(LOG_TAG, "SessionAlsaUtils::openDev failed");
                dev->stop();
                dev->close();
                rm->freeFrontEndEcTxIds(pcmDevEcTxIds);
                status = -EINVAL;
                goto exit;
            }
            pcmEcTx = pcm_open(rm->getVirtualSndCard(), pcmDevEcTxIds.at(0), PCM_IN, &config);
            if (!pcmEcTx) {
                PAL_ERR(LOG_TAG, "Exit pcm-ec-tx open failed");
                dev->stop();
                dev->close();
                rm->freeFrontEndEcTxIds(pcmDevEcTxIds);
                status = -EINVAL;
                goto exit;
            }

            if (!pcm_is_ready(pcmEcTx)) {
                PAL_ERR(LOG_TAG, "Exit pcm-ec-tx open not ready");
                pcmEcTx = NULL;
                dev->stop();
                dev->close();
                rm->freeFrontEndEcTxIds(pcmDevEcTxIds);
                status = -EINVAL;
                goto exit;
            }

            status = pcm_start(pcmEcTx);
            if (status) {
                PAL_ERR(LOG_TAG, "pcm_start ec_tx failed %d", status);
                pcm_close(pcmEcTx);
                pcmEcTx = NULL;
                dev->stop();
                dev->close();
                rm->freeFrontEndEcTxIds(pcmDevEcTxIds);
                status = -EINVAL;
                goto exit;
            }
        }
    }

exit:
    if (is_enable && status) {
        PAL_DBG(LOG_TAG, "Reset extECRefCnt as EXT EC graph fails to setup");
        extECRefCnt = 0;
    }
    extECMutex.unlock();
    PAL_DBG(LOG_TAG, "Exit.");
    return status;
}

/*
 Handle case when charging going is on and PB starts on speaker.
 Below steps are to ensure HW transition from charging->boost->charging
 which will avoid USB collapse due to HW transition in less moment.
 1. Set concurrency bit to update charger driver for respective session
 2. Enable Speaker boost when Audio done voting as part of PCM_open
 3. Set ICL config in device:Speaker module.
*/
int SessionAR::NotifyChargerConcurrency(std::shared_ptr<ResourceManager>rm, bool state)
{
    int status = -EINVAL;

    if (!rm)
        goto exit;

    PAL_DBG(LOG_TAG, "Enter concurrency state %d Notify state %d \n",
            rm->getConcurrentBoostState(), state);

    ResourceManager::mChargerBoostMutex.lock();
    if (rm->getChargerOnlineState()) {
        if (rm->getConcurrentBoostState() ^ state)
            status = rm->chargerListenerSetBoostState(state, CHARGER_ON_PB_STARTS);

        if (0 != status)
            PAL_ERR(LOG_TAG, "Failed to notify PMIC: %d", status);
    }

    PAL_DBG(LOG_TAG, "Exit concurrency state %d with status %d",
            rm->getConcurrentBoostState(), status);
    ResourceManager::mChargerBoostMutex.unlock();
exit:
    return status;
}

/* Handle case when charging going on and PB starts on speaker*/
int SessionAR::EnableChargerConcurrency(std::shared_ptr<ResourceManager>rm, Stream *s)
{
    int status = -EINVAL;

    if (!rm)
        goto exit;

    PAL_DBG(LOG_TAG, "Enter concurrency state %d", rm->getConcurrentBoostState());

    if ((s && rm->getChargerOnlineState()) &&
        (rm->getConcurrentBoostState())) {
         status = rm->setSessionParamConfig(PAL_PARAM_ID_CHARGER_STATE, s,
                                               CHARGE_CONCURRENCY_ON_TAG);
        if (0 != status) {
            PAL_DBG(LOG_TAG, "Set SessionParamConfig with status %d", status);
            status = rm->chargerListenerSetBoostState(false, CHARGER_ON_PB_STARTS);
            if (0 != status)
                PAL_ERR(LOG_TAG, "Failed to notify PMIC: %d", status);
        }
    }
    PAL_DBG(LOG_TAG, "Exit concurrency state %d with status %d",
            rm->getConcurrentBoostState(), status);
exit:
    return status;
}

int32_t SessionAR::setInitialVolume() {
    int32_t status = 0;
    struct volume_set_param_info vol_set_param_info = {};
    uint16_t volSize = 0;
    uint8_t *volPayload = nullptr;
    struct pal_stream_attributes sAttr = {};
    bool isStreamAvail = false;
    struct pal_vol_ctrl_ramp_param ramp_param = {};
    Session *session = NULL;
    bool forceSetParameters = false;

    PAL_DBG(LOG_TAG, "Enter status: %d", status);

    if (!streamHandle) {
        PAL_ERR(LOG_TAG, "streamHandle is invalid");
        goto exit;
    }
    status = streamHandle->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG, "stream get attributes failed");
        goto exit;
    }

    for (int32_t i = 0; streamHandle->mVolumeData &&
        i < (streamHandle->mVolumeData->no_of_volpair); i++) {
        if((i > 0) &&
            (abs(streamHandle->mVolumeData->volume_pair[0].vol -
                streamHandle->mVolumeData->volume_pair[i].vol) > VOLUME_TOLERANCE)) {
                forceSetParameters = true;
                break;
        }
    }

    memset(&vol_set_param_info, 0, sizeof(struct volume_set_param_info));
    rm->getVolumeSetParamInfo(&vol_set_param_info);
    isStreamAvail = (find(vol_set_param_info.streams_.begin(),
                vol_set_param_info.streams_.end(), sAttr.type) !=
                vol_set_param_info.streams_.end());
    if ((isStreamAvail && vol_set_param_info.isVolumeUsingSetParam) || forceSetParameters) {
        if (sAttr.direction == PAL_AUDIO_OUTPUT) {
           /* DSP default volume is highest value, non-0 rampping period
            * brings volume burst from highest amplitude to new volume
            * at the begining, that makes pop noise heard.
            * set ramp period to 0 ms before pcm_start only for output,
            * so desired volume can take effect instantly at the begining.
            */
            ramp_param.ramp_period_ms = 0;
            status = setParameters(streamHandle, TAG_STREAM_VOLUME,
                                   PAL_PARAM_ID_VOLUME_CTRL_RAMP, &ramp_param);
        }
        // apply if there is any cached volume
        if (streamHandle->mVolumeData) {
            volSize = (sizeof(struct pal_volume_data) +
                      (sizeof(struct pal_channel_vol_kv) *
                      (streamHandle->mVolumeData->no_of_volpair)));
            volPayload = new uint8_t[sizeof(pal_param_payload) +
                volSize]();
            pal_param_payload *pld = (pal_param_payload *)volPayload;
            pld->payload_size = sizeof(struct pal_volume_data);
            memcpy(pld->payload, streamHandle->mVolumeData, volSize);
            status = setParameters(streamHandle, TAG_STREAM_VOLUME,
                    PAL_PARAM_ID_VOLUME_USING_SET_PARAM, (void *)pld);
            delete[] volPayload;
        }
        if (sAttr.direction == PAL_AUDIO_OUTPUT) {
            //set ramp period back to default.
            ramp_param.ramp_period_ms = DEFAULT_RAMP_PERIOD;
            status = setParameters(streamHandle, TAG_STREAM_VOLUME,
                                   PAL_PARAM_ID_VOLUME_CTRL_RAMP, &ramp_param);
        }
    } else {
        // Setting the volume as in stream open, no default volume is set.
        if (sAttr.type != PAL_STREAM_ACD &&
            sAttr.type != PAL_STREAM_VOICE_UI &&
            sAttr.type != PAL_STREAM_CONTEXT_PROXY &&
            sAttr.type != PAL_STREAM_ULTRASOUND &&
            sAttr.type != PAL_STREAM_SENSOR_PCM_DATA &&
            sAttr.type != PAL_STREAM_HAPTICS &&
            sAttr.type != PAL_STREAM_COMMON_PROXY) {

            if (setConfig(streamHandle, CALIBRATION, TAG_STREAM_VOLUME) != 0) {
                PAL_ERR(LOG_TAG,"Setting volume failed");
            }
        }
    }

exit:
    PAL_DBG(LOG_TAG, "Exit status: %d", status);
    return status;
}

Session* SessionAR::makeACDBSession(const std::shared_ptr<ResourceManager>& rm,
                                    const struct pal_stream_attributes *sAttr)
{
    if (!rm || !sAttr) {
        PAL_ERR(LOG_TAG,"Invalid parameters passed");
        return nullptr;
    }

    Session* s = (Session*) nullptr;
    s = new SessionAgm(rm);

    return s;
}

int SessionAR::rwACDBParameters(void *payload, uint32_t sampleRate,
                                bool isParamWrite)
{
    int status = 0;
    uint8_t *payloadData = NULL;
    size_t payloadSize = 0;
    uint32_t miid = 0;
    char const *control = "setParamTagACDB";
    struct mixer_ctl *ctl = NULL;
    pal_effect_custom_payload_t *effectCustomPayload = nullptr;
    PayloadBuilder builder;
    pal_param_payload *paramPayload = nullptr;
    agm_acdb_param *effectACDBPayload = nullptr;

    paramPayload = (pal_param_payload *)payload;
    if (!paramPayload)
        return -EINVAL;

    effectACDBPayload = (agm_acdb_param *)(paramPayload->payload);
    if (!effectACDBPayload)
        return -EINVAL;

    PAL_DBG(LOG_TAG, "Enter.");

    status = getModuleInfo(control, effectACDBPayload->tag, &miid, &ctl, NULL);
    if (status || !miid) {
        PAL_ERR(LOG_TAG, "failed to look for module with tagID 0x%x, status = %d",
                    effectACDBPayload->tag, status);
        status = -EINVAL;
        goto exit;
    }

    effectCustomPayload =
        (pal_effect_custom_payload_t *)(effectACDBPayload->blob);

    status = builder.payloadACDBParam(&payloadData, &payloadSize,
                            (uint8_t *)effectACDBPayload,
                            miid, sampleRate);
    if (!payloadData) {
        PAL_ERR(LOG_TAG, "failed to create payload data.");
        goto exit;
    }

   if (isParamWrite) {
        status = mixer_ctl_set_array(ctl, payloadData, payloadSize);
        if (0 != status) {
            PAL_ERR(LOG_TAG, "Set custom config failed, status = %d", status);
            goto exit;
        }
    }

exit:
    ctl = NULL;
    free(payloadData);
    PAL_ERR(LOG_TAG, "Exit. status %d", status);
    return status;
}

int SessionAR::rwACDBParamTunnel(void *payload, pal_device_id_t palDeviceId,
                        pal_stream_type_t palStreamType, uint32_t sampleRate,
                        uint32_t instanceId, bool isParamWrite, Stream * s)
{
    int status = -EINVAL;
    struct pal_stream_attributes sAttr = {};

    PAL_DBG(LOG_TAG, "Enter");
    status = s->getStreamAttributes(&sAttr);
    streamHandle = s;
    if (0 != status) {
        PAL_ERR(LOG_TAG,"getStreamAttributes Failed \n");
        goto exit;
    }

    PAL_INFO(LOG_TAG, "PAL device id=0x%x", palDeviceId);
    status = SessionAlsaUtils::rwACDBTunnel(s, rm, palDeviceId, payload, isParamWrite, instanceId);
    if (status) {
        PAL_ERR(LOG_TAG, "session alsa open failed with %d", status);
    }

exit:
    PAL_DBG(LOG_TAG, "Exit status: %d", status);
    return status;
}

int SessionAR::HDRConfigKeyToDevOrientation(const char* hdr_custom_key)
{
    if (!strcmp(hdr_custom_key, "unprocessed-hdr-mic-portrait"))
        return ORIENTATION_0;
    else if (!strcmp(hdr_custom_key, "unprocessed-hdr-mic-landscape"))
        return ORIENTATION_90;
    else if (!strcmp(hdr_custom_key, "unprocessed-hdr-mic-inverted-portrait"))
        return ORIENTATION_180;
    else if (!strcmp(hdr_custom_key, "unprocessed-hdr-mic-inverted-landscape"))
        return ORIENTATION_270;

    PAL_DBG(LOG_TAG,"unknown device orientation %s for HDR record",hdr_custom_key);
    return ORIENTATION_0;
}

