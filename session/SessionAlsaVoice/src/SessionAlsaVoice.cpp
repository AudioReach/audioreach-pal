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
 */

/*
Changes from Qualcomm Technologies, Inc. are provided under the following license:
Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
SPDX-License-Identifier: BSD-3-Clause-Clear
*/


#define LOG_TAG "PAL: SessionAlsaVoice"

#include "SessionAlsaVoice.h"
#include "SessionAlsaUtils.h"
#include "Stream.h"
#include "ResourceManager.h"
#include "apm_api.h"
#include <sstream>
#include <string>
#include <agm/agm_api.h>
#ifdef PAL_CUTILS_SUPPORTED
#include <cutils/properties.h>
#endif
#ifdef FEATURE_IPQ_OPENWRT
#include "audio_route.h"
#include <stdarg.h>
#include <err.h>
#else
#include "audio_route/audio_route.h"
#endif

#define PAL_PADDING_8BYTE_ALIGN(x)  ((((x) + 7) & 7) ^ 7)
#define MAX_VOL_INDEX 5
#define MIN_VOL_INDEX 0
#define percent_to_index(val, min, max) \
            ((val) * ((max) - (min)) * 0.01 + (min) + .5)

#define NUM_OF_CAL_KEYS 3

static uint32_t retries = 0;

extern "C" Session* CreateVoiceSession(const std::shared_ptr<ResourceManager> rm) {
    return new SessionAlsaVoice(rm);
}

SessionAlsaVoice::SessionAlsaVoice(std::shared_ptr<ResourceManager> Rm)
{
   rm = Rm;
   builder = new PayloadBuilder();
   streamHandle = NULL;
   pcmRx = NULL;
   pcmTx = NULL;

   max_vol_index = rm->getMaxVoiceVol();
   if (max_vol_index == -1){
      max_vol_index = MAX_VOL_INDEX;
   }
   mState = SESSION_IDLE;
}

SessionAlsaVoice::~SessionAlsaVoice()
{
   delete builder;

}

void SessionAlsaVoice::HandleRxDtmfCallBack(uint64_t hdl, uint32_t event_id,
                                          void *data, uint32_t event_size)
{
    pal_event_dtmf_detect_data event_data;
    pal_stream_callback cb;
    struct dtmf_detect_event_t *dtmf_info = nullptr;
    Stream *s = NULL;

    PAL_ERR(LOG_TAG, "Enter");

    if ((hdl == 0) || !data || !event_size) {
        PAL_ERR(LOG_TAG, "Invalid stream handle or event data or event size");
        return;
    }
    PAL_ERR(LOG_TAG, "Enter, event detected on SPF, event id = 0x%x", event_id);
    if (event_id != EVENT_ID_DTMF_DETECTION) {
        return;
    }
    PAL_ERR(LOG_TAG, "EVENT_ID_DTMF_DETECTION detected on SPF, event id = 0x%x", event_id);
    dtmf_info = (struct dtmf_detect_event_t *)data;
    PAL_ERR(LOG_TAG, "high_freq: %d, low_freq: %d",
            dtmf_info->tone_high_freq, dtmf_info->tone_low_freq);
    s = reinterpret_cast<Stream *>(hdl);

    event_data.dir            = PAL_AUDIO_OUTPUT;
    event_data.dtmf_high_freq = dtmf_info->tone_high_freq;
    event_data.dtmf_low_freq = dtmf_info->tone_low_freq;
    PAL_ERR(LOG_TAG, "high_freq: %d, low_freq: %d",
            event_data.dtmf_high_freq, event_data.dtmf_low_freq);

    if (s->getCallBack(&cb) == 0) {
        if (cb) {
            PAL_ERR(LOG_TAG, "found callback");
             cb(reinterpret_cast<pal_stream_handle_t *>(s), PAL_STREAM_CBK_EVENT_DTMF_DETECTION, (uint32_t *)&event_data,
                event_size, s->cookie);
        }
    }

    PAL_ERR(LOG_TAG, "Exit");
    return;
}

void SessionAlsaVoice::HandleTxDtmfCallBack(uint64_t hdl, uint32_t event_id,
                                          void *data, uint32_t event_size)
{
    pal_event_dtmf_detect_data event_data;
    pal_stream_callback cb;
    struct dtmf_detect_event_t *dtmf_info = nullptr;
    Stream *s = NULL;

    PAL_ERR(LOG_TAG, "Enter");

    if ((hdl == 0) || !data || !event_size) {
        PAL_ERR(LOG_TAG, "Invalid stream handle or event data or event size");
        return;
    }
    PAL_ERR(LOG_TAG, "Enter, event detected on SPF, event id = 0x%x", event_id);
    if (event_id != EVENT_ID_DTMF_DETECTION) {
        return;
    }
    PAL_ERR(LOG_TAG, "EVENT_ID_DTMF_DETECTION detected on SPF, event id = 0x%x", event_id);
    dtmf_info = (struct dtmf_detect_event_t *)data;
    PAL_ERR(LOG_TAG, "high_freq: %d, low_freq: %d",
            dtmf_info->tone_high_freq, dtmf_info->tone_low_freq);
    s = reinterpret_cast<Stream *>(hdl);

    event_data.dir            = PAL_AUDIO_INPUT;
    event_data.dtmf_high_freq = dtmf_info->tone_high_freq;
    event_data.dtmf_low_freq = dtmf_info->tone_low_freq;
    PAL_ERR(LOG_TAG, "high_freq: %d, low_freq: %d",
            event_data.dtmf_high_freq, event_data.dtmf_low_freq);

    if (s->getCallBack(&cb) == 0) {
        if (cb) {
            PAL_ERR(LOG_TAG, "found callback");
             cb(reinterpret_cast<pal_stream_handle_t *>(s), PAL_STREAM_CBK_EVENT_DTMF_DETECTION, (uint32_t *)&event_data,
                event_size, s->cookie);
        }
    }

    PAL_ERR(LOG_TAG, "Exit");
    return;
}

int SessionAlsaVoice::registerDtmfEvent(int tagId, int dir) {
    int status = 0;
    int payload_size = 0;
    struct agm_event_reg_cfg *event_cfg = NULL;

    PAL_DBG(LOG_TAG, "Enter");

    payload_size = sizeof(struct agm_event_reg_cfg);
    event_cfg = (struct agm_event_reg_cfg *)calloc(1, payload_size);
    if (!event_cfg) {
        PAL_ERR(LOG_TAG, "Failed to allocate memory for event_cfg");
        status = -ENOMEM;
    } else {
        event_cfg->event_id = EVENT_ID_DTMF_DETECTION;
        event_cfg->event_config_payload_size = 0;

        if (tagId == DTMF_DETECT_ENABLE) {
            PAL_ERR(LOG_TAG, "Enter with tagID:%d dir %d", tagId, dir);
            event_cfg->is_register = 1;
        } else {
            PAL_ERR(LOG_TAG, "Enter with tagID:%d dir %d", tagId, dir);
            event_cfg->is_register = 0;
        }

        if (dir == PAL_AUDIO_INPUT) {
            status = SessionAlsaUtils::registerMixerEvent(mixer, pcmDevTxIds.at(0),
            txAifBackEnds[0].second.data(), DTMF_DETECTOR, (void *)event_cfg,
            payload_size);
        } else {
            status = SessionAlsaUtils::registerMixerEvent(mixer, pcmDevRxIds.at(0),
            rxAifBackEnds[0].second.data(), DTMF_DETECTOR, (void *)event_cfg,
            payload_size);
        }
        if (status != 0) {
            PAL_ERR(LOG_TAG,"registerMixerEvent failed");
        }
    }
    if (event_cfg) {
        free(event_cfg);
        event_cfg = NULL;
    }
    PAL_DBG(LOG_TAG, "Exit");
    return status;
}

struct mixer_ctl* SessionAlsaVoice::getFEMixerCtl(const char *controlName, int *device, pal_stream_direction_t dir)
{
    std::ostringstream CntrlName;
    struct mixer_ctl *ctl;
    char *stream = (char*)"VOICEMMODE1p";

    if (dir == PAL_AUDIO_OUTPUT) {
        if (pcmDevRxIds.size()) {
            *device = pcmDevRxIds.at(0);
        } else {
            PAL_ERR(LOG_TAG, "frontendIDs is not available.");
            return NULL;
        }
    } else if (dir == PAL_AUDIO_INPUT) {
        if (pcmDevTxIds.size()) {
            *device = pcmDevTxIds.at(0);
        } else {
            PAL_ERR(LOG_TAG, "frontendIDs is not available.");
            return NULL;
        }
    }

    if (vsid == VOICEMMODE1 ||
        vsid == VOICELBMMODE1) {
        if (dir == PAL_AUDIO_INPUT) {
            stream = (char*)"VOICEMMODE1c";
        } else {
            stream = (char*)"VOICEMMODE1p";
        }
    } else {
        if (dir == PAL_AUDIO_INPUT) {
            stream = (char*)"VOICEMMODE2c";
        } else {
            stream = (char*)"VOICEMMODE2p";
        }
    }

    CntrlName << stream << " " << controlName;
    ctl = mixer_get_ctl_by_name(mixer, CntrlName.str().data());
    if (!ctl) {
        PAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", CntrlName.str().data());
        return NULL;
    }

    return ctl;
}

int32_t SessionAlsaVoice::getFrontEndIds(std::vector<int>& devices, uint32_t ldir) const
{
    int32_t status = 0;
    switch(ldir) {
        case RX_HOSTLESS:
            if (pcmDevRxIds.size()) {
                devices = pcmDevRxIds;
                goto exit;
            }
            break;
        case TX_HOSTLESS:
            if (pcmDevTxIds.size()) {
                devices = pcmDevTxIds;
                goto exit;
            }
            break;
        default:
            break;
    }
    status = -EINVAL;
exit:
    return status;
}

bool SessionAlsaVoice::isActive()
{
    PAL_VERBOSE(LOG_TAG, "state = %d", mState);
    return mState == SESSION_STARTED;
}

uint32_t SessionAlsaVoice::getMIID(const char *backendName, uint32_t tagId, uint32_t *miid)
{
    int status = 0;
    int device = 0;

    switch (tagId) {
    case DEVICE_HW_ENDPOINT_TX:
    case BT_PLACEHOLDER_DECODER:
    case COP_DEPACKETIZER_V2:
    case TAG_ECNS:
        if (pcmDevTxIds.size()) {
            device = pcmDevTxIds.at(0);
        } else {
            PAL_ERR(LOG_TAG, "pcmDevTxIds:%x is not available.",tagId);
            return -EINVAL;
        }
        break;
    case DEVICE_HW_ENDPOINT_RX:
    case BT_PLACEHOLDER_ENCODER:
    case COP_PACKETIZER_V2:
    case COP_PACKETIZER_V0:
    case TAG_DEVICE_PP_MFC:
    case MODULE_SP:
        if (pcmDevRxIds.size()) {
            device = pcmDevRxIds.at(0);
        } else {
            PAL_ERR(LOG_TAG, "pcmDevRxIds:%x is not available.",tagId);
            return -EINVAL;
        }
        break;
    case RAT_RENDER:
    case BT_PCM_CONVERTER:
        if(strstr(backendName,"TX")) {
          if (pcmDevTxIds.size()) {
              device = pcmDevTxIds.at(0);
          } else {
              PAL_ERR(LOG_TAG, "pcmDevTxIds:%x is not available.",tagId);
              return -EINVAL;
          }
        } else {
           if (pcmDevRxIds.size()) {
               device = pcmDevRxIds.at(0);
           } else {
               PAL_ERR(LOG_TAG, "pcmDevRxIds:%x is not available.",tagId);
               return -EINVAL;
           }
        }
        break;
    default:
        PAL_INFO(LOG_TAG, "Unsupported tag info %x",tagId);
        return -EINVAL;
    }

    status = SessionAlsaUtils::getModuleInstanceId(mixer, device,
                                                   backendName,
                                                   tagId, miid);
    if (0 != status)
        PAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", tagId, status);

    return status;
}


int SessionAlsaVoice::prepare(Stream * s __unused)
{
   return 0;
}

int SessionAlsaVoice::open(Stream * s)
{
    int status = -EINVAL;
    struct pal_stream_attributes sAttr;
    std::vector<std::shared_ptr<Device>> associatedDevices;

    PAL_DBG(LOG_TAG,"Enter");
    status = s->getStreamAttributes(&sAttr);
    streamHandle = s;
    if(0 != status) {
        PAL_ERR(LOG_TAG,"getStreamAttributes Failed \n");
        goto exit;
    }

    status = s->getAssociatedDevices(associatedDevices);
    if(0 != status) {
        PAL_ERR(LOG_TAG,"getAssociatedDevices Failed \n");
        goto exit;
    }
    /*check to allow CRS SVA concurrency*/
    struct pal_device deviceAttribute;
    for (int32_t i = 0; i < associatedDevices.size(); i++) {
        status = associatedDevices[i]->getDeviceAttributes(&deviceAttribute, s);
        if (status) {
            PAL_ERR(LOG_TAG, "getDeviceAttributes failed with status %d", status);
            goto exit;
        }
        PAL_INFO(LOG_TAG, "device custom key=%s",
                        deviceAttribute.custom_config.custom_key);
        if (!strncmp(deviceAttribute.custom_config.custom_key,
                    "crsCall", sizeof("crsCall"))) {
                PAL_INFO(LOG_TAG, "setting RM CRS")
                rm->setCRSCallEnabled(true);
        }
    }

    if (sAttr.direction != (PAL_AUDIO_INPUT|PAL_AUDIO_OUTPUT)) {
        PAL_ERR(LOG_TAG,"Voice session dir must be input and output");
        goto exit;
    }

    status = allocateFrontEndIds(sAttr, RX_HOSTLESS);
    status = allocateFrontEndIds(sAttr, TX_HOSTLESS);
    if (!pcmDevRxIds.size() || !pcmDevTxIds.size()) {
        if (pcmDevRxIds.size()) {
            freeFrontEndIds(sAttr, RX_HOSTLESS);
            pcmDevRxIds.clear();
        }
        if (pcmDevTxIds.size()) {
            freeFrontEndIds(sAttr, TX_HOSTLESS);
            pcmDevTxIds.clear();
        }
        PAL_ERR(LOG_TAG, "allocateFrontEndIds failed");
        status = -EINVAL;
        goto exit;
    }

    vsid = sAttr.info.voice_call_info.VSID;
    ttyMode = sAttr.info.voice_call_info.tty_mode;

    rm->getBackEndNames(associatedDevices, rxAifBackEnds, txAifBackEnds);

    if (txAifBackEnds.empty()) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "no TX backend specified for this stream\n");
        goto exit;
    }

    if (rxAifBackEnds.empty()) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "no RX backend specified for this stream\n");
        goto exit;
    }

    status = rm->getVirtualAudioMixer(&mixer);
    if (status) {
        PAL_ERR(LOG_TAG,"mixer error");
        freeFrontEndIds(sAttr, RX_HOSTLESS);
        freeFrontEndIds(sAttr, TX_HOSTLESS);
        pcmDevRxIds.clear();
        pcmDevTxIds.clear();
        goto exit;
    }

    status = SessionAlsaUtils::open(s, rm, pcmDevRxIds, pcmDevTxIds,
                                    rxAifBackEnds, txAifBackEnds);

    if (status) {
        PAL_ERR(LOG_TAG, "session alsa open failed with %d", status);
        freeFrontEndIds(sAttr, RX_HOSTLESS);
        freeFrontEndIds(sAttr, TX_HOSTLESS);
        pcmDevRxIds.clear();
        pcmDevTxIds.clear();
        goto exit;
    }

    if (sAttr.type == PAL_STREAM_VOICE_CALL){
        PAL_DBG(LOG_TAG, "before registerMixerEventCallback");
        registerRxCallBack(HandleRxDtmfCallBack, (uint64_t)s);

        status = rm->registerMixerEventCallback(pcmDevRxIds,
            sessionRxCb, rxCbCookie, true);
        if (status != 0) {
            PAL_ERR(LOG_TAG, "Failed to register callback to rm for RX");
        }

        registerTxCallBack(HandleTxDtmfCallBack, (uint64_t)s);

        status = rm->registerMixerEventCallback(pcmDevTxIds,
            sessionTxCb, txCbCookie, true);
        if (status != 0) {
            PAL_ERR(LOG_TAG, "Failed to register callback to rm for TX");
        }
        PAL_DBG(LOG_TAG, "after registerMixerEventCallback for DTMF RX/TX");
    }

exit:
    PAL_DBG(LOG_TAG,"Exit ret: %d", status);
    return status;
}

int SessionAlsaVoice::getDeviceChannelInfo(Stream *s, uint16_t *channels)
{
    int status = 0;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    struct pal_device dAttr;
    int dev_id = 0;
    int idx = 0;

    memset(&dAttr, 0, sizeof(struct pal_device));

    status = s->getAssociatedDevices(associatedDevices);
    if ((0 != status) || (associatedDevices.size() == 0)) {
        PAL_ERR(LOG_TAG, "getAssociatedDevices fails or empty associated devices");
        goto exit;
    }

    rm->getBackEndNames(associatedDevices, rxAifBackEnds, txAifBackEnds);
    if (rxAifBackEnds.empty() && txAifBackEnds.empty()) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "no backend specified for this stream");
        return status;
    }

    for (idx = 0; idx < associatedDevices.size(); idx++) {
        dev_id = associatedDevices[idx]->getSndDeviceId();
        if (rm->isInputDevId(dev_id)) {
            status = associatedDevices[idx]->getDeviceAttributes(&dAttr);
            break;
        }
    }

    if (idx >= associatedDevices.size() || dAttr.id <= PAL_DEVICE_IN_MIN ||
            dAttr.id >= PAL_DEVICE_IN_MAX) {
        PAL_ERR(LOG_TAG, "Failed to get device attributes");
        status = -EINVAL;
        goto exit;
    }

    if (dAttr.id == PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET || dAttr.id == PAL_DEVICE_IN_BLUETOOTH_BLE)
    {
        struct pal_media_config codecConfig;
        status = associatedDevices[idx]->getCodecConfig(&codecConfig);
        if(0 != status) {
            PAL_ERR(LOG_TAG,"getCodecConfig Failed \n");
            goto exit;
        }
        *channels = codecConfig.ch_info.channels;
        PAL_DBG(LOG_TAG,"set devicePPMFC to match codec configuration for %d\n", dAttr.id);
    } else {
        *channels = dAttr.config.ch_info.channels;
    }

exit:
    return status;
}

int SessionAlsaVoice::start(Stream * s)
{
    int32_t status = 0;
    int ret = 0;
    struct pcm_config config;
    struct pal_stream_attributes sAttr;
    std::shared_ptr<Device> rxDevice = nullptr;
    pal_param_payload *palPayload = NULL;
    int txDevId = PAL_DEVICE_NONE;
    struct pal_volume_data *volume = NULL;
    bool isTxStarted = false, isRxStarted = false;
    void* plugin = nullptr;
    PluginConfig pluginConfig = nullptr;

    PAL_DBG(LOG_TAG,"Enter");

    rm->voteSleepMonitor(s, true);

    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG,"stream get attributes failed");
        goto exit;
    }

    if (mState == SESSION_IDLE) {
        s->getBufInfo(&in_buf_size,&in_buf_count,&out_buf_size,&out_buf_count);
        memset(&config, 0, sizeof(config));

        config.rate = sAttr.out_media_config.sample_rate;
        if (sAttr.out_media_config.bit_width == 32)
            config.format = PCM_FORMAT_S32_LE;
        else if (sAttr.out_media_config.bit_width == 24)
            config.format = PCM_FORMAT_S24_3LE;
        else if (sAttr.out_media_config.bit_width == 16)
            config.format = PCM_FORMAT_S16_LE;
        config.channels = sAttr.out_media_config.ch_info.channels;
        config.period_size = out_buf_size;
        config.period_count = out_buf_count;
        config.start_threshold = 0;
        config.stop_threshold = 0;
        config.silence_threshold = 0;

        /*setup external ec if needed*/
        status = getRXDevice(s, rxDevice);
        if (status) {
            PAL_ERR(LOG_TAG, "failed, could not find associated RX device");
            goto exit;
        }
        setExtECRef(s, rxDevice, true);

        pcmRx = pcm_open(rm->getVirtualSndCard(), pcmDevRxIds.at(0), PCM_OUT, &config);
        if (!pcmRx) {
            PAL_ERR(LOG_TAG, "Exit pcm-rx open failed");
            status = -EINVAL;
            goto err_pcm_open;
        }

        if (!pcm_is_ready(pcmRx)) {
            PAL_ERR(LOG_TAG, "Exit pcm-rx open not ready");
            status = -EINVAL;
            goto err_pcm_open;
        }

        config.rate = sAttr.in_media_config.sample_rate;
        if (sAttr.in_media_config.bit_width == 32)
            config.format = PCM_FORMAT_S32_LE;
        else if (sAttr.in_media_config.bit_width == 24)
            config.format = PCM_FORMAT_S24_3LE;
        else if (sAttr.in_media_config.bit_width == 16)
            config.format = PCM_FORMAT_S16_LE;
        config.channels = sAttr.in_media_config.ch_info.channels;
        config.period_size = in_buf_size;
        config.period_count = in_buf_count;

        pcmTx = pcm_open(rm->getVirtualSndCard(), pcmDevTxIds.at(0), PCM_IN, &config);
        if (!pcmTx) {
            PAL_ERR(LOG_TAG, "Exit pcm-tx open failed");
            status = -EINVAL;
            goto err_pcm_open;
        }

        if (!pcm_is_ready(pcmTx)) {
            PAL_ERR(LOG_TAG, "Exit pcm-tx open not ready");
            status = -EINVAL;
            goto err_pcm_open;
        }
    }

    /*set tty mode*/
    if (ttyMode) {
        palPayload = (pal_param_payload *)calloc(1,
                                 sizeof(pal_param_payload) + sizeof(ttyMode));
        if(palPayload != NULL){
            palPayload->payload_size = sizeof(ttyMode);
            *(palPayload->payload) = ttyMode;
            setParamWithTag(s, TTY_MODE, PAL_PARAM_ID_TTY_MODE, palPayload);
        }
    }

    try {
        pm = PluginManager::getInstance();
        if(!pm){
            PAL_ERR(LOG_TAG, "unable to get plugin manager instance");
            goto exit;
        }
        status = pm->openPlugin(PAL_PLUGIN_MANAGER_CONFIG, streamNameLUT.at(sAttr.type), plugin);
        if (plugin && !status) {
            pluginConfig = reinterpret_cast<PluginConfig>(plugin);
            uint32_t vsidCopy = vsid;
            ret = pluginConfig(s, PAL_PLUGIN_CONFIG_START, reinterpret_cast<void*>(&vsidCopy), sizeof(vsidCopy));
            if (ret) {
                PAL_ERR(LOG_TAG, "Config Plugin Unsuccessful.");
            }
        } else {
            PAL_ERR(LOG_TAG, "unable to get plugin for stream type %s", streamNameLUT.at(sAttr.type).c_str());
        }
    } catch (const std::exception& e) {
        throw std::runtime_error(e.what());
    }

    if (status != 0) {
        PAL_ERR(LOG_TAG,"setMixerParameter failed");
        goto err_pcm_open;
    }

    volume = (struct pal_volume_data *)malloc(sizeof(uint32_t) +
                                                (sizeof(struct pal_channel_vol_kv)));
    if (!volume) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG, "volume malloc failed %s", strerror(errno));
        goto err_pcm_open;
    }

    /*if no volume is set set a default volume*/
    if ((s->getVolumeData(volume))) {
        PAL_INFO(LOG_TAG, "no volume set, setting default vol to %f",
                 default_volume);
        volume->no_of_volpair = 1;
        volume->volume_pair[0].channel_mask = 1;
        volume->volume_pair[0].vol = default_volume;
        /*call will cache the volume but not apply it as stream has not moved to start state*/
        s->setVolume(volume);
    };
    /*call to apply volume*/
    if (rm->IsCRSCallEnabled()) {
        setConfig(s, MODULE, CRS_CALL_VOLUME, RX_HOSTLESS);
    } else {
        setConfig(s, CALIBRATION, TAG_STREAM_VOLUME, RX_HOSTLESS);
    }

    if (ResourceManager::isLpiLoggingEnabled()) {
        status = payloadTaged(s, MODULE, LPI_LOGGING_ON, pcmDevTxIds.at(0), TX_HOSTLESS);
        if (status)
            PAL_ERR(LOG_TAG, "Failed to set data logging param status = %d", status);
    }

    if (rm->IsChargeConcurrencyEnabled()) {
        if (PAL_DEVICE_OUT_SPEAKER == rxDevice->getSndDeviceId()) {
            status = NotifyChargerConcurrency(rm, true);
            if (0 == status) {
                status = EnableChargerConcurrency(rm, s);
                //Handle failure case of ICL config
                if (0 != status) {
                    PAL_DBG(LOG_TAG, "Failed to set ICL Config status %d", status);
                    status = NotifyChargerConcurrency(rm, false);
                }
            }
            status = 0;
        }
    }

    status = pcm_start(pcmRx);
    if (status) {
        PAL_ERR(LOG_TAG, "pcm_start rx failed %d", status);
        goto err_pcm_open;
    }
   isRxStarted = true;

    status = pcm_start(pcmTx);
    if (status) {
        PAL_ERR(LOG_TAG, "pcm_start tx failed %d", status);
        goto err_pcm_open;
    }
    isTxStarted = true;

    if (plugin) {
        pluginConfig = reinterpret_cast<PluginConfig>(plugin);
        ReconfigPluginPayload ppld;
        ppld.session = this;
        ppld.builder = reinterpret_cast<void*>(builder);
        ppld.payload = reinterpret_cast<void*>(&rxDevice);
        ppld.pcmDevIds = pcmDevRxIds;
        ret = pluginConfig(s, PAL_PLUGIN_CONFIG_POST_START, reinterpret_cast<void*>(&ppld), sizeof(ReconfigPluginPayload));
        if (ret) {
            PAL_ERR(LOG_TAG, "Config Plugin Unsuccessful.");
        }
    } else {
        PAL_ERR(LOG_TAG, "unable to get plugin for stream type %s", streamNameLUT.at(sAttr.type).c_str());
    }

    /*set sidetone*/
    if (sideTone_cnt == 0) {
        status = getTXDeviceId(s, &txDevId);
        if (status){
            PAL_ERR(LOG_TAG, "could not find TX device associated with this stream cannot set sidetone");
            goto err_pcm_open;
        } else {
            status = setSidetone(txDevId,s,1);
            if(0 != status) {
               PAL_ERR(LOG_TAG,"enabling sidetone failed \n");
            }
        }
    }
    retries = 0;
    status = 0;
    goto exit;

err_pcm_open:
    /*teardown external ec if needed*/
    setExtECRef(s, rxDevice, false);
    if (pcmRx) {
        if (isRxStarted)
            pcm_stop(pcmRx);
        pcm_close(pcmRx);
        pcmRx = NULL;
    }
    if (pcmTx) {
        if (isTxStarted)
            pcm_stop(pcmTx);
        pcm_close(pcmTx);
        pcmTx = NULL;
    }
    mState = SESSION_STARTED;

exit:
    builder->freeCustomPayload();
    if (palPayload) {
        free(palPayload);
    }
    if (volume)
        free(volume);
    if (status)
        rm->voteSleepMonitor(s, false);
    PAL_DBG(LOG_TAG,"Exit ret: %d", status);
    return status;
}

int SessionAlsaVoice::stop(Stream * s)
{
    int status = 0;
    int ret = 0;
    int txDevId = PAL_DEVICE_NONE;
    std::shared_ptr<Device> rxDevice = nullptr;
    pal_stream_attributes sAttr = {};
    PluginConfig pluginConfig = nullptr;
    void* plugin = nullptr;

    PAL_DBG(LOG_TAG,"Enter");
    /*disable sidetone*/
    if (sideTone_cnt > 0) {
        status = getTXDeviceId(s, &txDevId);
        if (status){
            PAL_ERR(LOG_TAG, "could not find TX device associated with this stream cannot set sidetone");
        } else {
            status = setSidetone(txDevId,s,0);
            if(0 != status) {
               PAL_ERR(LOG_TAG,"disabling sidetone failed");
            }
        }
    }
    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG,"stream get attributes failed");
        return status;
    }

    /**config mute on pop suppressor now in config plugin.
     * more AR module-specific logics shall be moved to plugin
     * if ever being added. However, necessary getter/setter of
     * private variables need to be created; and subroutine orders
     * need to be carefully considered.
     */
    try {
        pm = PluginManager::getInstance();
        if(!pm){
            PAL_ERR(LOG_TAG, "unable to get plugin manager instance");
            goto exit;
        }
        status = pm->openPlugin(PAL_PLUGIN_MANAGER_CONFIG, streamNameLUT.at(sAttr.type), plugin);
        if (plugin && !status) {
            pluginConfig = reinterpret_cast<PluginConfig>(plugin);
            ReconfigPluginPayload ppld;
            ppld.session = this;
            ret = pluginConfig(s, PAL_PLUGIN_CONFIG_STOP, reinterpret_cast<void*>(&ppld), sizeof(ReconfigPluginPayload));
            if (ret) {
                PAL_ERR(LOG_TAG, "Config Plugin Unsuccessful.");
            }
        } else {
            PAL_ERR(LOG_TAG, "unable to get plugin for stream type %s", streamNameLUT.at(sAttr.type).c_str());
        }
    }
    catch (const std::exception& e) {
        throw std::runtime_error(e.what());
    }

    if (pcmRx && isActive()) {
        status = pcm_stop(pcmRx);
        if (status) {
            PAL_ERR(LOG_TAG, "pcm_stop - rx failed %d", status);
        }
    }

    if (pcmTx && isActive()) {
        status = pcm_stop(pcmTx);
        if (status) {
            PAL_ERR(LOG_TAG, "pcm_stop - tx failed %d", status);
        }
    }

    /*teardown external ec if needed*/
    status = getRXDevice(s, rxDevice);
    if (status) {
        PAL_ERR(LOG_TAG, "failed, could not find associated RX device");
    } else {
        setExtECRef(s, rxDevice, false);
    }

exit:
    rm->voteSleepMonitor(s, false);
    PAL_DBG(LOG_TAG,"Exit ret: %d", status);
    mState = SESSION_STOPPED;

    return status;
}

int SessionAlsaVoice::close(Stream * s)
{
    int status = 0;
    struct pal_stream_attributes sAttr;
    std::string backendname;
    int32_t beDevId = 0;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    std::vector<std::pair<std::string, int>> freeDeviceMetadata;

    PAL_DBG(LOG_TAG,"Enter");
    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG,"stream get attributes failed");
        return status;
    }

    status = s->getAssociatedDevices(associatedDevices);
    if (status != 0) {
        PAL_ERR(LOG_TAG, "getAssociatedDevices failed\n");
        goto exit;
    }
    freeDeviceMetadata.clear();

    for (auto &dev: associatedDevices) {
         beDevId = dev->getSndDeviceId();
         rm->getBackendName(beDevId, backendname);
         PAL_DBG(LOG_TAG, "backendname %s", backendname.c_str());
         if (dev->getDeviceCount() > 1) {
             PAL_DBG(LOG_TAG, "dev %d still active", beDevId);
             freeDeviceMetadata.push_back(std::make_pair(backendname, 0));
         } else {
             PAL_DBG(LOG_TAG, "dev %d not active", beDevId);
             freeDeviceMetadata.push_back(std::make_pair(backendname, 1));
         }
    }
    status = SessionAlsaUtils::close(s, rm, pcmDevRxIds, pcmDevTxIds,
             rxAifBackEnds, txAifBackEnds, freeDeviceMetadata);

    if (pcmRx) {
        status = pcm_close(pcmRx);
        if (status) {
            PAL_ERR(LOG_TAG, "pcm_close - rx failed %d", status);
        }
    }

    if (pcmTx) {
        status = pcm_close(pcmTx);
        if (status) {
            PAL_ERR(LOG_TAG, "pcm_close - tx failed %d", status);
        }
    }

exit:
    if (pcmDevRxIds.size()) {
        freeFrontEndIds(sAttr, RX_HOSTLESS);
        pcmDevRxIds.clear();
        pcmRx = NULL;
    }
    if (pcmDevTxIds.size()) {
        freeFrontEndIds(sAttr, TX_HOSTLESS);
        pcmDevTxIds.clear();
        pcmTx = NULL;
    }
    PAL_DBG(LOG_TAG,"Exit ret: %d", status);
    mState = SESSION_IDLE;
    return status;
}
int SessionAlsaVoice::setParamWithTag(Stream *s, int tagId, uint32_t param_id __unused, void *payload)
{
    int status = 0;
    int device = 0;
    uint8_t* paramData = NULL;
    size_t paramSize = 0;

    uint32_t tty_mode;
    int mute_dir = RX_HOSTLESS;
    int mute_tag = DEVICE_UNMUTE;
    pal_param_payload *PalPayload = (pal_param_payload *)payload;

    PAL_INFO(LOG_TAG,"Enter setParamWithTag called with tag: %d ", tagId);

    switch (static_cast<uint32_t>(tagId)) {

        case VOICE_VOLUME_BOOST:
            volume_boost = *((bool *)PalPayload->payload);
            status = payloadCalKeys(s, &paramData, &paramSize);
            if (!paramData) {
                status = -ENOMEM;
                PAL_ERR(LOG_TAG, "failed to get payload status %d", status);
                goto exit;
            }
            status = setVoiceMixerParameter(s, mixer, paramData, paramSize,
                                            RX_HOSTLESS);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to set voice params status = %d",
                        status);
            }
            break;

        case VOICE_SLOW_TALK_OFF:
        case VOICE_SLOW_TALK_ON:
            if (pcmDevRxIds.size()) {
                device = pcmDevRxIds.at(0);
            } else {
                PAL_ERR(LOG_TAG, "pcmDevRxIds is not available.");
                status = -EINVAL;
                goto exit;
            }
            slow_talk = *((bool *)PalPayload->payload);
            status = payloadTaged(s, MODULE, tagId, device, RX_HOSTLESS);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to set voice slow_Talk params status = %d",
                        status);
            }
            break;

        case TTY_MODE:
            tty_mode = *((uint32_t *)PalPayload->payload);
            status = payloadSetTTYMode(&paramData, &paramSize,
                                       tty_mode);
            status = setVoiceMixerParameter(s, mixer, paramData, paramSize,
                                            RX_HOSTLESS);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to set voice tty params status = %d",
                        status);
                break;
            }

            if (!paramData) {
                status = -ENOMEM;
                PAL_ERR(LOG_TAG, "failed to get tty payload status %d", status);
                goto exit;
            }
            break;

        case VOICE_HD_VOICE:
            hd_voice = *((bool *)PalPayload->payload);
            status = payloadCalKeys(s, &paramData, &paramSize);
            if (!paramData) {
                status = -ENOMEM;
                PAL_ERR(LOG_TAG, "failed to get payload status %d", status);
                goto exit;
            }
            status = setVoiceMixerParameter(s, mixer, paramData, paramSize,
                                            RX_HOSTLESS);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to set voice params status = %d",
                        status);
            }
            break;
      case DEVICE_MUTE:
            if (pcmDevRxIds.size()) {
                device = pcmDevRxIds.at(0);
            } else {
                PAL_ERR(LOG_TAG, "pcmDevRxIds is not available.");
                status = -EINVAL;
                goto exit;
            }
            dev_mute = *((pal_device_mute_t *)PalPayload->payload);
            if (dev_mute.dir == PAL_AUDIO_INPUT) {
                mute_dir = TX_HOSTLESS;
            }
            if (dev_mute.mute == 1) {
                mute_tag = DEVICE_MUTE;
            }
            PAL_DBG(LOG_TAG, "setting device mute dir %d mute flag %d", mute_dir, mute_tag);
            status = payloadTaged(s, MODULE, mute_tag, device, mute_dir);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to set device mute params status = %d",
                        status);
            }
            break;
      case DTMF_DETECT_ENABLE:
      case DTMF_DETECT_DISABLE:
      {
            pal_param_dtmf_detection_cfg* dtmf_detect_payload;

            dtmf_detect_payload = (pal_param_dtmf_detection_cfg*) PalPayload->payload;
            device = pcmDevRxIds.at(0);

            status = registerDtmfEvent(tagId, dtmf_detect_payload->dir);
            if (status != 0) {
                PAL_ERR(LOG_TAG,"registerDtmfEvent failed");
            }

            status = payloadTaged(s, MODULE, tagId, device, dtmf_detect_payload->dir);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to set dtmf detection params status = %d",
                        status);
            }
            break;
       }
       default:
            PAL_ERR(LOG_TAG,"Failed unsupported tag type %d \n",
                    static_cast<uint32_t>(tagId));
            status = -EINVAL;
            break;
    }

    if (0 != status) {
        PAL_ERR(LOG_TAG,"Failed to set config data");
        goto exit;
    }

    PAL_VERBOSE(LOG_TAG, "%pK - payload and %zu size", paramData , paramSize);

exit:
if (paramData) {
    free(paramData);
}
    PAL_DBG(LOG_TAG,"exit status:%d ", status);
    return status;

}

int SessionAlsaVoice::setConfig(Stream * s, configType type, int tag)
{
    int status = 0;
    int device = 0;
    uint8_t* paramData = NULL;
    size_t paramSize = 0;

    PAL_DBG(LOG_TAG,"Enter setConfig called with tag: %d ", tag);

    switch (static_cast<uint32_t>(tag)) {
        case VOLUME_LVL:
            status = payloadCalKeys(s, &paramData, &paramSize);
            status = SessionAlsaVoice::setVoiceMixerParameter(s, mixer,
                                                           paramData,
                                                           paramSize,
                                                           RX_HOSTLESS);
            if (status) {
               PAL_ERR(LOG_TAG, "Failed to set voice params status = %d",
                     status);
            }
            if (!paramData) {
               status = -ENOMEM;
               PAL_ERR(LOG_TAG, "failed to get payload status %d", status);
               goto exit;
            }
            break;
        case MUTE_TAG:
        case UNMUTE_TAG:
            if (pcmDevTxIds.size()) {
               device = pcmDevTxIds.at(0);
               status = payloadTaged(s, type, tag, device, TX_HOSTLESS);
            } else {
              PAL_ERR(LOG_TAG, "pcmDevTxIds:%x is not available.",tag);
              status = -EINVAL;
            }
            break;
        case CHARGE_CONCURRENCY_ON_TAG:
        case CHARGE_CONCURRENCY_OFF_TAG:
            if (pcmDevRxIds.size()) {
               device = pcmDevRxIds.at(0);
               status = payloadTaged(s, type, tag, device, RX_HOSTLESS);
            } else {
              PAL_ERR(LOG_TAG, "pcmDevRxIds:%x is not available.",tag);
              status = -EINVAL;
            }
            break;
        case CRS_CALL_VOLUME:
            if (pcmDevRxIds.size()) {
               device = pcmDevRxIds.at(0);
               status = payloadTaged(s, type, tag, device, RX_HOSTLESS);
            } else {
               PAL_ERR(LOG_TAG, "pcmDevRxIds is not available.");
               status = -EINVAL;
            }
            break;
        default:
            PAL_ERR(LOG_TAG,"Failed unsupported tag type %d", static_cast<uint32_t>(tag));
            status = -EINVAL;
            break;
    }
    if (0 != status) {
        PAL_ERR(LOG_TAG,"Failed to set config data");
        goto exit;
    }

    PAL_VERBOSE(LOG_TAG, "%pK - payload and %zu size", paramData , paramSize);

exit:
if (paramData) {
    free(paramData);
}
    PAL_DBG(LOG_TAG,"Exit status:%d ", status);
    return status;
}

int SessionAlsaVoice::setConfig(Stream * s, configType type __unused, int tag, int dir)
{
    int status = 0;
    int device = 0;
    uint8_t* paramData = NULL;
    size_t paramSize = 0;
    uint8_t* customPayload = NULL;
    size_t customPayloadSize = 0;

    PAL_DBG(LOG_TAG,"Enter setConfig called with tag: %d ", tag);

    switch (static_cast<uint32_t>(tag)) {

       case TAG_STREAM_VOLUME:
            status = payloadCalKeys(s, &paramData, &paramSize);
            if (status || !paramData) {
                status = -ENOMEM;
                PAL_ERR(LOG_TAG, "failed to get payload status %d", status);
                goto exit;
            }
            status = SessionAlsaVoice::setVoiceMixerParameter(s, mixer,
                                                              paramData,
                                                              paramSize,
                                                              dir);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to set voice params status = %d",
                        status);
            }
            if (!paramData) {
                status = -ENOMEM;
                PAL_ERR(LOG_TAG, "failed to get payload status %d", status);
                goto exit;
            }
            break;

        case MUTE_TAG:
        case UNMUTE_TAG:
            if (pcmDevTxIds.size()) {
                device = pcmDevTxIds.at(0);
                status = payloadTaged(s, type, tag, device, TX_HOSTLESS);
            } else {
                PAL_ERR(LOG_TAG, "pcmDevTxIds:%x is not available.",tag);
                status = -EINVAL;
            }
            break;
        case CHANNEL_INFO:
            status = payloadSetChannelInfo(s, &paramData, &paramSize);
            status = SessionAlsaVoice::setVoiceMixerParameter(s, mixer,
                                                              paramData,
                                                              paramSize,
                                                              dir);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to set voice params status = %d",
                        status);
                break;
            }

            if (!paramData) {
                status = -ENOMEM;
                PAL_ERR(LOG_TAG, "failed to get payload status %d", status);
                goto exit;
            }
            break;

        case CRS_CALL_VOLUME:
            if (pcmDevRxIds.size()) {
               device = pcmDevRxIds.at(0);
               status = payloadTaged(s, type, tag, device, RX_HOSTLESS);
            } else {
               PAL_ERR(LOG_TAG, "pcmDevRxIds is not available.");
               status = -EINVAL;
            }
            break;

        default:
            PAL_ERR(LOG_TAG,"Failed unsupported tag type %d", static_cast<uint32_t>(tag));
            status = -EINVAL;
            break;
    }
    if (0 != status) {
        PAL_ERR(LOG_TAG,"Failed to set config data\n");
        goto exit;
    }

    PAL_VERBOSE(LOG_TAG, "%pK - payload and %zu size", paramData , paramSize);

exit:
    builder->freeCustomPayload();
    builder->freeCustomPayload(&paramData, &paramSize);
    PAL_DBG(LOG_TAG,"Exit status:%d ", status);
    return status;
}

int SessionAlsaVoice::payloadTaged(Stream * s, configType type, int tag,
                                   int device __unused, int dir){
    int status = 0;
    uint32_t tagsent;
    struct agm_tag_config* tagConfig;
    const char *setParamTagControl = "setParamTag";
    struct mixer_ctl *ctl;
    std::ostringstream tagCntrlName;
    int tkv_size = 0;
    const char *stream = SessionAlsaVoice::getMixerVoiceStream(s, dir);
    switch (type) {
        case MODULE:
            tkv.clear();
            status = builder->populateTagKeyVector(s, tkv, tag, &tagsent);
            if (0 != status) {
                PAL_ERR(LOG_TAG,"Failed to set the tag configuration\n");
                goto exit;
            }

            if (tkv.size() == 0) {
                status = -EINVAL;
                goto exit;
            }

            tagConfig = (struct agm_tag_config*)malloc (sizeof(struct agm_tag_config) +
                            (tkv.size() * sizeof(agm_key_value)));

            if(!tagConfig) {
                status = -EINVAL;
                goto exit;
            }

            status = SessionAlsaUtils::getTagMetadata(tagsent, tkv, tagConfig);
            if (0 != status) {
                goto exit;
            }
            tagCntrlName<<stream<<" "<<setParamTagControl;
            ctl = mixer_get_ctl_by_name(mixer, tagCntrlName.str().data());
            if (!ctl) {
                PAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", tagCntrlName.str().data());
                if (tagConfig)
                    free(tagConfig);
                return -ENOENT;
            }

            tkv_size = tkv.size()*sizeof(struct agm_key_value);
            status = mixer_ctl_set_array(ctl, tagConfig, sizeof(struct agm_tag_config) + tkv_size);
            if (status != 0) {
                PAL_ERR(LOG_TAG,"failed to set the tag calibration %d", status);
                goto exit;
            }
            ctl = NULL;
            tkv.clear();
            if (tagConfig) {
                free(tagConfig);
            }
            break;
        default:
            PAL_ERR(LOG_TAG,"invalid type ");
            status = -EINVAL;
    }

exit:
    return status;
}

int SessionAlsaVoice::payloadSetChannelInfo(Stream * s, uint8_t **payload, size_t *size)
{
    int status = 0;
    apm_module_param_data_t* header;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0, padBytes = 0;
    uint8_t *ch_info_pl;
    vcpm_param_id_tx_dev_pp_channel_info_t ch_info_payload;
    uint16_t channels = 0;

    status = getDeviceChannelInfo(s, &channels);
    if (status != 0) {
        PAL_ERR(LOG_TAG,"device get channel info failed");
        return status;
    }

    payloadSize = sizeof(struct apm_module_param_data_t)+
                  sizeof(vcpm_param_id_tx_dev_pp_channel_info_t);
    padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

    payloadInfo = new uint8_t[payloadSize + padBytes]();
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return -EINVAL;
    }
    header = (apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = VCPM_MODULE_INSTANCE_ID;
    header->param_id = VCPM_PARAM_ID_TX_DEV_PP_CHANNEL_INFO;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);

    PAL_DBG(LOG_TAG, "vsid %d num_channels %d", vsid, channels);
    ch_info_payload.vsid = vsid;
    ch_info_payload.num_channels = channels;
    ch_info_pl = (uint8_t*)payloadInfo + sizeof(apm_module_param_data_t);
    ar_mem_cpy(ch_info_pl,  sizeof(vcpm_param_id_tx_dev_pp_channel_info_t),
                     &ch_info_payload,  sizeof(vcpm_param_id_tx_dev_pp_channel_info_t));

    *size = payloadSize + padBytes;
    *payload = payloadInfo;

    return status;
}

int SessionAlsaVoice::payloadCalKeys(Stream * s, uint8_t **payload, size_t *size)
{
    int status = 0;
    apm_module_param_data_t* header;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0, padBytes = 0;
    uint8_t *vol_pl;
    vcpm_param_cal_keys_payload_t cal_keys = {};
    vcpm_ckv_pair_t cal_key_pair[NUM_OF_CAL_KEYS] = {};
    float volume = 0.0;
    int vol;
    struct pal_volume_data *voldata = NULL;

    voldata = (struct pal_volume_data *)calloc(1, (sizeof(uint32_t) +
                      (sizeof(struct pal_channel_vol_kv) * (0xFFFF))));
    if (!voldata) {
        status = -ENOMEM;
        goto exit;
    }
    status = s->getVolumeData(voldata);
    if(0 != status) {
        PAL_ERR(LOG_TAG,"getVolumeData Failed");
        goto exit;
    }

    PAL_VERBOSE(LOG_TAG,"volume sent:%f", (voldata->volume_pair[0].vol));
    volume = (voldata->volume_pair[0].vol);

    payloadSize = sizeof(apm_module_param_data_t) +
                  sizeof(vcpm_param_cal_keys_payload_t) +
                  sizeof(vcpm_ckv_pair_t)*NUM_OF_CAL_KEYS;
    padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

    payloadInfo = (uint8_t *) calloc(1, payloadSize + padBytes);
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return -EINVAL;
    }
    header = (apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = VCPM_MODULE_INSTANCE_ID;
    header->param_id = VCPM_PARAM_ID_CAL_KEYS;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    cal_keys.vsid = vsid;
    cal_keys.num_ckv_pairs = NUM_OF_CAL_KEYS;
    if (volume < 0.0) {
            volume = 0.0;
    } else if (volume > 1.0) {
        volume = 1.0;
    }

    vol = lrint(volume * 100.0);

    // Voice volume levels from android are mapped to driver volume levels as follows.
    // 0 -> 5, 20 -> 4, 40 ->3, 60 -> 2, 80 -> 1, 100 -> 0
    // So adjust the volume to get the correct volume index in driver
    vol = 100 - vol;

    /*volume key*/
    cal_key_pair[0].cal_key_id = VCPM_CAL_KEY_ID_VOLUME_LEVEL;
    cal_key_pair[0].value = percent_to_index(vol, MIN_VOL_INDEX, max_vol_index);

    /*cal key for volume boost*/
    cal_key_pair[1].cal_key_id = VCPM_CAL_KEY_ID_VOL_BOOST;
    cal_key_pair[1].value = volume_boost;

     /*cal key for BWE/HD_VOICE*/
    cal_key_pair[2].cal_key_id = VCPM_CAL_KEY_ID_BWE;
    cal_key_pair[2].value = hd_voice;

    vol_pl = (uint8_t*)payloadInfo + sizeof(apm_module_param_data_t);
    ar_mem_cpy(vol_pl, sizeof(vcpm_param_cal_keys_payload_t),
                     &cal_keys, sizeof(vcpm_param_cal_keys_payload_t));

    vol_pl += sizeof(vcpm_param_cal_keys_payload_t);
    ar_mem_cpy(vol_pl, sizeof(vcpm_ckv_pair_t)*NUM_OF_CAL_KEYS,
                     &cal_key_pair, sizeof(vcpm_ckv_pair_t)*NUM_OF_CAL_KEYS);


    *size = payloadSize + padBytes;
    *payload = payloadInfo;
    PAL_DBG(LOG_TAG, "Volume level: %lf, volume boost: %d, HD voice: %d",
            percent_to_index(vol, MIN_VOL_INDEX, max_vol_index),
            volume_boost, hd_voice);

exit:
    if (voldata) {
        free(voldata);
    }
    return status;
}

int SessionAlsaVoice::payloadSetTTYMode(uint8_t **payload, size_t *size, uint32_t mode){
    int status = 0;
    apm_module_param_data_t* header;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0, padBytes = 0;
    uint8_t *phrase_pl;
    vcpm_param_id_tty_mode_t tty_payload;

    payloadSize = sizeof(struct apm_module_param_data_t)+
                  sizeof(tty_payload);
    padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

    payloadInfo = new uint8_t[payloadSize + padBytes]();
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return -EINVAL;
    }
    header = (apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = VCPM_MODULE_INSTANCE_ID;
    header->param_id = VCPM_PARAM_ID_TTY_MODE;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);

    tty_payload.vsid = vsid;
    tty_payload.mode = mode;
    phrase_pl = (uint8_t*)payloadInfo + sizeof(apm_module_param_data_t);
    ar_mem_cpy(phrase_pl,  sizeof(vcpm_param_id_tty_mode_t),
                     &tty_payload,  sizeof(vcpm_param_id_tty_mode_t));

    *size = payloadSize + padBytes;
    *payload = payloadInfo;
    return status;
}

int SessionAlsaVoice::setSidetone(int deviceId,Stream * s, bool enable){
    int status = 0;
    sidetone_mode_t mode;

    status = rm->getSidetoneMode((pal_device_id_t)deviceId, PAL_STREAM_VOICE_CALL, &mode);
    if(status) {
            PAL_ERR(LOG_TAG, "get sidetone mode failed");
    }
    if (mode == SIDETONE_HW) {
        PAL_DBG(LOG_TAG, "HW sidetone mode being set");
        if (enable) {
            status = setHWSidetone(s,1);
        } else {
            status = setHWSidetone(s,0);
        }
    }
    /*if SW mode it will be set via kv in graph open*/
    return status;
}

int SessionAlsaVoice::setHWSidetone(Stream * s, bool enable){
    int status = 0;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    std::shared_ptr<Device> rxDevice = nullptr;
    struct audio_route *audioRoute;
    bool set = false;

    status = s->getAssociatedDevices(associatedDevices);
    status = rm->getAudioRoute(&audioRoute);

    if (getRXDevice(s, rxDevice) != 0) {
        PAL_ERR(LOG_TAG, "failed, could not find associated RX device");
        return status;
    }

    status = s->getAssociatedDevices(associatedDevices);
    for(int i =0; i < associatedDevices.size(); i++) {
        switch(associatedDevices[i]->getSndDeviceId()){
            case PAL_DEVICE_IN_HANDSET_MIC:
                if(enable) {
                    if (rxDevice->getSndDeviceId() == PAL_DEVICE_OUT_WIRED_HEADPHONE)
                        audio_route_apply_and_update_path(audioRoute, "sidetone-headphone-handset-mic");
                    else
                        audio_route_apply_and_update_path(audioRoute, "sidetone-handset");
                    sideTone_cnt++;
                } else {
                    if (rxDevice->getSndDeviceId() == PAL_DEVICE_OUT_WIRED_HEADPHONE)
                        audio_route_reset_and_update_path(audioRoute, "sidetone-headphone-handset-mic");
                    else
                        audio_route_reset_and_update_path(audioRoute, "sidetone-handset");
                    sideTone_cnt--;
                }
                set = true;
                break;
            case PAL_DEVICE_IN_WIRED_HEADSET:
                if(enable) {
                    audio_route_apply_and_update_path(audioRoute, "sidetone-headphones");
                    sideTone_cnt++;
                } else {
                    audio_route_reset_and_update_path(audioRoute, "sidetone-headphones");
                    sideTone_cnt--;
                }
                set = true;
                break;
            default:
                PAL_DBG(LOG_TAG,"codec sidetone not supported on device %d",associatedDevices[i]->getSndDeviceId());
                break;

        }
        if(set)
            break;
    }
    return status;
}

int SessionAlsaVoice::disconnectSessionDevice(Stream *streamHandle,
                                              pal_stream_type_t streamType,
                                              std::shared_ptr<Device> deviceToDisconnect)
{
    std::vector<std::shared_ptr<Device>> deviceList;
    std::vector<std::string> aifBackEndsToDisconnect;
    struct pal_device dAttr;
    int status = 0;
    int ret = 0;
    int txDevId = PAL_DEVICE_NONE;
    void* plugin = nullptr;
    PluginConfig pluginConfig = nullptr;
    struct ReconfigPluginPayload ppld;

    PAL_DBG(LOG_TAG,"Enter");
    deviceList.push_back(deviceToDisconnect);
    rm->getBackEndNames(deviceList, rxAifBackEnds,txAifBackEnds);

    deviceToDisconnect->getDeviceAttributes(&dAttr);

    try {
        pm = PluginManager::getInstance();
        if(!pm){
            PAL_ERR(LOG_TAG, "unable to get plugin manager instance");
            return -EINVAL;
        }
        status = pm->openPlugin(PAL_PLUGIN_MANAGER_CONFIG, streamNameLUT.at(streamType), plugin);
        if (plugin && !status) {
            pluginConfig = reinterpret_cast<PluginConfig>(plugin);
        } else {
            PAL_ERR(LOG_TAG, "unable to get plugin for stream type %s", streamNameLUT.at(streamType).c_str());
        }
    }
    catch (const std::exception& e) {
        throw std::runtime_error(e.what());
    }

    if (rxAifBackEnds.size() > 0) {
        /*config mute on pop suppressor*/
        if (streamHandle->getCurState() != STREAM_INIT) {
            ppld.dAttr = dAttr;
            ppld.session = this;
            //pop suppressor call now in plugin config.
            if (pluginConfig) {
                ret = pluginConfig(streamHandle, PAL_PLUGIN_PRE_RECONFIG,
                            reinterpret_cast<void*>(&ppld), sizeof(ReconfigPluginPayload));
                if (ret) {
                    PAL_ERR(LOG_TAG, "Config Plugin Unsuccessful.");
                }
            } else {
                PAL_ERR(LOG_TAG, "pluginConfig is null, skipping plugin %d call",
                        PAL_PLUGIN_PRE_RECONFIG);
            }
        }

        /*if HW sidetone is enable disable it */
        if (sideTone_cnt > 0) {
            status = getTXDeviceId(streamHandle, &txDevId);
            if (status){
                PAL_ERR(LOG_TAG, "could not find TX device associated with this stream cannot set sidetone");
            } else {
                status = setSidetone(txDevId,streamHandle,0);
                if(0 != status) {
                   PAL_ERR(LOG_TAG,"disabling sidetone failed");
                }
            }
        }
        status =  SessionAlsaUtils::disconnectSessionDevice(streamHandle,
                                                            streamType, rm,
                                                            dAttr, pcmDevRxIds,
                                                            rxAifBackEnds);
        if(0 != status) {
            PAL_ERR(LOG_TAG,"disconnectSessionDevice on RX Failed \n");
            return status;
        }
    } else if (txAifBackEnds.size() > 0) {
        /*if HW sidetone is enable disable it */
        if (sideTone_cnt > 0) {
            status = getTXDeviceId(streamHandle, &txDevId);
            if (status){
                PAL_ERR(LOG_TAG, "could not find TX device associated with this stream cannot set sidetone");
            } else {
                status = setSidetone(txDevId,streamHandle,0);
                if(0 != status) {
                   PAL_ERR(LOG_TAG,"disabling sidetone failed");
                }
            }
        }

        if ((dAttr.id == PAL_DEVICE_IN_HANDSET_MIC ||
             dAttr.id == PAL_DEVICE_IN_SPEAKER_MIC) &&
             (rm->IsSilenceDetectionEnabledVoice() ||
              rm->IsSilenceDetectionEnabledPcm())) {
            ppld.config_ctrl = "silence_detection";
            ppld.dAttr = dAttr;
            ppld.session = this;
            ppld.pcmDevIds = pcmDevTxIds;
            ppld.aifBackEnds = txAifBackEnds;
            if (pluginConfig) {
                status = pluginConfig(streamHandle, PAL_PLUGIN_PRE_RECONFIG,
                                    reinterpret_cast<void*>(&ppld), sizeof(ppld));
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "pluginConfig failed");
                }
            } else {
                PAL_ERR(LOG_TAG, "pluginConfig is null, skipping plugin %d call",
                        PAL_PLUGIN_PRE_RECONFIG);
            }
        }

        status =  SessionAlsaUtils::disconnectSessionDevice(streamHandle,
                                                            streamType, rm,
                                                            dAttr, pcmDevTxIds,
                                                            txAifBackEnds);
        if(0 != status) {
            PAL_ERR(LOG_TAG,"disconnectSessionDevice on TX Failed");
        }
    }

    /*teardown external ec if needed*/
    if (SessionAlsaUtils::isRxDevice(dAttr.id)) {
        setExtECRef(streamHandle,deviceToDisconnect,false);
    }

    return status;
}

int SessionAlsaVoice::setupSessionDevice(Stream* streamHandle,
                                 pal_stream_type_t streamType,
                                 std::shared_ptr<Device> deviceToConnect)
{
    std::vector<std::shared_ptr<Device>> deviceList;
    std::vector<std::string> aifBackEndsToConnect;
    struct pal_device dAttr;
    int status = 0;
    void* plugin = nullptr;
    PluginConfig pluginConfig = nullptr;
    ReconfigPluginPayload ppld = {};

    deviceList.push_back(deviceToConnect);
    rm->getBackEndNames(deviceList, rxAifBackEnds, txAifBackEnds);
    deviceToConnect->getDeviceAttributes(&dAttr);

    /*setup external ec if needed*/
    if (SessionAlsaUtils::isRxDevice(dAttr.id)) {
        setExtECRef(streamHandle,deviceToConnect,true);
    }

    if (rxAifBackEnds.size() > 0) {
        status =  SessionAlsaUtils::setupSessionDevice(streamHandle, streamType,
                                                       rm, dAttr, pcmDevRxIds,
                                                       rxAifBackEnds);
        if(0 != status) {
            PAL_ERR(LOG_TAG,"setupSessionDevice on RX Failed");
            return status;
        }
    } else if (txAifBackEnds.size() > 0) {
        status =  SessionAlsaUtils::setupSessionDevice(streamHandle, streamType,
                                                       rm, dAttr, pcmDevTxIds,
                                                       txAifBackEnds);
        if(0 != status) {
            PAL_ERR(LOG_TAG,"setupSessionDevice on TX Failed");
        }
        if ((dAttr.id == PAL_DEVICE_IN_HANDSET_MIC ||
            dAttr.id == PAL_DEVICE_IN_SPEAKER_MIC) &&
            (rm->IsSilenceDetectionEnabledVoice())) {
            try {
                pm = PluginManager::getInstance();
                if (!pm){
                    PAL_ERR(LOG_TAG, "unable to get plugin manager instance");
                    return -EINVAL;
                }
                status = pm->openPlugin(PAL_PLUGIN_MANAGER_CONFIG, streamNameLUT.at(streamType),
                                         plugin);
                if (plugin && !status) {
                   pluginConfig = reinterpret_cast<PluginConfig>(plugin);
                   ppld.config_ctrl = "silence_detection";
                   ppld.dAttr = dAttr;
                   ppld.pcmDevIds = pcmDevTxIds;
                   ppld.session = this;
                   ppld.builder = reinterpret_cast<void*>(builder);
                   status = pluginConfig(streamHandle, PAL_PLUGIN_RECONFIG,
                                    reinterpret_cast<void*>(&ppld), sizeof(ReconfigPluginPayload));
                }
            } catch (const std::exception& e) {
                      throw std::runtime_error(e.what());
            }
        }
    }
    return status;
}

int SessionAlsaVoice::connectSessionDevice(Stream* streamHandle,
                                           pal_stream_type_t streamType,
                                           std::shared_ptr<Device> deviceToConnect)
{
    std::vector<std::shared_ptr<Device>> deviceList;
    std::shared_ptr<Device> rxDevice = nullptr;
    struct pal_device dAttr;
    struct pal_stream_attributes sAttr;
    int status = 0;
    int ret = 0;
    int txDevId = PAL_DEVICE_NONE;
    void* plugin = nullptr;
    PluginConfig pluginConfig = nullptr;
    ReconfigPluginPayload ppld = {};

    deviceList.push_back(deviceToConnect);
    rm->getBackEndNames(deviceList, rxAifBackEnds, txAifBackEnds);
    deviceToConnect->getDeviceAttributes(&dAttr);

    status = streamHandle->getStreamAttributes(&sAttr);

    if (status) {
        PAL_ERR(LOG_TAG, "could not get stream attributes\n");
        goto exit;
    }

    /* *This plugin is for post reconfig processing logic in SessionAlsaVoice.
     *  Now it's only being used for CRS populate rx mfc coeff payload usecase.
     *  It can be reorganized if more post-reconfig logics are added.
     * */
    try {
        pm = PluginManager::getInstance();
        if(!pm){
            PAL_ERR(LOG_TAG, "unable to get plugin manager instance");
            goto exit;
        }
        status = pm->openPlugin(PAL_PLUGIN_MANAGER_CONFIG, streamNameLUT.at(sAttr.type), plugin);
        if (plugin && !status) {
            pluginConfig = reinterpret_cast<PluginConfig>(plugin);
            ppld.pcmDevIds = pcmDevRxIds;
            ppld.session = this;
            ppld.builder = reinterpret_cast<void*>(builder);
        } else {
            PAL_ERR(LOG_TAG, "unable to get plugin for stream type %s", streamNameLUT.at(sAttr.type).c_str());
        }
    } catch (const std::exception& e) {
        throw std::runtime_error(e.what());
    }

    if (rxAifBackEnds.size() > 0) {
        status = SessionAlsaUtils::connectSessionDevice(this, streamHandle,
                                                         streamType, rm,
                                                         dAttr, pcmDevRxIds,
                                                         rxAifBackEnds);
        if(0 != status) {
            PAL_ERR(LOG_TAG,"connectSessionDevice on RX Failed");
            return status;
        }

        if(sideTone_cnt == 0) {
           if (deviceToConnect->getSndDeviceId() == PAL_DEVICE_OUT_HANDSET ||
               deviceToConnect->getSndDeviceId() == PAL_DEVICE_OUT_WIRED_HEADSET ||
               deviceToConnect->getSndDeviceId() == PAL_DEVICE_OUT_WIRED_HEADPHONE ||
               deviceToConnect->getSndDeviceId() == PAL_DEVICE_OUT_USB_DEVICE ||
               deviceToConnect->getSndDeviceId() == PAL_DEVICE_OUT_USB_HEADSET) {
               // set sidetone on new tx device after pcm_start
               status = getTXDeviceId(streamHandle, &txDevId);
               if (status){
                   PAL_ERR(LOG_TAG,"could not find TX device associated with this stream\n");
               }
               if (txDevId != PAL_DEVICE_NONE) {
                   status = setSidetone(txDevId, streamHandle, 1);
               }
               if (0 != status) {
                   PAL_ERR(LOG_TAG,"enabling sidetone failed");
               }
           }
        }
        //if CRSCall enabled, populate rx mfc coeff payload, in plugin.
        if (rm->IsCRSCallEnabled()) {
            ppld.payload = reinterpret_cast<void*>(&deviceToConnect);
            if (pluginConfig) {
                ret = pluginConfig(streamHandle, PAL_PLUGIN_POST_RECONFIG,
                                    reinterpret_cast<void*>(&ppld), sizeof(ReconfigPluginPayload));
                if (ret) {
                    PAL_ERR(LOG_TAG, "Config Plugin Unsuccessful.");
                }
            } else {
                PAL_ERR(LOG_TAG, "pluginConfig is null, skipping plugin %d call",
                        PAL_PLUGIN_POST_RECONFIG);
            }
        }
    } else if (txAifBackEnds.size() > 0) {
        status = SessionAlsaUtils::connectSessionDevice(this, streamHandle,
                                                         streamType, rm,
                                                         dAttr, pcmDevTxIds,
                                                         txAifBackEnds);
        if(0 != status) {
            PAL_ERR(LOG_TAG,"connectSessionDevice on TX Failed");
            return status;
        }

        if(sideTone_cnt == 0) {
           if (deviceToConnect->getSndDeviceId() > PAL_DEVICE_IN_MIN &&
               deviceToConnect->getSndDeviceId() < PAL_DEVICE_IN_MAX) {
               txDevId = deviceToConnect->getSndDeviceId();
           }
           if (getRXDevice(streamHandle, rxDevice) != 0) {
               PAL_DBG(LOG_TAG,"no active rx device, no need to setSidetone");
               return status;
           } else if (rxDevice && rxDevice->getDeviceCount() != 0 &&
                      txDevId != PAL_DEVICE_NONE) {
               status = setSidetone(txDevId, streamHandle, 1);
           }
           if (0 != status) {
               PAL_ERR(LOG_TAG,"enabling sidetone failed");
           }
        }
        //if CRSCall enabled, populate rx mfc coeff payload, in plugin.
        if (rm->IsCRSCallEnabled()) {
            ppld.payload = reinterpret_cast<void*>(&rxDevice);
            if (pluginConfig) {
                ret = pluginConfig(streamHandle, PAL_PLUGIN_POST_RECONFIG,
                                    reinterpret_cast<void*>(&ppld), sizeof(ReconfigPluginPayload));
                if (ret) {
                    PAL_ERR(LOG_TAG, "Config Plugin Unsuccessful.");
                }
            } else {
                PAL_ERR(LOG_TAG, "pluginConfig is null, skipping plugin %d call",
                        PAL_PLUGIN_POST_RECONFIG);
            }
        }
        //During Voice call device switch populate silence detection payload in plugin.
        if ((dAttr.id == PAL_DEVICE_IN_HANDSET_MIC ||
            dAttr.id == PAL_DEVICE_IN_SPEAKER_MIC) &&
            (rm->IsSilenceDetectionEnabledVoice())) {
            ppld.config_ctrl = "silence_detection";
            ppld.dAttr = dAttr;
            ppld.pcmDevIds = pcmDevTxIds;
            ppld.session = this;
            ppld.builder = reinterpret_cast<void*>(builder);
            PAL_ERR(LOG_TAG, "connect post reconfig session device\n");
            if (pluginConfig) {
                status = pluginConfig(streamHandle, PAL_PLUGIN_POST_RECONFIG,
                                    reinterpret_cast<void*>(&ppld), sizeof(ReconfigPluginPayload));
            } else {
                PAL_ERR(LOG_TAG, "pluginConfig is null, skipping plugin %d call",
                        PAL_PLUGIN_POST_RECONFIG);
            }
        }
    }
exit:
    PAL_DBG(LOG_TAG,"Exit ret: %d", status);
    return status;
}

int32_t SessionAlsaVoice::allocateFrontEndIds(const struct pal_stream_attributes &sAttr,
                                              int lDirection) {
    uint32_t status = 0;
    int id;

    PAL_DBG(LOG_TAG, "Enter");
    if (lDirection == RX_HOSTLESS) {
        if (sAttr.info.voice_call_info.VSID == VOICEMMODE1 ||
            sAttr.info.voice_call_info.VSID == VOICELBMMODE1)
            id = rm->allocateFrontEndIds(VOICE1_PLAYBACK_HOSTLESS);
        else if (sAttr.info.voice_call_info.VSID == VOICEMMODE2 ||
                 sAttr.info.voice_call_info.VSID == VOICELBMMODE2)
            id = rm->allocateFrontEndIds(VOICE2_PLAYBACK_HOSTLESS);
        if (id < 0) {
            PAL_ERR(LOG_TAG, "allocateFrontEndIds failed");
            status = -EINVAL;
            goto exit;
        }
        pcmDevRxIds.push_back(id);
    } else {
        if (sAttr.info.voice_call_info.VSID == VOICEMMODE1 ||
            sAttr.info.voice_call_info.VSID == VOICELBMMODE1)
            id = rm->allocateFrontEndIds(VOICE1_RECORD_HOSTLESS);
        else if (sAttr.info.voice_call_info.VSID == VOICEMMODE2 ||
                 sAttr.info.voice_call_info.VSID == VOICELBMMODE2)
            id = rm->allocateFrontEndIds(VOICE2_RECORD_HOSTLESS);
        if (id < 0) {
            PAL_ERR(LOG_TAG, "allocateFrontEndIds failed");
            status = -EINVAL;
            goto exit;
        }
        pcmDevTxIds.push_back(id);
    }

exit:
    PAL_DBG(LOG_TAG, "Exit status %d", status);
    return status;
}

void SessionAlsaVoice::freeFrontEndIds(const struct pal_stream_attributes &sAttr, int lDirection) {
    if (lDirection == RX_HOSTLESS) {
        if (sAttr.info.voice_call_info.VSID == VOICEMMODE1 ||
            sAttr.info.voice_call_info.VSID == VOICELBMMODE1)
            rm->freeFrontEndIds(VOICE1_PLAYBACK_HOSTLESS, pcmDevRxIds);
        else if (sAttr.info.voice_call_info.VSID == VOICEMMODE2 ||
                 sAttr.info.voice_call_info.VSID == VOICELBMMODE2)
            rm->freeFrontEndIds(VOICE2_PLAYBACK_HOSTLESS, pcmDevRxIds);
    } else {
        if (sAttr.info.voice_call_info.VSID == VOICEMMODE1 ||
            sAttr.info.voice_call_info.VSID == VOICELBMMODE1)
            rm->freeFrontEndIds(VOICE1_RECORD_HOSTLESS, pcmDevTxIds);
        else if (sAttr.info.voice_call_info.VSID == VOICEMMODE2 ||
                 sAttr.info.voice_call_info.VSID == VOICELBMMODE2)
            rm->freeFrontEndIds(VOICE2_RECORD_HOSTLESS, pcmDevTxIds);
    }
    return;
}

int SessionAlsaVoice::setVoiceMixerParameter(Stream * s, struct mixer *mixer,
                                             void *payload, int size, int dir)
{
    char *control = (char*)"setParam";
    char *mixer_str;
    struct mixer_ctl *ctl;
    int ctl_len = 0,ret = 0;
    struct pal_stream_attributes sAttr;
    char *stream = SessionAlsaVoice::getMixerVoiceStream(s, dir);

    ret = s->getStreamAttributes(&sAttr);

    if (ret) {
         PAL_ERR(LOG_TAG, "could not get stream attributes\n");
        return ret;
    }

    ctl_len = strlen(stream) + 4 + strlen(control) + 1;
    mixer_str = (char *)calloc(1, ctl_len);
    if (!mixer_str) {
        free(payload);
        return -ENOMEM;
    }
    snprintf(mixer_str, ctl_len, "%s %s", stream, control);

    PAL_VERBOSE(LOG_TAG, "- mixer -%s-\n", mixer_str);
    ctl = mixer_get_ctl_by_name(mixer, mixer_str);
    if (!ctl) {
        PAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", mixer_str);
        free(mixer_str);
        return -ENOENT;
    }

    ret = mixer_ctl_set_array(ctl, payload, size);

    PAL_DBG(LOG_TAG, "exit, ret = %d, cnt = %d\n", ret, size);
    free(mixer_str);
    return ret;
}

char* SessionAlsaVoice::getMixerVoiceStream(Stream *s, int dir){
    char *stream = (char*)"VOICEMMODE1p";
    struct pal_stream_attributes sAttr;

    s->getStreamAttributes(&sAttr);
    if (sAttr.info.voice_call_info.VSID == VOICEMMODE1 ||
        sAttr.info.voice_call_info.VSID == VOICELBMMODE1) {
        if (dir == TX_HOSTLESS) {
            stream = (char*)"VOICEMMODE1c";
        } else {
            stream = (char*)"VOICEMMODE1p";
        }
    } else {
        if (dir == TX_HOSTLESS) {
            stream = (char*)"VOICEMMODE2c";
        } else {
            stream = (char*)"VOICEMMODE2p";
        }
    }
    return stream;
}

int SessionAlsaVoice::setExtECRef(Stream *s, std::shared_ptr<Device> rx_dev, bool is_enable)
{
    int status = 0;
    struct pal_stream_attributes sAttr = {};
    struct pal_device rxDevAttr = {};
    struct pal_device_info rxDevInfo = {};

    if (!s) {
        PAL_ERR(LOG_TAG, "Invalid stream");
        status = -EINVAL;
        goto exit;
    }

    status = s->getStreamAttributes(&sAttr);
    if(0 != status) {
       PAL_ERR(LOG_TAG, "getStreamAttributes Failed \n");
       goto exit;
    }

    rxDevInfo.isExternalECRefEnabledFlag = 0;
    if (rx_dev) {
        status = rx_dev->getDeviceAttributes(&rxDevAttr, s);
        if (status != 0) {
            PAL_ERR(LOG_TAG," get device attributes failed");
            goto exit;
        }
        rm->getDeviceInfo(rxDevAttr.id, sAttr.type, rxDevAttr.custom_config.custom_key, &rxDevInfo);
    }

    if (rxDevInfo.isExternalECRefEnabledFlag) {
        status = checkAndSetExtEC(rm, s, is_enable);
        if (status)
            PAL_ERR(LOG_TAG,"Failed to enable Ext EC for voice");
    }

exit:
    return status;
}

int SessionAlsaVoice::getTXDeviceId(Stream *s, int *id)
{
    int status = 0;
    int i;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    *id = PAL_DEVICE_NONE;

    status = s->getAssociatedDevices(associatedDevices);
    if(0 != status) {
        PAL_ERR(LOG_TAG,"getAssociatedDevices Failed");
        return status;
    }

    for (i =0; i < associatedDevices.size(); i++) {
        if (associatedDevices[i]->getSndDeviceId() > PAL_DEVICE_IN_MIN &&
            associatedDevices[i]->getSndDeviceId() < PAL_DEVICE_IN_MAX) {
            *id = associatedDevices[i]->getSndDeviceId();
            break;
        }
    }
    if(i >= PAL_DEVICE_IN_MAX){
        status = -EINVAL;
    }
    return status;
}

int SessionAlsaVoice::registerRxCallBack(session_callback cb, uint64_t cookie)
{
    sessionRxCb = cb;
    rxCbCookie = cookie;
    return 0;
}

int SessionAlsaVoice::registerTxCallBack(session_callback cb, uint64_t cookie)
{
    sessionTxCb = cb;
    txCbCookie = cookie;
    return 0;
}

int SessionAlsaVoice::getRXDevice(Stream *s, std::shared_ptr<Device> &rx_dev)
{
    int status = 0;
    int i;
    std::vector<std::shared_ptr<Device>> associatedDevices;

    rx_dev = nullptr;
    status = s->getAssociatedDevices(associatedDevices);
    if(0 != status) {
        PAL_ERR(LOG_TAG,"getAssociatedDevices Failed");
        return status;
    }

    for (i = 0; i < associatedDevices.size(); i++) {
        if (associatedDevices[i]->getSndDeviceId() > PAL_DEVICE_OUT_MIN &&
            associatedDevices[i]->getSndDeviceId() < PAL_DEVICE_OUT_MAX) {
            rx_dev = associatedDevices[i];
            break;
        }
    }
    if(rx_dev == nullptr) {
        status = -EINVAL;
    }
    return status;
}
