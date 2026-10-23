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
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "PAL: DisplayPort"
#include "DisplayPortUpstream.h"
#include "SessionAlsaUtils.h"
#include "ResourceManager.h"
#include "PayloadBuilder.h"
#include "Device.h"
#include "kvh2xml.h"
#include <tinyalsa/asoundlib.h>
#include "SessionAR.h"

extern "C" void CreateDisplayDevice(struct pal_device *device,
                                    const std::shared_ptr<ResourceManager> rm,
                                    std::shared_ptr<Device> *dev) {
    *dev = DisplayPort::getInstance(device, rm);

}

enum {
    EXT_DISPLAY_TYPE_NONE,
    EXT_DISPLAY_TYPE_HDMI,
    EXT_DISPLAY_TYPE_DP
};

enum cea_edid_versions {
    CEA_EDID_VER_NONE      = 0,
    CEA_EDID_VER_CEA861    = 1,
    CEA_EDID_VER_CEA861A   = 2,
    CEA_EDID_VER_CEA861BCD = 3,
    CEA_EDID_VER_RESERVED  = 4,
};

enum eld_versions {
    ELD_VER_CEA_861D = 2,
    ELD_VER_PARTIAL  = 31,
};

#define GRAB_BITS(buf, byte, lowbit, bits) \
({\
    (buf[byte] >> (lowbit)) & ((1 << (bits)) - 1);\
})

#define MAX_SAD_BLOCKS      10
#define SAD_BLOCK_SIZE      3
#define HDMI_CONTROLLER     1
#define HDMI_STREAM         0
#define DP_CONTROLLER       0
#define DP_STREAM           0
#define ELD_FIXED_BYTES     20
#define ELD_MAX_MNL         16

#define ELD_VER_OFFSET         0
#define ELD_VER_START_BIT      3
#define ELD_VER_NUM_BITS       5

#define SAD_COUNT_OFFSET       5
#define SAD_COUNT_START_BIT    4
#define SAD_COUNT_NUM_BITS     4

#define MNL_OFFSET             4
#define MNL_START_BIT          0
#define MNL_NUM_BITS           5

#define SPK_ALLOC_OFFSET       7
#define SPK_ALLOC_START_BIT    0
#define SPK_ALLOC_NUM_BITS     7

#define SAD_CHNL_BYTE          0
#define SAD_CHNL_START_BIT     0
#define SAD_CHNL_NUM_BITS      3

#define SAD_FMT_BYTE           0
#define SAD_FMT_START_BIT      3
#define SAD_FMT_NUM_BITS       4

#define SAD_FREQ_BYTE          1
#define SAD_FREQ_START_BIT     0
#define SAD_FREQ_NUM_BITS      7

#define SAD_BITRATE_BYTE       2
#define SAD_BITRATE_START_BIT  0
#define SAD_BITRATE_NUM_BITS   3

static struct extDispState {
    void *eldInfo = NULL;
    bool valid = false;
    int type = EXT_DISPLAY_TYPE_NONE;
} extDisp[MAX_CONTROLLERS][MAX_STREAMS_PER_CONTROLLER];

std::shared_ptr<Device> DisplayPort::dpObj = nullptr;
std::shared_ptr<Device> DisplayPort::hdmiObj = nullptr;

std::shared_ptr<Device> DisplayPort::getInstance(struct pal_device *device,
                                             std::shared_ptr<ResourceManager> Rm)
{
    if (!device)
       return NULL;

    PAL_DBG(LOG_TAG, "Enter, device id %d", device->id);

    if (device->id == PAL_DEVICE_OUT_HDMI) {
        if (!hdmiObj) {
            std::shared_ptr<Device> sp(new DisplayPort(device, Rm));
            hdmiObj = sp;
        }
        return hdmiObj;
    } else if ((device->id == PAL_DEVICE_OUT_AUX_DIGITAL) ||
      (device->id == PAL_DEVICE_OUT_AUX_DIGITAL_1)) {
        if (!dpObj) {
            std::shared_ptr<Device> sp(new DisplayPort(device, Rm));
            dpObj = sp;
        }
        return dpObj;
    }
    return NULL;
}

std::shared_ptr<Device> DisplayPort::getObject(pal_device_id_t id)
{
    if ((id == PAL_DEVICE_OUT_AUX_DIGITAL) ||
        (id == PAL_DEVICE_OUT_AUX_DIGITAL_1)) {
        if (dpObj) {
            if (dpObj->getSndDeviceId() == id)
                return dpObj;
        }
    } else if (id == PAL_DEVICE_OUT_HDMI) {
        if (hdmiObj) {
            if (hdmiObj->getSndDeviceId() == id)
                return hdmiObj;
        }
    }
    return NULL;
}

DisplayPort::DisplayPort(struct pal_device *device, std::shared_ptr<ResourceManager> Rm) :
Device(device, Rm)
{

}

int DisplayPort::getDeviceChannelAllocation(int num_channels)
{
    int channel_allocation = 0;

    switch (num_channels) {
        case 2:
            channel_allocation = 0x0; break;
        case 3:
            channel_allocation = 0x02; break;
        case 4:
            channel_allocation = 0x06; break;
        case 5:
            channel_allocation = 0x0A; break;
        case 6:
            channel_allocation = 0x0B; break;
        case 7:
            channel_allocation = 0x12; break;
        case 8:
            channel_allocation = 0x13; break;
        default:
            channel_allocation = 0x0; break;
            PAL_ERR(LOG_TAG, "invalid num channels: %d\n",
                    num_channels);
            break;
    }

    PAL_DBG(LOG_TAG, "num channels: %d, ca: 0x%x", num_channels,
            channel_allocation);

    return channel_allocation;
}

int DisplayPort::getDeviceAttributes(struct pal_device *dattr, Stream* streamHandle)
{
    int status = 0;
    int channel_allocation = 0;
    (void)streamHandle;

    if (!dattr) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG,"Invalid device attributes %d", status);
        return status;
    }
    ar_mem_cpy(dattr, sizeof(struct pal_device), &deviceAttr, sizeof(struct pal_device));

    channel_allocation = getDeviceChannelAllocation(deviceAttr.config.ch_info.channels);

    retrieveChannelMapLpass(channel_allocation, &dattr->config.ch_info.ch_map[0],
            PAL_MAX_CHANNELS_SUPPORTED);

    return status;
}

int DisplayPort::start()
{
    int status = 0;

    if (customPayload)
        free(customPayload);

    customPayload = NULL;
    customPayloadSize = 0;

    if ((deviceAttr.id == PAL_DEVICE_OUT_AUX_DIGITAL) ||
            (deviceAttr.id == PAL_DEVICE_OUT_AUX_DIGITAL_1)) {
        status = configureDpEndpoint();
        if (status != 0) {
            PAL_ERR(LOG_TAG,"Endpoint Configuration Failed");
            return status;
        }
    }

    status = Device::start();
    return status;

}

int DisplayPort::configureDpEndpoint()
{
    int status = 0;
    std::string backEndName;
    PayloadBuilder* builder = new PayloadBuilder();
    struct dpAudioConfig cfg;
    uint8_t* payload = NULL;
    Stream *stream = NULL;
    Session *session = NULL;
    size_t payloadSize = 0;
    std::shared_ptr<Device> dev = nullptr;
    std::vector<Stream*> activestreams;
    uint32_t miid = 0;

    rm->getBackendName(deviceAttr.id, backEndName);
    dev = Device::getInstance(&deviceAttr, rm);
    status = rm->getActiveStream_l(activestreams, dev);
    if ((0 != status) || (activestreams.size() == 0)) {
        PAL_ERR(LOG_TAG, "no active stream available");
        status = -EINVAL;
        goto exit;
    }
    stream = static_cast<Stream *>(activestreams[0]);
    stream->getAssociatedSession(&session);
    status = dynamic_cast<SessionAR*>(session)->getMIID(backEndName.c_str(), DEVICE_HW_ENDPOINT_RX, &miid);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", DEVICE_HW_ENDPOINT_RX, status);
        goto exit;
    }
    cfg.channel_allocation = getDeviceChannelAllocation(deviceAttr.config.ch_info.channels);
    cfg.mst_idx = dp_stream;
    cfg.dptx_idx = dp_controller;
    builder->payloadDpAudioConfig(&payload, &payloadSize, miid, &cfg);
    if (payloadSize) {
        status = updateCustomPayload(payload, payloadSize);
        free(payload);
        if (0 != status) {
            PAL_ERR(LOG_TAG," updateCustomPayload Failed\n");
            goto exit;
        }
    }

exit:
    if(builder) {
       delete builder;
       builder = NULL;
    }
    return status;
}

int DisplayPort::init(pal_param_device_connection_t device_conn)
{
    PAL_DBG(LOG_TAG," Enter");
    int status = 0;
    struct mixer *mixer;
    status = rm->getHwAudioMixer(&mixer);
    pal_param_disp_port_config_params* dp_config = (pal_param_disp_port_config_params*) &device_conn.device_config.dp_config;
    if (status) {
        PAL_ERR(LOG_TAG," mixer error");
        return status;
    }

    if (device_conn.id == PAL_DEVICE_OUT_HDMI) {
        dp_controller = HDMI_CONTROLLER;
        dp_stream = HDMI_STREAM;
        extDisp[dp_controller][dp_stream].type = EXT_DISPLAY_TYPE_HDMI;
    }
    else if (device_conn.id == PAL_DEVICE_OUT_AUX_DIGITAL ||
        device_conn.id == PAL_DEVICE_OUT_AUX_DIGITAL_1) {
        dp_controller = DP_CONTROLLER;
        dp_stream = DP_STREAM;
        extDisp[dp_controller][dp_stream].type = EXT_DISPLAY_TYPE_DP;
    }
    PAL_DBG(LOG_TAG," DP contr: %d  stream: %d", dp_controller, dp_stream);
    cacheEld(mixer, dp_controller, dp_stream, dp_config->pcmId);
    PAL_DBG(LOG_TAG," Exit");
    return 0;
}

int DisplayPort::deinit(pal_param_device_connection_t device_conn __unused)
{
    //To-Do : Have to invalidate the cahed ELD
    return 0;
}

void DisplayPort::resetEldInfo() {
    PAL_VERBOSE(LOG_TAG," enter");

    int i = 0, j = 0;
    for (i = 0; i < MAX_CONTROLLERS; ++i) {
        for (j = 0; j < MAX_STREAMS_PER_CONTROLLER; ++j) {
            struct extDispState *state = &extDisp[i][j];
            state->type = EXT_DISPLAY_TYPE_NONE;
            if (state->eldInfo) {
                free(state->eldInfo);
                state->eldInfo = NULL;
            }
            state->valid = false;
        }
    }
}

int DisplayPort::getEldInfo(struct audio_mixer *mixer, int controller, int stream, int pcmId)
{
    char block[ELD_FIXED_BYTES + ELD_MAX_MNL + MAX_SAD_BLOCKS * SAD_BLOCK_SIZE];
    int ret, count;
    char eldData[ELD_FIXED_BYTES + ELD_MAX_MNL + MAX_SAD_BLOCKS * SAD_BLOCK_SIZE + 1] = {0};
    struct extDispState *state = NULL;
    struct mixer_ctl *ctl = NULL;
    char mixerCtlName[MIXER_PATH_MAX_LENGTH] = {0};

    state = &extDisp[controller][stream];
    if (state->valid) {
        /* use cached eld */
        return 0;
    }

    snprintf(mixerCtlName, sizeof(mixerCtlName), "ELD");

    if (state->eldInfo == NULL)
        state->eldInfo = (struct eldAudioInfo *)calloc(1, sizeof(struct eldAudioInfo));

    PAL_VERBOSE(LOG_TAG," mixer ctl name: %s", mixerCtlName);

    ctl = mixer_get_ctl_by_name_and_device(mixer, mixerCtlName, pcmId);
    if (!ctl) {
        PAL_ERR(LOG_TAG," Could not get ctl for mixer cmd - %s", mixerCtlName);
        goto fail;
    }

    mixer_ctl_update(ctl);
    count = mixer_ctl_get_num_values(ctl);
    /* Read SAD blocks, clamping the maximum size for safety */
    if (count > (int)sizeof(block))
        count = (int)sizeof(block);

    ret = mixer_ctl_get_array(ctl, block, count);
    if (ret != 0) {
        PAL_ERR(LOG_TAG," mixer_ctl_get_array() failed to get ELD info");
        goto fail;
    }

    eldData[0] = count;
    memcpy(&eldData[1], block, count);

    if (!getSinkCaps((struct eldAudioInfo *)state->eldInfo, eldData)) {
        PAL_ERR(LOG_TAG," Failed to get extn disp sink capabilities");
        goto fail;
    }

    PAL_VERBOSE(LOG_TAG," received eld data: count %d", eldData[0]);
    state->valid = true;
    return 0;
fail:
    if (state->eldInfo) {
        free(state->eldInfo);
        state->eldInfo = NULL;
        state->valid = false;
    }
    PAL_ERR(LOG_TAG," return -EINVAL");
    return -EINVAL;
}

void DisplayPort::cacheEld(struct audio_mixer *mixer, int controller, int stream, int pcmId)
{
/*
    1) getEldInfo()
    2) parseEldInfo()
*/
    getEldInfo(mixer, controller, stream, pcmId);
}

int32_t DisplayPort::isSampleRateSupported(uint32_t sampleRate)
{
    int32_t rc = 0;
    int i = 0;
    PAL_DBG(LOG_TAG, "sampleRate %d", sampleRate);
    struct extDispState *state = NULL;

    state = &extDisp[dp_controller][dp_stream];
    if (!(state && state->eldInfo)) {
        rc = -EINVAL;
        PAL_ERR(LOG_TAG, "ELD not available");
        return rc;
    }

    eldAudioInfo* info = (eldAudioInfo*) state->eldInfo;
    if (info != NULL && sampleRate != 0) {
        for (i = 0; i < info->audioBlocks && i < MAX_ELD_BLOCKS; i++) {
                if (isSampleRateSupported(info->audioBlocksArray[i].samplingFreqBitmask,
                        sampleRate)) {
                        PAL_DBG(LOG_TAG," Returns true for sample rate [%d]", sampleRate);
                        return rc;
                }
        }
    }

    PAL_ERR(LOG_TAG," Returns false for sample rate [%d]", sampleRate);
    return -EINVAL;
}

int32_t DisplayPort::isChannelSupported(uint32_t numChannels)
{
    int32_t rc = 0;
    int i = 0;
    PAL_DBG(LOG_TAG, "numChannels %u", numChannels);
    struct extDispState *state = NULL;

    state = &extDisp[dp_controller][dp_stream];
    if (!(state && state->eldInfo)) {
        rc = -EINVAL;
        PAL_ERR(LOG_TAG, "ELD not available");
        return rc;
    }

    eldAudioInfo* info = (eldAudioInfo*) state->eldInfo;
    if (info != NULL) {
        for (i = 0; i < info->audioBlocks && i < MAX_ELD_BLOCKS; i++) {
                if (info->audioBlocksArray[i].formatId == LPCM) {
                        if (numChannels == info->audioBlocksArray[i].channels){
                                return rc;
                        }
                }
        }
    }
    PAL_ERR(LOG_TAG," Returns false for numChannels [%d]", numChannels);
    return -EINVAL;
}

int32_t DisplayPort::isBitWidthSupported(uint32_t bitWidth)
{
    int32_t rc = 0;
    int i = 0;
    PAL_DBG(LOG_TAG, "bitWidth %u", bitWidth);
    struct extDispState *state = NULL;

    if (bitWidth == 16 || bitWidth == 24 || bitWidth == 32) {
        //16 bit bps is always supported
        //some oem may not update 16bit support in their edid info
        return rc;
    }

    state = &extDisp[dp_controller][dp_stream];
    if (!(state && state->eldInfo)) {
        rc = -EINVAL;
        PAL_ERR(LOG_TAG, "ELD not available");
        return rc;
    }

    eldAudioInfo* info = (eldAudioInfo*) state->eldInfo;
    if (info != NULL && bitWidth != 0) {
        for (i = 0; i < info->audioBlocks && i < MAX_ELD_BLOCKS; i++) {
                if (isSupportedBps(info->audioBlocksArray[i].bitsPerSampleBitmask, bitWidth)) {
                        PAL_VERBOSE(LOG_TAG," returns true for bit width [%d]", bitWidth);
                        return rc;
                }
        }
    }
    PAL_ERR(LOG_TAG," Returns false for bitWidth [%d]", bitWidth);
    return -EINVAL;
}

int32_t DisplayPort::checkAndUpdateBitWidth(uint32_t *bitWidth)
{
    int32_t rc = 0;
    int i = 0;
    PAL_DBG(LOG_TAG, "bitWidth %u", *bitWidth);
    struct extDispState *state = NULL;

    state = &extDisp[dp_controller][dp_stream];
    if (!(state && state->eldInfo)) {
        rc = -EINVAL;
        PAL_ERR(LOG_TAG, "ELD not available");
        return rc;
    }
    if (*bitWidth == 16) {
        //16 bit bps is always supported
        //some oem may not update 16bit support in their edid info
        return rc;
    }

    eldAudioInfo* info = (eldAudioInfo*) state->eldInfo;
    if (info != NULL && *bitWidth != 0) {
        for (i = 0; i < info->audioBlocks && i < MAX_ELD_BLOCKS; i++) {
                if (isSupportedBps(info->audioBlocksArray[i].bitsPerSampleBitmask, *bitWidth)) {
                        PAL_VERBOSE(LOG_TAG," returns true for bit width [%d]", *bitWidth);
                        return rc;
                }
        }
    }
    *bitWidth = BITWIDTH_16;
    PAL_DBG(LOG_TAG, "bit width not supported, setting to default 16 bit");
    return rc;
}

int32_t DisplayPort::checkAndUpdateSampleRate(uint32_t *sampleRate)
{
    int32_t rc = 0;

    if (*sampleRate <= SAMPLINGRATE_48K)
        *sampleRate = SAMPLINGRATE_48K;
    else if (*sampleRate > SAMPLINGRATE_48K && *sampleRate <= SAMPLINGRATE_96K)
        *sampleRate = SAMPLINGRATE_96K;
    else if (*sampleRate > SAMPLINGRATE_96K && *sampleRate <= SAMPLINGRATE_192K)
        *sampleRate = SAMPLINGRATE_192K;
    else if (*sampleRate > SAMPLINGRATE_192K && *sampleRate <= SAMPLINGRATE_384K)
        *sampleRate = SAMPLINGRATE_384K;

    PAL_DBG(LOG_TAG, "sampleRate %d", *sampleRate);

    return rc;
}


int32_t DisplayPort::getDeviceConfig(struct pal_device *deviceattr,
                                     struct pal_stream_attributes *sAttr) {

    int32_t status = 0;
    struct pal_channel_info dev_ch_info;
    struct pal_device_info devinfo = {};
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    if (!sAttr) {
        PAL_ERR(LOG_TAG, "Invalid parameter.");
        return -EINVAL;
    }
    rm->getDeviceInfo(deviceattr->id, sAttr->type,
                  deviceattr->custom_config.custom_key, &devinfo);
    /**
     * Comparision of stream channel and device supported max channel.
     * If stream channel is less than or equal to device supported
     * channel then the channel of stream is taken othewise it is of
     * device
     */
    int channels = this->getMaxChannel();

    if (channels > sAttr->out_media_config.ch_info.channels)
        channels = sAttr->out_media_config.ch_info.channels;

    /**
     * According to HDMI spec CEA-861-E, 1 channel is not
     * supported, thus converting 1 channel to 2 channels.
     */
    if (channels == 1)
        channels = 2;

    dev_ch_info.channels = channels;

    rm->getChannelMap(&(dev_ch_info.ch_map[0]), channels);
    deviceattr->config.ch_info = dev_ch_info;

    if (!this->isSupportedSR(NULL,deviceattr->config.sample_rate)) {
        deviceattr->config.sample_rate = this->getHighestSupportedSR();

        if (sAttr->out_media_config.sample_rate < SAMPLINGRATE_32K &&
            (sAttr->out_media_config.sample_rate % 11025) == 0 &&
            this->isSupportedSR(NULL,SAMPLINGRATE_44K)) {
                deviceattr->config.sample_rate = SAMPLINGRATE_44K;
        }
    }

    if (this->isBitWidthSupported(deviceattr->config.bit_width) != 0) {
        int bps = this->getHighestSupportedBps();
        if (sAttr->out_media_config.bit_width > bps)
            deviceattr->config.bit_width = bps;
        else
            deviceattr->config.bit_width = BITWIDTH_16;
    }
    if ((deviceattr->config.bit_width == BITWIDTH_32) &&
                (devinfo.bitFormatSupported != PAL_AUDIO_FMT_PCM_S32_LE)) {
        PAL_DBG(LOG_TAG, "32 bit is not supported; update with supported bit format");
        deviceattr->config.aud_fmt_id = devinfo.bitFormatSupported;
        deviceattr->config.bit_width =
                rm->palFormatToBitwidthLookup(devinfo.bitFormatSupported);
    } else {
        if (deviceattr->config.bit_width == 32) {
            deviceattr->config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S32_LE;
        } else if (deviceattr->config.bit_width == 24) {
            if (sAttr->out_media_config.aud_fmt_id == PAL_AUDIO_FMT_PCM_S24_LE)
                deviceattr->config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S24_LE;
            else
                deviceattr->config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S24_3LE;
        } else {
            deviceattr->config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S16_LE;
        }
    }

    PAL_DBG(LOG_TAG, "device %d sample rate %d bitwidth %d",
            deviceattr->id, deviceattr->config.sample_rate,
            deviceattr->config.bit_width);

    return status;
}

/* ----------------------------------------------------------------------------------
   ------------------------         Eld                           -------------------
   ----------------------------------------------------------------------------------*/
const char * DisplayPort::eldFormatToStr(unsigned char format)
{
    switch (format) {
    case LPCM:
        return "Format:LPCM";
    case AC3:
        return "Format:AC-3";
    case MPEG1:
        return "Format:MPEG1 (Layers 1 & 2)";
    case MP3:
        return "Format:MP3 (MPEG1 Layer 3)";
    case MPEG2_MULTI_CHANNEL:
        return "Format:MPEG2 (multichannel)";
    case AAC:
        return "Format:AAC";
    case DTS:
        return "Format:DTS";
    case ATRAC:
        return "Format:ATRAC";
    case SACD:
        return "Format:One-bit audio aka SACD";
    case DOLBY_DIGITAL_PLUS:
        return "Format:Dolby Digital +";
    case DTS_HD:
        return "Format:DTS-HD";
    case MAT:
        return "Format:MAT (MLP)";
    case DST:
        return "Format:DST";
    case WMA_PRO:
        return "Format:WMA Pro";
    default:
        return "??";
    }
}

bool DisplayPort::isSampleRateSupported(unsigned char srByte, int samplingRate)
{
    int result = 0;
    // Codec Supports Sample rate in range of 48K-192K
    PAL_VERBOSE(LOG_TAG," srByte: %d, samplingRate: %d", srByte, samplingRate);
    switch (samplingRate) {
    case 192000:
        result = (srByte & BIT(6));
        break;
    case 176400:
        result = (srByte & BIT(5));
        break;
    case 96000:
        result = (srByte & BIT(4));
        break;
    case 88200:
        result = (srByte & BIT(3));
        break;
    case 48000:
        result = (srByte & BIT(2));
        break;
    case 44100:
        result = (srByte & BIT(1));
        break;
    case 32000:
        result = (srByte & BIT(0));
        break;
     default:
        break;
    }

    if (result)
        return true;

    return false;
}

unsigned char DisplayPort::getEldBpsByte(unsigned char byte,
                        unsigned char format)
{
    if (format == 0) {
        PAL_VERBOSE(LOG_TAG," not lpcm format, return 0");
        return 0;
    }
    return byte;
}

bool DisplayPort::isSupportedBps(unsigned char bpsByte, int bps)
{
    int result = 0;

    switch (bps) {
    case 24:
        PAL_VERBOSE(LOG_TAG,"24bit");
        result = (bpsByte & BIT(2));
        break;
    case 16:
        PAL_VERBOSE(LOG_TAG,"16bit");
        result = (bpsByte & BIT(0));
        break;
     default:
        break;
    }

    if (result)
        return true;

    return false;
}

int DisplayPort::getHighestEldSF(unsigned char byte)
{
    int nfreq = 0;

    if (byte & BIT(6)) {
        PAL_VERBOSE(LOG_TAG,"Highest: 192kHz");
        nfreq = 192000;
    } else if (byte & BIT(5)) {
        PAL_VERBOSE(LOG_TAG,"Highest: 176kHz");
        nfreq = 176000;
    } else if (byte & BIT(4)) {
        PAL_VERBOSE(LOG_TAG,"Highest: 96kHz");
        nfreq = 96000;
    } else if (byte & BIT(3)) {
        PAL_VERBOSE(LOG_TAG,"Highest: 88.2kHz");
        nfreq = 88200;
    } else if (byte & BIT(2)) {
        PAL_VERBOSE(LOG_TAG,"Highest: 48kHz");
        nfreq = 48000;
    } else if (byte & BIT(1)) {
        PAL_VERBOSE(LOG_TAG,"Highest: 44.1kHz");
        nfreq = 44100;
    } else if (byte & BIT(0)) {
        PAL_VERBOSE(LOG_TAG,"Highest: 32kHz");
        nfreq = 32000;
    }
    return nfreq;
}

void DisplayPort::updateChannelMap(eldAudioInfo* info)
{
    /* HDMI Cable follows CEA standard so SAD is received in CEA
     * Input source file channel map is fed to ASM in WAV standard(audio.h)
     * so upto 7.1 SAD bits are:
     * in CEA convention: RLC/RRC,FLC/FRC,RC,RL/RR,FC,LFE,FL/FR
     * in WAV convention: BL/BR,FLC/FRC,BC,SL/SR,FC,LFE,FL/FR
     * Corresponding ADSP IDs (apr-audio_v2.h):
     * PCM_CHANNEL_FL/PCM_CHANNEL_FR,
     * PCM_CHANNEL_LFE,
     * PCM_CHANNEL_FC,
     * PCM_CHANNEL_LS/PCM_CHANNEL_RS,
     * PCM_CHANNEL_CS,
     * PCM_CHANNEL_FLC/PCM_CHANNEL_FRC
     * PCM_CHANNEL_LB/PCM_CHANNEL_RB
     */
    if (!info)
        return;
    memset(info->channelMap, 0, MAX_CHANNELS_SUPPORTED);
    if(info->speakerAllocation[0] & BIT(0)) {
        info->channelMap[0] = PCM_CHANNEL_L;
        info->channelMap[1] = PCM_CHANNEL_R;
    }
    if(info->speakerAllocation[0] & BIT(1)) {
        info->channelMap[2] = PCM_CHANNEL_LFE;
    }
    if(info->speakerAllocation[0] & BIT(2)) {
        info->channelMap[3] = PCM_CHANNEL_C;
    }
    if(info->speakerAllocation[0] & BIT(3)) {
    /*
     * As per CEA(HDMI Cable) standard Bit 3 is equivalent
     * to SideLeft/SideRight of WAV standard
     */
        info->channelMap[4] = PCM_CHANNEL_LS;
        info->channelMap[5] = PCM_CHANNEL_RS;
    }
    if(info->speakerAllocation[0] & BIT(4)) {
        if(info->speakerAllocation[0] & BIT(3)) {
            info->channelMap[6] = PCM_CHANNEL_CS;
            info->channelMap[7] = 0;
        } else if (info->speakerAllocation[1] & BIT(1)) {
            info->channelMap[6] = PCM_CHANNEL_CS;
            info->channelMap[7] = PCM_CHANNEL_TS;
        } else if (info->speakerAllocation[1] & BIT(2)) {
            info->channelMap[6] = PCM_CHANNEL_CS;
            info->channelMap[7] = PCM_CHANNEL_CVH;
        } else {
            info->channelMap[4] = PCM_CHANNEL_CS;
            info->channelMap[5] = 0;
        }
    }
    if(info->speakerAllocation[0] & BIT(5)) {
        info->channelMap[6] = PCM_CHANNEL_FLC;
        info->channelMap[7] = PCM_CHANNEL_FRC;
    }
    if(info->speakerAllocation[0] & BIT(6)) {
        // If RLC/RRC is present, RC is invalid as per specification
        info->speakerAllocation[0] &= 0xef;
        /*
         * As per CEA(HDMI Cable) standard Bit 6 is equivalent
         * to BackLeft/BackRight of WAV standard
         */
        info->channelMap[6] = PCM_CHANNEL_LB;
        info->channelMap[7] = PCM_CHANNEL_RB;
    }
    // higher channel are not defined by LPASS
    //info->nSpeakerAllocation[0] &= 0x3f;
    if(info->speakerAllocation[0] & BIT(7)) {
        info->channelMap[6] = 0; // PCM_CHANNEL_FLW; but not defined by LPASS
        info->channelMap[7] = 0; // PCM_CHANNEL_FRW; but not defined by LPASS
    }
    if(info->speakerAllocation[1] & BIT(0)) {
        info->channelMap[6] = 0; // PCM_CHANNEL_FLH; but not defined by LPASS
        info->channelMap[7] = 0; // PCM_CHANNEL_FRH; but not defined by LPASS
    }

    PAL_VERBOSE(LOG_TAG," channel map updated to [%d %d %d %d %d %d %d %d ]  [%x %x %x]"
        , info->channelMap[0], info->channelMap[1], info->channelMap[2]
        , info->channelMap[3], info->channelMap[4], info->channelMap[5]
        , info->channelMap[6], info->channelMap[7]
        , info->speakerAllocation[0], info->speakerAllocation[1]
        , info->speakerAllocation[2]);
}

void DisplayPort::dumpSpeakerAllocation(eldAudioInfo* info)
{
    if (!info)
        return;

    if (info->speakerAllocation[0] & BIT(7))
        PAL_VERBOSE(LOG_TAG,"FLW/FRW");
    if (info->speakerAllocation[0] & BIT(6))
        PAL_VERBOSE(LOG_TAG,"RLC/RRC");
    if (info->speakerAllocation[0] & BIT(5))
        PAL_VERBOSE(LOG_TAG,"FLC/FRC");
    if (info->speakerAllocation[0] & BIT(4))
        PAL_VERBOSE(LOG_TAG,"RC");
    if (info->speakerAllocation[0] & BIT(3))
        PAL_VERBOSE(LOG_TAG,"RL/RR");
    if (info->speakerAllocation[0] & BIT(2))
        PAL_VERBOSE(LOG_TAG,"FC");
    if (info->speakerAllocation[0] & BIT(1))
        PAL_VERBOSE(LOG_TAG,"LFE");
    if (info->speakerAllocation[0] & BIT(0))
        PAL_VERBOSE(LOG_TAG,"FL/FR");
    if (info->speakerAllocation[1] & BIT(2))
        PAL_VERBOSE(LOG_TAG,"FCH");
    if (info->speakerAllocation[1] & BIT(1))
        PAL_VERBOSE(LOG_TAG,"TC");
    if (info->speakerAllocation[1] & BIT(0))
        PAL_VERBOSE(LOG_TAG,"FLH/FRH");
}

void DisplayPort::updateChannelAllocation(eldAudioInfo* info)
{
    int16_t ca;
    int16_t spkrAlloc;

    if (!info)
        return;

    /* Most common 5.1 SAD is 0xF, ca 0x0b
     * and 7.1 SAD is 0x4F, ca 0x13 */
    spkrAlloc = ((info->speakerAllocation[1]) << 8) |
               (info->speakerAllocation[0]);
    PAL_VERBOSE(LOG_TAG,"info->nSpeakerAllocation %x %x\n", info->speakerAllocation[0],
                                              info->speakerAllocation[1]);
    PAL_VERBOSE(LOG_TAG,"spkrAlloc: %x", spkrAlloc);

    /* The below switch case calculates channel allocation values
       as defined in CEA-861 section 6.6.2 */
    switch (spkrAlloc) {
    case BIT(0):                                           ca = 0x00; break;
    case BIT(0)|BIT(1):                                    ca = 0x01; break;
    case BIT(0)|BIT(2):                                    ca = 0x02; break;
    case BIT(0)|BIT(1)|BIT(2):                             ca = 0x03; break;
    case BIT(0)|BIT(4):                                    ca = 0x04; break;
    case BIT(0)|BIT(1)|BIT(4):                             ca = 0x05; break;
    case BIT(0)|BIT(2)|BIT(4):                             ca = 0x06; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(4):                      ca = 0x07; break;
    case BIT(0)|BIT(3):                                    ca = 0x08; break;
    case BIT(0)|BIT(1)|BIT(3):                             ca = 0x09; break;
    case BIT(0)|BIT(2)|BIT(3):                             ca = 0x0A; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3):                      ca = 0x0B; break;
    case BIT(0)|BIT(3)|BIT(4):                             ca = 0x0C; break;
    case BIT(0)|BIT(1)|BIT(3)|BIT(4):                      ca = 0x0D; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(4):                      ca = 0x0E; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4):               ca = 0x0F; break;
    case BIT(0)|BIT(3)|BIT(6):                             ca = 0x10; break;
    case BIT(0)|BIT(1)|BIT(3)|BIT(6):                      ca = 0x11; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(6):                      ca = 0x12; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(6):               ca = 0x13; break;
    case BIT(0)|BIT(5):                                    ca = 0x14; break;
    case BIT(0)|BIT(1)|BIT(5):                             ca = 0x15; break;
    case BIT(0)|BIT(2)|BIT(5):                             ca = 0x16; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(5):                      ca = 0x17; break;
    case BIT(0)|BIT(4)|BIT(5):                             ca = 0x18; break;
    case BIT(0)|BIT(1)|BIT(4)|BIT(5):                      ca = 0x19; break;
    case BIT(0)|BIT(2)|BIT(4)|BIT(5):                      ca = 0x1A; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(4)|BIT(5):               ca = 0x1B; break;
    case BIT(0)|BIT(3)|BIT(5):                             ca = 0x1C; break;
    case BIT(0)|BIT(1)|BIT(3)|BIT(5):                      ca = 0x1D; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(5):                      ca = 0x1E; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(5):               ca = 0x1F; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(10):                     ca = 0x20; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(10):              ca = 0x21; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(9):                      ca = 0x22; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(9):               ca = 0x23; break;
    case BIT(0)|BIT(3)|BIT(8):                             ca = 0x24; break;
    case BIT(0)|BIT(1)|BIT(3)|BIT(8):                      ca = 0x25; break;
    case BIT(0)|BIT(3)|BIT(7):                             ca = 0x26; break;
    case BIT(0)|BIT(1)|BIT(3)|BIT(7):                      ca = 0x27; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(4)|BIT(9):               ca = 0x28; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(9):        ca = 0x29; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(4)|BIT(10):              ca = 0x2A; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(10):       ca = 0x2B; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(9)|BIT(10):              ca = 0x2C; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(9)|BIT(10):       ca = 0x2D; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(8):                      ca = 0x2E; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(8):               ca = 0x2F; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(7):                      ca = 0x30; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(7):               ca = 0x31; break;
    default:                                               ca = 0x0;  break;
    }
    PAL_DBG(LOG_TAG," channel allocation: %x", ca);
    info->channelAllocation = ca;
}

void DisplayPort::retrieveChannelMapLpass(int ca, uint8_t *ch_map, int ch_map_size)
{
    if (!ch_map)
        return;

    if (((ca < 0) || (ca > 0x1f)) &&
         (ca != 0x2f)) {
        PAL_ERR(LOG_TAG,"Channel allocation out of supported range");
        return;
    }
    PAL_VERBOSE(LOG_TAG,"channelAllocation 0x%x", ca);

    if (ch_map_size < MAX_CHANNELS_SUPPORTED)
        return;

    memset(ch_map, 0, MAX_CHANNELS_SUPPORTED);

    switch(ca) {
    case 0x0:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        break;
    case 0x1:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        break;
    case 0x2:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_C;
        break;
    case 0x3:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_C;
        break;
    case 0x4:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_CS;
        break;
    case 0x5:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_CS;
        break;
    case 0x6:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_C;
        ch_map[3] = PCM_CHANNEL_CS;
        break;
    case 0x7:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_C;
        ch_map[4] = PCM_CHANNEL_CS;
        break;
    case 0x8:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LS;
        ch_map[3] = PCM_CHANNEL_RS;
        break;
    case 0x9:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_LS;
        ch_map[4] = PCM_CHANNEL_RS;
        break;
    case 0xa:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_C;
        ch_map[3] = PCM_CHANNEL_LS;
        ch_map[4] = PCM_CHANNEL_RS;
        break;
    case 0xb:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_C;
        ch_map[4] = PCM_CHANNEL_LS;
        ch_map[5] = PCM_CHANNEL_RS;
        break;
    case 0xc:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LS;
        ch_map[3] = PCM_CHANNEL_RS;
        ch_map[4] = PCM_CHANNEL_CS;
        break;
    case 0xd:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_LS;
        ch_map[4] = PCM_CHANNEL_RS;
        ch_map[5] = PCM_CHANNEL_CS;
        break;
    case 0xe:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_C;
        ch_map[3] = PCM_CHANNEL_LS;
        ch_map[4] = PCM_CHANNEL_RS;
        ch_map[5] = PCM_CHANNEL_CS;
        break;
    case 0xf:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_C;
        ch_map[4] = PCM_CHANNEL_LS;
        ch_map[5] = PCM_CHANNEL_RS;
        ch_map[6] = PCM_CHANNEL_CS;
        break;
    case 0x10:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LS;
        ch_map[3] = PCM_CHANNEL_RS;
        ch_map[4] = PCM_CHANNEL_LB;
        ch_map[5] = PCM_CHANNEL_RB;
        break;
    case 0x11:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_LS;
        ch_map[4] = PCM_CHANNEL_RS;
        ch_map[5] = PCM_CHANNEL_LB;
        ch_map[6] = PCM_CHANNEL_RB;
        break;
    case 0x12:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_C;
        ch_map[3] = PCM_CHANNEL_LS;
        ch_map[4] = PCM_CHANNEL_RS;
        ch_map[5] = PCM_CHANNEL_LB;
        ch_map[6] = PCM_CHANNEL_RB;
        break;
    case 0x13:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_C;
        ch_map[4] = PCM_CHANNEL_LS;
        ch_map[5] = PCM_CHANNEL_RS;
        ch_map[6] = PCM_CHANNEL_LB;
        ch_map[7] = PCM_CHANNEL_RB;
        break;
    case 0x14:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_FLC;
        ch_map[3] = PCM_CHANNEL_FRC;
        break;
    case 0x15:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_FLC;
        ch_map[4] = PCM_CHANNEL_FRC;
        break;
    case 0x16:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_C;
        ch_map[3] = PCM_CHANNEL_FLC;
        ch_map[4] = PCM_CHANNEL_FRC;
        break;
    case 0x17:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_C;
        ch_map[4] = PCM_CHANNEL_FLC;
        ch_map[5] = PCM_CHANNEL_FRC;
        break;
    case 0x18:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_CS;
        ch_map[3] = PCM_CHANNEL_FLC;
        ch_map[4] = PCM_CHANNEL_FRC;
        break;
    case 0x19:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_CS;
        ch_map[4] = PCM_CHANNEL_FLC;
        ch_map[5] = PCM_CHANNEL_FRC;
        break;
    case 0x1a:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_C;
        ch_map[3] = PCM_CHANNEL_CS;
        ch_map[4] = PCM_CHANNEL_FLC;
        ch_map[5] = PCM_CHANNEL_FRC;
        break;
    case 0x1b:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_C;
        ch_map[4] = PCM_CHANNEL_CS;
        ch_map[5] = PCM_CHANNEL_FLC;
        ch_map[6] = PCM_CHANNEL_FRC;
        break;
    case 0x1c:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LS;
        ch_map[3] = PCM_CHANNEL_RS;
        ch_map[4] = PCM_CHANNEL_FLC;
        ch_map[5] = PCM_CHANNEL_FRC;
        break;
    case 0x1d:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_LS;
        ch_map[4] = PCM_CHANNEL_RS;
        ch_map[5] = PCM_CHANNEL_FLC;
        ch_map[6] = PCM_CHANNEL_FRC;
        break;
    case 0x1e:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_C;
        ch_map[3] = PCM_CHANNEL_LS;
        ch_map[4] = PCM_CHANNEL_RS;
        ch_map[5] = PCM_CHANNEL_FLC;
        ch_map[6] = PCM_CHANNEL_FRC;
        break;
    case 0x1f:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_C;
        ch_map[4] = PCM_CHANNEL_LS;
        ch_map[5] = PCM_CHANNEL_RS;
        ch_map[6] = PCM_CHANNEL_FLC;
        ch_map[7] = PCM_CHANNEL_FRC;
        break;
    case 0x2f:
        ch_map[0] = PCM_CHANNEL_L;
        ch_map[1] = PCM_CHANNEL_R;
        ch_map[2] = PCM_CHANNEL_LFE;
        ch_map[3] = PCM_CHANNEL_C;
        ch_map[4] = PCM_CHANNEL_LS;
        ch_map[5] = PCM_CHANNEL_RS;
        ch_map[6] = 0; // PCM_CHANNEL_TFL; but not defined by LPASS
        ch_map[7] = 0; // PCM_CHANNEL_TFR; but not defined by LPASS
        break;
    default:
        break;
    }
    PAL_DBG(LOG_TAG," channel map updated to [%d %d %d %d %d %d %d %d ]",
          ch_map[0], ch_map[1], ch_map[2],
          ch_map[3], ch_map[4], ch_map[5],
          ch_map[6], ch_map[7]);
}

void DisplayPort::updateChannelMapLpass(eldAudioInfo* info)
{
    if (!info)
        return;

    retrieveChannelMapLpass(info->channelAllocation, (uint8_t *)&info->channelMap[0],
            MAX_CHANNELS_SUPPORTED);
}

void DisplayPort::updateChannelMask(eldAudioInfo* info)
{
    if (!info)
        return;
    if (((info->channelAllocation < 0) ||
         (info->channelAllocation > 0x1f)) &&
         (info->channelAllocation != 0x2f)) {
        PAL_ERR(LOG_TAG,"Channel allocation out of supported range");
        return;
    }
    PAL_VERBOSE(LOG_TAG,"channelAllocation 0x%x", info->channelAllocation);
    // Don't distinguish channel mask below?
    // AUDIO_CHANNEL_OUT_5POINT1 and AUDIO_CHANNEL_OUT_5POINT1_SIDE
    // AUDIO_CHANNEL_OUT_QUAD and AUDIO_CHANNEL_OUT_QUAD_SIDE
    switch(info->channelAllocation) {
    case 0x0:
        info->channelMask = AUDIO_CHANNEL_OUT_STEREO;
        break;
    case 0x1:
        info->channelMask = AUDIO_CHANNEL_OUT_2POINT1;
        break;
    case 0x2:
        info->channelMask = AUDIO_CHANNEL_OUT_STEREO;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_CENTER;
        break;
    case 0x3:
        info->channelMask = AUDIO_CHANNEL_OUT_2POINT1;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_CENTER;
        break;
    case 0x4:
        info->channelMask = AUDIO_CHANNEL_OUT_STEREO;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        break;
    case 0x5:
        info->channelMask = AUDIO_CHANNEL_OUT_2POINT1;
        info->channelMask |= AUDIO_CHANNEL_OUT_LOW_FREQUENCY;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        break;
    case 0x6:
        info->channelMask = AUDIO_CHANNEL_OUT_SURROUND;
        break;
    case 0x7:
        info->channelMask = AUDIO_CHANNEL_OUT_SURROUND;
        info->channelMask |= AUDIO_CHANNEL_OUT_LOW_FREQUENCY;
        break;
    case 0x8:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        break;
    case 0x9:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_LOW_FREQUENCY;
        break;
    case 0xa:
        info->channelMask = AUDIO_CHANNEL_OUT_PENTA;
        break;
    case 0xb:
        info->channelMask = AUDIO_CHANNEL_OUT_5POINT1;
        break;
    case 0xc:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        break;
    case 0xd:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_LOW_FREQUENCY;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        break;
    case 0xe:
        info->channelMask = AUDIO_CHANNEL_OUT_PENTA;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        break;
    case 0xf:
        info->channelMask = AUDIO_CHANNEL_OUT_5POINT1;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        break;
    case 0x10:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_SIDE_LEFT;
        info->channelMask |= AUDIO_CHANNEL_OUT_SIDE_RIGHT;
        break;
    case 0x11:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_LOW_FREQUENCY;
        info->channelMask |= AUDIO_CHANNEL_OUT_SIDE_LEFT;
        info->channelMask |= AUDIO_CHANNEL_OUT_SIDE_RIGHT;
        break;
    case 0x12:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_SIDE_LEFT;
        info->channelMask |= AUDIO_CHANNEL_OUT_SIDE_RIGHT;
        break;
    case 0x13:
        info->channelMask = AUDIO_CHANNEL_OUT_7POINT1;
        break;
    case 0x14:
        info->channelMask = AUDIO_CHANNEL_OUT_FRONT_LEFT;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x15:
        info->channelMask = AUDIO_CHANNEL_OUT_2POINT1;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x16:
        info->channelMask = AUDIO_CHANNEL_OUT_FRONT_LEFT;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x17:
        info->channelMask = AUDIO_CHANNEL_OUT_2POINT1;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x18:
        info->channelMask = AUDIO_CHANNEL_OUT_FRONT_LEFT;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x19:
        info->channelMask = AUDIO_CHANNEL_OUT_2POINT1;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x1a:
        info->channelMask = AUDIO_CHANNEL_OUT_SURROUND;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x1b:
        info->channelMask = AUDIO_CHANNEL_OUT_SURROUND;
        info->channelMask |= AUDIO_CHANNEL_OUT_LOW_FREQUENCY;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x1c:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x1d:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_LOW_FREQUENCY;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x1e:
        info->channelMask = AUDIO_CHANNEL_OUT_PENTA;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x1f:
        info->channelMask = AUDIO_CHANNEL_OUT_5POINT1;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x2f:
        info->channelMask = AUDIO_CHANNEL_OUT_5POINT1POINT2;
        break;
    default:
        break;
    }
    PAL_DBG(LOG_TAG," channel mask updated to %d", info->channelMask);
}

void DisplayPort::dumpEldData(eldAudioInfo *info)
{

    int i;
    for (i = 0; i < info->audioBlocks && i < MAX_ELD_BLOCKS; i++) {
        PAL_VERBOSE(LOG_TAG,"FormatId:%d rate:%d bps:%d channels:%d",
              info->audioBlocksArray[i].formatId,
              info->audioBlocksArray[i].samplingFreqBitmask,
              info->audioBlocksArray[i].bitsPerSampleBitmask,
              info->audioBlocksArray[i].channels);
    }
    PAL_VERBOSE(LOG_TAG,"no of audio blocks:%d", info->audioBlocks);
    PAL_VERBOSE(LOG_TAG,"speaker allocation:[%x %x %x]",
           info->speakerAllocation[0], info->speakerAllocation[1],
           info->speakerAllocation[2]);
    PAL_VERBOSE(LOG_TAG,"channel map:[%x %x %x %x %x %x %x %x]",
           info->channelMap[0], info->channelMap[1],
           info->channelMap[2], info->channelMap[3],
           info->channelMap[4], info->channelMap[5],
           info->channelMap[6], info->channelMap[7]);
    PAL_VERBOSE(LOG_TAG,"channel allocation:%d", info->channelAllocation);
    PAL_VERBOSE(LOG_TAG,"[%d %d %d %d %d %d %d %d ]",
           info->channelMap[0], info->channelMap[1],
           info->channelMap[2], info->channelMap[3],
           info->channelMap[4], info->channelMap[5],
           info->channelMap[6], info->channelMap[7]);
}

//eldData[0] will be count
//block received from mixerctl will begin from eldData[1]
bool DisplayPort::getSinkCaps(eldAudioInfo* info, char *eldData)
{
    unsigned char channels[MAX_SAD_BLOCKS];
    unsigned char formats[MAX_SAD_BLOCKS];
    unsigned char frequency[MAX_SAD_BLOCKS];
    unsigned char bitrate[MAX_SAD_BLOCKS];
    int monitorNameLength = 0;
    int i = 0;
    int length = (int) *eldData++;
    int countDesc = 0;
    int eldVersion = 0;

    eldVersion = GRAB_BITS(eldData, ELD_VER_OFFSET, ELD_VER_START_BIT, ELD_VER_NUM_BITS);
    if (eldVersion != ELD_VER_CEA_861D && eldVersion != ELD_VER_PARTIAL) {
         PAL_ERR(LOG_TAG, "HDMI: Unknown ELD version");
         return false;
    }

    PAL_VERBOSE(LOG_TAG,"Total length is %d",length);
    if (length < ELD_FIXED_BYTES) {
        PAL_ERR(LOG_TAG,"insufficient block length");
        return false;
    }

    countDesc = GRAB_BITS(eldData, SAD_COUNT_OFFSET, SAD_COUNT_START_BIT, SAD_COUNT_NUM_BITS); //SAD count
    if (!countDesc) {
        PAL_ERR(LOG_TAG,"insufficient descriptors");
        return false;
    }
    PAL_VERBOSE(LOG_TAG,"Total # of audio descriptors %d",countDesc);

    monitorNameLength = GRAB_BITS(eldData, MNL_OFFSET, MNL_START_BIT, MNL_NUM_BITS);

    memset(info, 0, sizeof(eldAudioInfo));

    info->audioBlocks = countDesc;
    if (info->audioBlocks > MAX_SAD_BLOCKS) {
        info->audioBlocks = MAX_SAD_BLOCKS;
    }

    info->speakerAllocation[0] = GRAB_BITS(eldData, SPK_ALLOC_OFFSET, SPK_ALLOC_START_BIT, SPK_ALLOC_NUM_BITS);
    if(!info->speakerAllocation[0])
        info->speakerAllocation[0] = 0xFF;

    for (i=0; i<info->audioBlocks; i++) {
        if (ELD_FIXED_BYTES + monitorNameLength  + SAD_BLOCK_SIZE * (i + 1) > length) {
                PAL_ERR(LOG_TAG,"insufficient block length for SAD");
                return false;
        }
        channels [i]   = GRAB_BITS((eldData + ELD_FIXED_BYTES + monitorNameLength +  SAD_BLOCK_SIZE* i), SAD_CHNL_BYTE, SAD_CHNL_START_BIT, SAD_CHNL_NUM_BITS) + 1;
        formats  [i]   = GRAB_BITS((eldData + ELD_FIXED_BYTES + monitorNameLength +  SAD_BLOCK_SIZE* i), SAD_FMT_BYTE, SAD_FMT_START_BIT, SAD_FMT_NUM_BITS);
        frequency[i]   = GRAB_BITS((eldData + ELD_FIXED_BYTES + monitorNameLength +  SAD_BLOCK_SIZE* i), SAD_FREQ_BYTE, SAD_FREQ_START_BIT, SAD_FREQ_NUM_BITS);
        bitrate  [i]   = GRAB_BITS((eldData + ELD_FIXED_BYTES + monitorNameLength +  SAD_BLOCK_SIZE* i), SAD_BITRATE_BYTE, SAD_BITRATE_START_BIT, SAD_BITRATE_NUM_BITS);
    }
    updateChannelMap(info);
    updateChannelAllocation(info);
    updateChannelMapLpass(info);
    updateChannelMask(info);

    for (i=0; i<info->audioBlocks; i++) {
        PAL_VERBOSE(LOG_TAG,"AUDIO DESC BLOCK # %d\n",i);

        info->audioBlocksArray[i].channels = channels[i];
        PAL_DBG(LOG_TAG,"info->audioBlocksArray[i].channels %d\n",
              info->audioBlocksArray[i].channels);

        PAL_VERBOSE(LOG_TAG,"Format Byte %d\n", formats[i]);
        info->audioBlocksArray[i].formatId = (eldAudioFormatId)formats[i];
        PAL_DBG(LOG_TAG,"info->audioBlocksArray[i].formatId %s",
             eldFormatToStr(formats[i]));

        PAL_VERBOSE(LOG_TAG,"Frequency Bitmask %d\n", frequency[i]);
        info->audioBlocksArray[i].samplingFreqBitmask = frequency[i];
        PAL_VERBOSE(LOG_TAG,"info->audioBlocksArray[i].samplingFreqBitmask %d",
              info->audioBlocksArray[i].samplingFreqBitmask);

        PAL_VERBOSE(LOG_TAG,"BitsPerSample Bitmask %d\n", bitrate[i]);
        info->audioBlocksArray[i].bitsPerSampleBitmask =
                   getEldBpsByte(bitrate[i],formats[i]);
        PAL_VERBOSE(LOG_TAG,"info->audioBlocksArray[i].bitsPerSampleBitmask %d",
              info->audioBlocksArray[i].bitsPerSampleBitmask);
    }
    dumpSpeakerAllocation(info);
    dumpEldData(info);
    return true;
}

bool DisplayPort::isSupportedSR(eldAudioInfo* info, int sr)
{
    int i = 0;
    struct extDispState *state = NULL;

    state = &extDisp[dp_controller][dp_stream];
    if (state && state->eldInfo)
    {
        info = (eldAudioInfo*) state->eldInfo;
    }
    if (info != NULL && sr != 0) {
        for (i = 0; i < info->audioBlocks && i < MAX_ELD_BLOCKS; i++) {
                if (isSampleRateSupported(info->audioBlocksArray[i].samplingFreqBitmask,
                    sr)) {
                        PAL_DBG(LOG_TAG," Returns true for sample rate [%d]", sr);
                        return true;
                }
        }
    }
    PAL_ERR(LOG_TAG," Returns false for sample rate [%d]", sr);
    return false;
}

int DisplayPort::getMaxChannel()
{
    int i = 0;
    struct extDispState *state = NULL;
    int max_channel = 2;
    eldAudioInfo *info = NULL;

    state = &extDisp[dp_controller][dp_stream];
    if (state && state->eldInfo)
    {
        info = (eldAudioInfo*) state->eldInfo;
    }

    if (info != NULL) {
        for (i = 0; i < info->audioBlocks && i < MAX_ELD_BLOCKS; i++) {
                if (info->audioBlocksArray[i].formatId == LPCM) {
                        if (max_channel < info->audioBlocksArray[i].channels) {
                                max_channel = info->audioBlocksArray[i].channels;
                                PAL_DBG(LOG_TAG," Max channels updated to [%d]", max_channel);
                        }
                }
        }
    }
    return max_channel;
}

bool DisplayPort::isSupportedBps(eldAudioInfo* info, int bps)
{
    int i = 0;

    if (bps == 16) {
        //16 bit bps is always supported
        //some oem may not update 16bit support in their eld info
        return true;
    }

    if (info != NULL && bps != 0) {
        for (i = 0; i < info->audioBlocks && i < MAX_ELD_BLOCKS; i++) {
                if (isSupportedBps(info->audioBlocksArray[i].bitsPerSampleBitmask, bps)) {
                        PAL_VERBOSE(LOG_TAG," returns true for bit width [%d]", bps);
                        return true;
                }
        }
    }
    PAL_VERBOSE(LOG_TAG," returns false for bit width [%d]", bps);
    return false;
}

int DisplayPort::getHighestSupportedSR()
{
    int sr = 0;
    int highestSR = 0;
    int i;
    struct extDispState *state = NULL;
    eldAudioInfo *info = NULL;

    state = &extDisp[dp_controller][dp_stream];
    if (state && state->eldInfo)
    {
        info = (eldAudioInfo*) state->eldInfo;
    }

    if (info != NULL) {
        for (i = 0; i < info->audioBlocks && i < MAX_ELD_BLOCKS; i++) {
                sr = getHighestEldSF(info->audioBlocksArray[i].samplingFreqBitmask);
                if (sr > highestSR)
                        highestSR = sr;
        }
    }
    else {
        PAL_ERR(LOG_TAG," info is NULL");
        highestSR = SAMPLINGRATE_48K;
    }

    if (highestSR == 0) {
        PAL_ERR(LOG_TAG,"Unable to get Highest SR. Setting default SR");
        highestSR = SAMPLINGRATE_48K;
    }

    PAL_VERBOSE(LOG_TAG," returns [%d] for highest supported sr", highestSR);
    return highestSR;
}

int DisplayPort::getHighestSupportedBps()
{
    int bpsMask = 0;
    int highestBps = 0;
    int i;
    struct extDispState *state = NULL;
    eldAudioInfo *info = NULL;

    state = &extDisp[dp_controller][dp_stream];
    if (state && state->eldInfo)
    {
        info = (eldAudioInfo*) state->eldInfo;
    }

    if (info != NULL) {
        for (i = 0; i < info->audioBlocks && i < MAX_ELD_BLOCKS; i++) {
                bpsMask = info->audioBlocksArray[i].bitsPerSampleBitmask;
                if (isSupportedBps(bpsMask, 24)) {
                        highestBps = 24;
                        break;
                }
                else if (isSupportedBps(bpsMask, BITWIDTH_16))
                        if (highestBps < BITWIDTH_16)
                                highestBps = BITWIDTH_16;
        }
    }

    if (highestBps == 0) {
        PAL_ERR(LOG_TAG, "None of the supported BPS is highest");
        highestBps = 16;
    }
    return highestBps;
}

bool DisplayPort::isDpDevice(pal_device_id_t id) {
    if (id == PAL_DEVICE_OUT_AUX_DIGITAL || id == PAL_DEVICE_OUT_AUX_DIGITAL_1 ||
        id == PAL_DEVICE_OUT_HDMI)
        return true;
    else
        return false;
}

