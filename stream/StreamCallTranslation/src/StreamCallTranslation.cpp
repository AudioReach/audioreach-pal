/*
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "StreamCallTranslation"

#include "StreamCallTranslation.h"
#include "Session.h"
#include "ResourceManager.h"
#include <unistd.h>
#include "MemLogBuilder.h"


extern "C" Stream* CreateCallTranslationStream(const struct pal_stream_attributes *sattr, struct pal_device *dattr,
                                  const uint32_t no_of_devices, const struct modifier_kv *modifiers,
                                  const uint32_t no_of_modifiers, const std::shared_ptr<ResourceManager> rm) {
       return new StreamCallTranslation(sattr, dattr, no_of_devices, modifiers, no_of_modifiers, rm);
}

StreamCallTranslation::StreamCallTranslation(const struct pal_stream_attributes *sattr __unused,
                       struct pal_device *dattr __unused, const uint32_t no_of_devices __unused,
                       const struct modifier_kv *modifiers __unused, const uint32_t no_of_modifiers __unused,
                       const std::shared_ptr<ResourceManager> rm)
{
    mStreamMutex.lock();
    uint32_t in_channels = 0, out_channels = 0;
    uint32_t attribute_size = 0;

    session = NULL;
    mGainLevel = -1;
    std::shared_ptr<Device> dev = nullptr;
    mStreamAttr = (struct pal_stream_attributes *)nullptr;
    mDevices.clear();
    currentState = STREAM_IDLE;
    //Modify cached values only at time of SSR down.
    cachedState = STREAM_IDLE;
    cookie_ = 0;
    bool isDeviceConfigUpdated = false;

    PAL_DBG(LOG_TAG, "Enter");

    //TBD handle modifiers later
    mNoOfModifiers = 0; //no_of_modifiers;
    mModifiers = (struct modifier_kv *) (NULL);
    std::ignore = modifiers;
    std::ignore = no_of_modifiers;

    if (!sattr) {
        PAL_ERR(LOG_TAG,"Error:invalid arguments");
        mStreamMutex.unlock();
        throw std::runtime_error("invalid arguments");
    }

    attribute_size = sizeof(struct pal_stream_attributes);
    mStreamAttr = (struct pal_stream_attributes *) calloc(1, attribute_size);
    if (!mStreamAttr) {
        PAL_ERR(LOG_TAG, "Error:malloc for stream attributes failed %s", strerror(errno));
        mStreamMutex.unlock();
        throw std::runtime_error("failed to malloc for stream attributes");
    }

    memcpy(mStreamAttr, sattr, sizeof(pal_stream_attributes));

    if (mStreamAttr->in_media_config.ch_info.channels > PAL_MAX_CHANNELS_SUPPORTED) {
        PAL_ERR(LOG_TAG,"Error:in_channels is invalid %d", in_channels);
        mStreamAttr->in_media_config.ch_info.channels = PAL_MAX_CHANNELS_SUPPORTED;
    }
    if (mStreamAttr->out_media_config.ch_info.channels > PAL_MAX_CHANNELS_SUPPORTED) {
        PAL_ERR(LOG_TAG,"Error:out_channels is invalid %d", out_channels);
        mStreamAttr->out_media_config.ch_info.channels = PAL_MAX_CHANNELS_SUPPORTED;
    }

    PAL_VERBOSE(LOG_TAG, "Create new Session for stream type %d", sattr->type);
    session = Session::makeSession(rm, sattr);
    if (!session) {
        PAL_ERR(LOG_TAG, "Error:session creation failed");
        mStreamMutex.unlock();
        throw std::runtime_error("failed to create session object");
    }

    PAL_VERBOSE(LOG_TAG, "Create new Devices with no_of_devices - %d", no_of_devices);
    /* update handset/speaker sample rate for UPD with shared backend */
    if ((sattr->type == PAL_STREAM_ULTRASOUND ||
         sattr->type == PAL_STREAM_SENSOR_PCM_RENDERER) && !rm->IsDedicatedBEForUPDEnabled()) {
        struct pal_device devAttr = {};
        struct pal_device_info inDeviceInfo;
        pal_device_id_t upd_dev[] = {PAL_DEVICE_OUT_SPEAKER, PAL_DEVICE_OUT_HANDSET};
        for (int i = 0; i < sizeof(upd_dev)/sizeof(upd_dev[0]); i++) {
            devAttr.id = upd_dev[i];
            dev = Device::getInstance(&devAttr, rm);
            if (!dev)
                continue;
            rm->getDeviceInfo(devAttr.id, sattr->type, "", &inDeviceInfo);
            dev->setSampleRate(inDeviceInfo.samplerate);
            if (devAttr.id == PAL_DEVICE_OUT_HANDSET)
                dev->setBitWidth(inDeviceInfo.bit_width);
        }
    }
    for (int i = 0; i < no_of_devices; i++) {
        //Check with RM if the configuration given can work or not
        //for e.g., if incoming stream needs 24 bit device thats also
        //being used by another stream, then the other stream should route

        dev = Device::getInstance((struct pal_device *)&dattr[i] , rm);
        if (!dev) {
            PAL_ERR(LOG_TAG, "Error:Device creation failed");
            //TBD::free session too
            mStreamMutex.unlock();
            throw std::runtime_error("failed to create device object");
        }
        dev->insertStreamDeviceAttr(&dattr[i], this);
        mPalDevices.push_back(dev);
        mStreamMutex.unlock();
        // streams with VA MIC is handled in rm::handleConcurrentStreamSwitch()
        if (dattr[i].id != PAL_DEVICE_IN_HANDSET_VA_MIC &&
            dattr[i].id != PAL_DEVICE_IN_HEADSET_VA_MIC)
            isDeviceConfigUpdated = rm->updateDeviceConfig(&dev, &dattr[i], sattr);
        mStreamMutex.lock();

        if (isDeviceConfigUpdated)
            PAL_VERBOSE(LOG_TAG, "Device config updated");

        /* Create only update device attributes first time so update here using set*/
        /* this will have issues if same device is being currently used by different stream */
        mDevices.push_back(dev);
    }
    mStreamMutex.unlock();
    rm->registerStream(this);
    PAL_DBG(LOG_TAG, "Exit. state %d", currentState);
    return;
}

StreamCallTranslation::~StreamCallTranslation() {
    rm->deregisterStream(this);
}

int32_t  StreamCallTranslation::open()
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter. session handle - %pK device count - %zu", session,
            mDevices.size());

    mStreamMutex.lock();
    if (PAL_CARD_STATUS_DOWN(rm->getSoundCardState())) {
        PAL_ERR(LOG_TAG, "Error:Sound card offline/standby, can not open stream");
        usleep(SSR_RECOVERY);
        status = -EIO;
        goto exit;
    }

    if (currentState == STREAM_IDLE) {
        for (int32_t i = 0; i < mDevices.size(); i++) {
            status = mDevices[i]->open();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Error:device open failed with status %d", status);
                goto exit;
            }
        }

        status = session->open(this);
        if (0 != status) {
            PAL_ERR(LOG_TAG, "Error:session open failed with status %d", status);
            goto closeDevice;
        }
        PAL_VERBOSE(LOG_TAG, "session open successful");
        currentState = STREAM_INIT;
        PAL_DBG(LOG_TAG, "streamLL opened. state %d", currentState);
        goto exit;
    } else if (currentState == STREAM_INIT) {
        PAL_INFO(LOG_TAG, "Stream is already opened, state %d", currentState);
        status = 0;
        goto exit;
    } else {
        PAL_ERR(LOG_TAG, "Error:Stream is not in correct state %d", currentState);
        //TBD : which error code to return here.
        status = -EINVAL;
        goto exit;
    }
closeDevice:
    for (int32_t i = 0; i < mDevices.size(); i++) {
        status = mDevices[i]->close();
        if (0 != status) {
            PAL_ERR(LOG_TAG, "device close is failed with status %d", status);
        }
    }
exit:
    palStateEnqueue(this, PAL_STATE_OPENED, status);
    mStreamMutex.unlock();
    PAL_DBG(LOG_TAG, "Exit ret %d", status)
    return status;
}

int32_t  StreamCallTranslation::close()
{
    int32_t status = 0;
    mStreamMutex.lock();

    if (currentState == STREAM_IDLE) {
        PAL_INFO(LOG_TAG, "Stream is already closed");
        mStreamMutex.unlock();
        return status;
    }

    PAL_DBG(LOG_TAG, "Enter. session handle - %pK device count - %zu stream_type - %d state %d",
             session, mDevices.size(), mStreamAttr->type, currentState);

    if (currentState == STREAM_STARTED || currentState == STREAM_PAUSED) {
        mStreamMutex.unlock();
        status = stop();
        if (0 != status)
            PAL_ERR(LOG_TAG, "Error:stream stop failed. status %d",  status);
        mStreamMutex.lock();
    }

    rm->lockGraph();
    status = session->close(this);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Error:session close failed with status %d", status);
    }

    for (int32_t i = 0; i < mDevices.size(); i++) {
        status = mDevices[i]->close();
        if (0 != status) {
            PAL_ERR(LOG_TAG, "Error:device close is failed with status %d", status);
        }
    }
    PAL_VERBOSE(LOG_TAG, "closed the devices successfully");
    currentState = STREAM_IDLE;
    rm->unlockGraph();
    rm->checkAndSetDutyCycleParam();
    palStateEnqueue(this, PAL_STATE_CLOSED, status);
    mStreamMutex.unlock();

    PAL_DBG(LOG_TAG, "Exit. closed the stream successfully %d status %d",
             currentState, status);
    return status;
}

int32_t StreamCallTranslation::start()
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter. session handle - %pK mStreamAttr->direction - %d state %d",
            session, mStreamAttr->direction, currentState);

    mStreamMutex.lock();

    if (PAL_CARD_STATUS_DOWN(rm->getSoundCardState())) {
        cachedState = STREAM_STARTED;
        PAL_ERR(LOG_TAG, "Error:Sound card offline/standby. Update the cached state %d",
                cachedState);
        goto exit;
    }

    if (currentState == STREAM_INIT || currentState == STREAM_STOPPED) {
        rm->lockGraph();
        status = start_device();
        if (0 != status) {
            rm->unlockGraph();
            goto exit;
        }
        PAL_VERBOSE(LOG_TAG, "device started successfully");
        status = startSession();
        if (0 != status) {
            rm->unlockGraph();
            goto exit;
        }
        rm->unlockGraph();
        PAL_VERBOSE(LOG_TAG, "session start successful");

        /*pcm_open and pcm_start done at once here,
         *so directly jump to STREAM_STARTED state.
         */
        currentState = STREAM_STARTED;
        mStreamMutex.unlock();
        rm->lockActiveStream();
        mStreamMutex.lock();
        for (int i = 0; i < mDevices.size(); i++) {
            rm->registerDevice(mDevices[i], this);
        }
        rm->unlockActiveStream();
        rm->checkAndSetDutyCycleParam();
    } else if (currentState == STREAM_STARTED) {
        PAL_INFO(LOG_TAG, "Stream already started, state %d", currentState);
    } else {
        PAL_ERR(LOG_TAG, "Error:Stream is not opened yet");
        status = -EINVAL;
    }
exit:
    palStateEnqueue(this, PAL_STATE_STARTED, status);
    PAL_DBG(LOG_TAG, "Exit. state %d", currentState);
    mStreamMutex.unlock();
    return status;
}

int32_t StreamCallTranslation::start_device()
{
    int32_t status = 0;
    for (int32_t i=0; i < mDevices.size(); i++) {
         status = mDevices[i]->start();
         if (0 != status) {
             PAL_ERR(LOG_TAG, "Error:%s device start is failed with status %d",
                     GET_DIR_STR(mStreamAttr->direction), status);
         }
    }
    return status;
}

int32_t StreamCallTranslation::startSession()
{
    int32_t status = 0, devStatus = 0;
    status = session->prepare(this);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Error:%s session prepare is failed with status %d",
                    GET_DIR_STR(mStreamAttr->direction), status);
        goto session_fail;
    }
    PAL_VERBOSE(LOG_TAG, "session prepare successful");

    status = session->start(this);
    if (errno == -ENETRESET) {
        if (PAL_CARD_STATUS_DOWN(rm->getSoundCardState())) {
            PAL_ERR(LOG_TAG, "Error:Sound card offline/standby, informing RM");
            rm->ssrHandler(CARD_STATUS_OFFLINE);
        }
        cachedState = STREAM_STARTED;
        status = 0;
        goto session_fail;
    }
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Error:%s session start is failed with status %d",
                    GET_DIR_STR(mStreamAttr->direction), status);
        goto session_fail;
    }
    goto exit;

session_fail:
    for (int32_t i=0; i < mDevices.size(); i++) {
        devStatus = mDevices[i]->stop();
        if (devStatus)
            status = devStatus;
    }
exit:
    return status;
}

int32_t StreamCallTranslation::stop()
{
    int32_t status = 0;

    mStreamMutex.lock();
    PAL_DBG(LOG_TAG, "Enter. session handle - %pK mStreamAttr->direction - %d state %d",
                session, mStreamAttr->direction, currentState);

    if (currentState == STREAM_STARTED || currentState == STREAM_PAUSED) {
        mStreamMutex.unlock();
        rm->lockActiveStream();
        mStreamMutex.lock();
        currentState = STREAM_STOPPED;
        for (int i = 0; i < mDevices.size(); i++) {
            rm->deregisterDevice(mDevices[i], this);
        }
        rm->unlockActiveStream();
        PAL_VERBOSE(LOG_TAG, "In %s, device count - %zu",
                    GET_DIR_STR(mStreamAttr->direction), mDevices.size());

        rm->lockGraph();
        status = session->stop(this);
        if (0 != status) {
            PAL_ERR(LOG_TAG, "Error:%s session stop failed with status %d",
                    GET_DIR_STR(mStreamAttr->direction), status);
        }
        PAL_VERBOSE(LOG_TAG, "session stop successful");
        for (int32_t i=0; i < mDevices.size(); i++) {
             status = mDevices[i]->stop();
             if (0 != status) {
                 PAL_ERR(LOG_TAG, "Error:%s device stop failed with status %d",
                         GET_DIR_STR(mStreamAttr->direction), status);
             }
        }
        rm->unlockGraph();
        PAL_VERBOSE(LOG_TAG, "devices stop successful");
    } else if (currentState == STREAM_STOPPED || currentState == STREAM_IDLE) {
        PAL_INFO(LOG_TAG, "Stream is already in Stopped state %d", currentState);
    } else {
        PAL_ERR(LOG_TAG, "Error:Stream should be in start/pause state, %d", currentState);
        status = -EINVAL;
    }
    palStateEnqueue(this, PAL_STATE_STOPPED, status);
    PAL_DBG(LOG_TAG, "Exit. status %d, state %d", status, currentState);

    mStreamMutex.unlock();
    return status;
}
