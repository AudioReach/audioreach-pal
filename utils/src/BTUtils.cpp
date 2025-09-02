/*
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "PAL: BTUtils"
#include "ResourceManager.h"
#include "PalDefs.h"
#include <vector>
#include <dlfcn.h>
#include "Device.h"
#include <bt_intf.h>

#define CLOCK_SRC_DEFAULT 1

static std::map<std::pair<uint32_t, std::string>, std::string> btCodecMap;
static std::map<uint32_t, uint32_t> btSlimClockSrcMap;
int32_t scoOutConnectCount = 0;
int32_t scoInConnectCount = 0;

#define MAKE_STRING_FROM_ENUM(string) { {#string}, string }
static std::map<std::string, uint32_t> btFmtTable = {
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_AAC),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_SBC),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_APTX),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_APTX_HD),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_APTX_DUAL_MONO),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_LDAC),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_CELT),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_APTX_AD),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_APTX_AD_SPEECH),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_LC3),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_PCM),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_APTX_AD_QLEA),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_APTX_AD_R4)
};

template <class T>
void SortAndUnique(std::vector<T> &streams)
{
    std::sort(streams.begin(), streams.end());
    typename std::vector<T>::iterator iter =
        std::unique(streams.begin(), streams.end());
    streams.erase(iter, streams.end());
    return;
}

void BTUtilsInit() {
    btCodecMap.clear();
    btSlimClockSrcMap.clear();
}

int32_t BTUtilsDeviceNotReadyToDummy(Stream *s, bool& a2dpSuspend)
{
    int32_t status = 0;
    int sndDevId = 0;
    struct pal_device dattr = {};
    struct pal_device switchDevDattr = {};
    std::shared_ptr<Device> dev = nullptr;
    std::shared_ptr<Device> switchDev = nullptr;
    pal_param_bta2dp_t *param_bt_a2dp = nullptr;
    std::vector <Stream *> activeStreams;
    struct pal_stream_attributes sAttr;
    std::vector<std::shared_ptr<Device>> mDevices;
    Session *session = nullptr;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    status = s->getStreamAttributes(&sAttr);
    if(status != 0) {
        PAL_ERR(LOG_TAG,"getStreamAttributes Failed \n");
        goto exit;
    }

    status = s->getAssociatedDevices(mDevices);
    if (status != 0) {
        PAL_ERR(LOG_TAG,"getAssociatedDevices failed");
        goto exit;
    }

    s->getAssociatedSession(&session);
    a2dpSuspend = false;

    /* SCO device is not ready */
    if (rm->isDeviceAvailable(mDevices, PAL_DEVICE_OUT_BLUETOOTH_SCO) &&
        !rm->isDeviceReady(PAL_DEVICE_OUT_BLUETOOTH_SCO)) {
        // If it's sco + speaker combo device, route to speaker.
        // Otherwise, return -EAGAIN.
        if (rm->isDeviceAvailable(mDevices, PAL_DEVICE_OUT_SPEAKER)) {
            PAL_INFO(LOG_TAG, "BT SCO output device is not ready, route to speaker");
            for (auto iter = mDevices.begin(); iter != mDevices.end();) {
                if ((*iter)->getSndDeviceId() == PAL_DEVICE_OUT_SPEAKER) {
                    iter++;
                    continue;
                }

                // Invoke session API to explicitly update the device metadata
                rm->lockGraph();
                status = session->disconnectSessionDevice(s, sAttr.type, (*iter));
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "disconnectSessionDevice failed:%d", status);
                    rm->unlockGraph();
                    goto exit;
                }

                status = (*iter)->close();
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "device close failed with status %d", status);
                    rm->unlockGraph();
                    goto exit;
                }
                iter = mDevices.erase(iter);
                s->removemDevice((*iter)->getSndDeviceId());
                s->removePalDevice(s, PAL_DEVICE_OUT_BLUETOOTH_SCO);
                rm->unlockGraph();
            }
        } else {
            PAL_ERR(LOG_TAG, "BT SCO output device is not ready");
            status = -EAGAIN;
            goto exit;
        }
    }

    /* A2DP/BLE device is not ready */
    if (rm->isDeviceAvailable(mDevices, PAL_DEVICE_OUT_BLUETOOTH_A2DP) ||
        rm->isDeviceAvailable(mDevices, PAL_DEVICE_OUT_BLUETOOTH_BLE) ||
        rm->isDeviceAvailable(mDevices, PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST)) {
        if (rm->isDeviceAvailable(mDevices, PAL_DEVICE_OUT_BLUETOOTH_A2DP)) {
            dattr.id = PAL_DEVICE_OUT_BLUETOOTH_A2DP;
        } else if (rm->isDeviceAvailable(mDevices, PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST)){
            dattr.id = PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST;
        } else {
            dattr.id = PAL_DEVICE_OUT_BLUETOOTH_BLE;
        }
        dev = Device::getInstance(&dattr, rm);
        if (!dev) {
            status = -ENODEV;
            PAL_ERR(LOG_TAG, "failed to get a2dp/ble device object");
            goto exit;
        }
        dev->getDeviceParameter(PAL_PARAM_ID_BT_A2DP_SUSPENDED,
                        (void **)&param_bt_a2dp);
        if (param_bt_a2dp->a2dp_suspended == false) {
            PAL_DBG(LOG_TAG, "BT A2DP/BLE output device is good to go");
            goto exit;
        }

        PAL_INFO(LOG_TAG, "BT A2DP/BLE output device is not ready");

        s->suspendedOutDevIds.clear();
        s->suspendedOutDevIds.push_back(dattr.id);

        for (auto iter = mDevices.begin(); iter != mDevices.end();) {
            sndDevId = (*iter)->getSndDeviceId();
            if (sndDevId == dattr.id) {
                rm->lockGraph();
                status = session->disconnectSessionDevice(s, sAttr.type, (*iter));
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "disconnectSessionDevice failed:%d", status);
                    rm->unlockGraph();
                    goto exit;
                }

                /* Special handling for aaudio usecase on A2DP/BLE.
                * A2DP/BLE device starts even when stream is not in START state,
                * hence stop A2DP/BLE device to match device start&stop count.
                */
                if (s->isMMap) {
                    status = (*iter)->stop();
                    if (0 != status) {
                        PAL_ERR(LOG_TAG, "BT A2DP/BLE device stop failed with status %d", status);
                    }
                }

                status = (*iter)->close();
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "device close failed with status %d", status);
                    rm->unlockGraph();
                    goto exit;
                }
                iter = mDevices.erase(iter);
                s->removemDevice((*iter)->getSndDeviceId());
                s->removePalDevice(s, sndDevId);
                rm->unlockGraph();
            } else {
                iter++;
            }
        }

        if (mDevices.size() == 0) {
            a2dpSuspend = true;
            switchDevDattr.id = PAL_DEVICE_OUT_DUMMY;
            switchDev = Device::getInstance(&switchDevDattr, rm);
            if (!switchDev) {
                status = -ENODEV;
                PAL_ERR(LOG_TAG, "Failed to get out dummy device instance");
                goto exit;
            }

            status = rm->getDeviceConfig(&switchDevDattr, NULL);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to get device config");
                goto exit;
            }

            switchDev->setDeviceAttributes(switchDevDattr);
            status = switchDev->open();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "device open failed with status %d", status);
                goto exit;
            }

            status = session->setupSessionDevice(s, sAttr.type, switchDev);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "setupSessionDevice failed:%d", status);
                switchDev->close();
                goto exit;
            }

            status = session->connectSessionDevice(s, sAttr.type, switchDev);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "connectSessionDevice failed:%d", status);
                switchDev->close();
                goto exit;
            }
            mDevices.push_back(switchDev);
            s->addmDevice(&switchDevDattr);
            s->addPalDevice(s, &switchDevDattr);
        } else {
            s->suspendedOutDevIds.push_back(switchDevDattr.id);
        }
    }

exit:
    return status;
}

int32_t BTUtilsDeviceNotReady(Stream *s, bool& a2dpSuspend)
{
    int32_t status = 0;
    struct pal_device dattr = {};
    struct pal_device spkrDattr = {};
    struct pal_device handsetDattr = {};
    std::shared_ptr<Device> dev = nullptr;
    std::shared_ptr<Device> spkrDev = nullptr;
    std::shared_ptr<Device> handsetDev = nullptr;
    pal_param_bta2dp_t *param_bt_a2dp = nullptr;
    std::vector <Stream *> activeStreams;
    struct pal_stream_attributes sAttr;
    std::vector<std::shared_ptr<Device>> mDevices;
    Session *session = nullptr;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    status = s->getStreamAttributes(&sAttr);
    if(status != 0) {
        PAL_ERR(LOG_TAG,"getStreamAttributes Failed \n");
        goto exit;
    }

    status = s->getAssociatedDevices(mDevices);
    if (status != 0) {
        PAL_ERR(LOG_TAG,"getAssociatedDevices failed");
        goto exit;
    }

    s->getAssociatedSession(&session);
    a2dpSuspend = false;

    /* Check for BT device connected state */
    for (int32_t i = 0; i < mDevices.size(); i++) {
        pal_device_id_t dev_id = (pal_device_id_t)mDevices[i]->getSndDeviceId();
        if (rm->isBtDevice(dev_id) && !(rm->isDeviceAvailable(dev_id))) {
            PAL_ERR(LOG_TAG, "BT device %d not connected", dev_id);
            status = -ENODEV;
            goto exit;
        }
    }

    /* SCO device is not ready */
    if (rm->isDeviceAvailable(mDevices, PAL_DEVICE_OUT_BLUETOOTH_SCO) &&
        !rm->isDeviceReady(PAL_DEVICE_OUT_BLUETOOTH_SCO)) {
        // If it's sco + speaker combo device, route to speaker.
        // Otherwise, return -EAGAIN.
        if (rm->isDeviceAvailable(mDevices, PAL_DEVICE_OUT_SPEAKER)) {
            PAL_INFO(LOG_TAG, "BT SCO output device is not ready, route to speaker");
            for (auto iter = mDevices.begin(); iter != mDevices.end();) {
                if ((*iter)->getSndDeviceId() == PAL_DEVICE_OUT_SPEAKER) {
                    iter++;
                    continue;
                }

                // Invoke session API to explicitly update the device metadata
                rm->lockGraph();
                status = session->disconnectSessionDevice(s, sAttr.type, (*iter));
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "disconnectSessionDevice failed:%d", status);
                    rm->unlockGraph();
                    goto exit;
                }

                status = (*iter)->close();
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "device close failed with status %d", status);
                    rm->unlockGraph();
                    goto exit;
                }
                iter = mDevices.erase(iter);
                s->removemDevice((*iter)->getSndDeviceId());
                rm->unlockGraph();
            }
        } else {
            PAL_ERR(LOG_TAG, "BT SCO output device is not ready");
            status = -EAGAIN;
            goto exit;
        }
    }

    /* A2DP/BLE device is not ready */
    if (rm->isDeviceAvailable(mDevices, PAL_DEVICE_OUT_BLUETOOTH_A2DP) ||
        rm->isDeviceAvailable(mDevices, PAL_DEVICE_OUT_BLUETOOTH_BLE) ||
        rm->isDeviceAvailable(mDevices, PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST)) {
        if (rm->isDeviceAvailable(mDevices, PAL_DEVICE_OUT_BLUETOOTH_A2DP)) {
            dattr.id = PAL_DEVICE_OUT_BLUETOOTH_A2DP;
        } else if (rm->isDeviceAvailable(mDevices, PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST)){
            dattr.id = PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST;
        } else {
            dattr.id = PAL_DEVICE_OUT_BLUETOOTH_BLE;
        }
        dev = Device::getInstance(&dattr, rm);
        if (!dev) {
            status = -ENODEV;
            PAL_ERR(LOG_TAG, "failed to get a2dp/ble device object");
            goto exit;
        }
        dev->getDeviceParameter(PAL_PARAM_ID_BT_A2DP_SUSPENDED,
                        (void **)&param_bt_a2dp);
        if (param_bt_a2dp->a2dp_suspended == false) {
            PAL_DBG(LOG_TAG, "BT A2DP/BLE output device is good to go");
            goto exit;
        }

        if (rm->isDeviceAvailable(mDevices, PAL_DEVICE_OUT_SPEAKER)) {
            // If it's a2dp + speaker combo device, route to speaker.
            PAL_INFO(LOG_TAG, "BT A2DP/BLE output device is not ready, route to speaker");

            /* In combo use case, if ringtone routed to a2dp + spkr and at that time a2dp/ble
             * device is in suspended state, so during resume ringtone won't be able to route
             * to BLE device. In that case, add both speaker and a2dp/ble into suspended devices
             * list so that a2dp/ble will be restored during a2dpResume without removing speaker
             * from stream
             */
            s->suspendedOutDevIds.clear();
            s->suspendedOutDevIds.push_back(PAL_DEVICE_OUT_SPEAKER);
            s->suspendedOutDevIds.push_back(dattr.id);

            for (auto iter = mDevices.begin(); iter != mDevices.end();) {
                if ((*iter)->getSndDeviceId() == PAL_DEVICE_OUT_SPEAKER) {
                    iter++;
                    continue;
                }

                rm->lockGraph();
                status = session->disconnectSessionDevice(s, sAttr.type, (*iter));
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "disconnectSessionDevice failed:%d", status);
                    rm->unlockGraph();
                    goto exit;
                }

                status = (*iter)->close();
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "device close failed with status %d", status);
                    rm->unlockGraph();
                    goto exit;
                }
                iter = mDevices.erase(iter);
                s->removemDevice((*iter)->getSndDeviceId());
                rm->unlockGraph();
            }
        } else {
            // For non-combo device, mute the stream and route to speaker or handset
            PAL_INFO(LOG_TAG, "BT A2DP/BLE output device is not ready");

            // Mark the suspendedOutDevIds state early - As a2dpResume may happen during this time.
            a2dpSuspend = true;
            s->suspendedOutDevIds.clear();
            s->suspendedOutDevIds.push_back(dattr.id);

            rm->lockGraph();
            for (int i = 0; i < mDevices.size(); i++) {
                status = session->disconnectSessionDevice(s, sAttr.type, mDevices[i]);
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "disconnectSessionDevice failed:%d", status);
                    rm->unlockGraph();
                    goto exit;
                }
                /* Special handling for aaudio usecase on A2DP/BLE.
                * A2DP/BLE device starts even when stream is not in START state,
                * hence stop A2DP/BLE device to match device start&stop count.
                */
                if (((mDevices[i]->getSndDeviceId() == PAL_DEVICE_OUT_BLUETOOTH_A2DP) ||
                    (mDevices[i]->getSndDeviceId() == PAL_DEVICE_OUT_BLUETOOTH_BLE)) && s->isMMap) {
                    status = mDevices[i]->stop();
                    if (0 != status) {
                        PAL_ERR(LOG_TAG, "BT A2DP/BLE device stop failed with status %d", status);
                        }
                }
                status = mDevices[i]->close();
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "device close failed with status %d", status);
                    rm->unlockGraph();
                    goto exit;
                }
            }
            mDevices.clear();
            rm->unlockGraph();
            s->clearOutPalDevices(s);

            /* Check whether there's active stream associated with handset or speaker
             * - Device selected to switch by default is speaker.
             * - Check handset as well if no stream on speaker.
             */
            // NOTE: lockGraph is not intended for speaker or handset device
            spkrDattr.id = PAL_DEVICE_OUT_SPEAKER;
            spkrDev = Device::getInstance(&spkrDattr , rm);
            handsetDattr.id = PAL_DEVICE_OUT_HANDSET;
            handsetDev = Device::getInstance(&handsetDattr , rm);
            if (!spkrDev || !handsetDev) {
                status = -ENODEV;
                PAL_ERR(LOG_TAG, "Failed to get speaker or handset instance");
                goto exit;
            }

            dattr.id = spkrDattr.id;
            dev = spkrDev;

            rm->getActiveStream_l(activeStreams, spkrDev);
            if (activeStreams.empty()) {
                rm->getActiveStream_l(activeStreams, handsetDev);
                if (!activeStreams.empty()) {
                    // active streams found on handset
                    dattr.id = PAL_DEVICE_OUT_HANDSET;
                    dev = handsetDev;
                } else {
                    // no active stream found on both speaker and handset, get the deafult
                    pal_device_info devInfo;
                    memset(&devInfo, 0, sizeof(pal_device_info));
                    status = rm->getDeviceConfig(&dattr, NULL);
                    if (!status) {
                        // get the default device info and update snd name
                        rm->getDeviceInfo(dattr.id, (pal_stream_type_t)0,
                                dattr.custom_config.custom_key, &devInfo);
                        rm->updateSndName(dattr.id, devInfo.sndDevName);
                    }
                    dev->setDeviceAttributes(dattr);
                }
            }

            PAL_INFO(LOG_TAG, "mute stream and route to device %d", dattr.id);

            status = dev->open();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "device open failed with status %d", status);
                goto exit;
            }

            mDevices.push_back(dev);
            s->addmDevice(&dattr);
            rm->lockGraph();
            status = session->setupSessionDevice(s, sAttr.type, dev);
            rm->unlockGraph();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "setupSessionDevice failed:%d", status);
                dev->close();
                mDevices.pop_back();
                goto exit;
            }

            /* Special handling for aaudio usecase on Speaker
             * Speaker device start needs to be called before graph_open
             * to start VI feedback graph and send SP payload to AGM.
             */
            if (dev->getSndDeviceId() == PAL_DEVICE_OUT_SPEAKER && s->isMMap) {
                status = dev->start();
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "Speaker device start failed with status %d", status);
                    dev->close();
                    mDevices.pop_back();
                    s->removeLastmDevice();
                    goto exit;
                }
            }
            status = session->connectSessionDevice(s, sAttr.type, dev);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "connectSessionDevice failed:%d", status);
                dev->close();
                mDevices.pop_back();
                s->removeLastmDevice();
                goto exit;
            }
            dev->getDeviceAttributes(&dattr, s);
            s->addPalDevice(s, &dattr);
        }
    }

exit:
    return status;
}

void handleA2dpBleConcurrency(std::shared_ptr<Device> *inDev,
        struct pal_device *inDevAttr, struct pal_device &dummyDevAttr,
        std::vector <std::tuple<Stream *, uint32_t>> &streamDevDisconnect,
        std::vector <std::tuple<Stream *, struct pal_device *>> &streamDevConnect)
{
    struct pal_device devAttr = {};
    std::shared_ptr<Device> dev = nullptr;
    std::vector <Stream *> streams;
    std::vector <Stream *>::iterator sIter;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    if (inDevAttr->id == PAL_DEVICE_IN_BLUETOOTH_BLE) {
        devAttr.id = PAL_DEVICE_OUT_BLUETOOTH_A2DP;
        dev = Device::getInstance(&devAttr, rm);
        if (!dev) {
            PAL_ERR(LOG_TAG, "getting a2dp/ble device instance failed");
            return;
        }
        rm->getActiveStream_l(streams, dev);
        if (streams.size() == 0) {
            return;
        }
        dummyDevAttr.id = PAL_DEVICE_OUT_DUMMY;
        if (rm->getDeviceConfig(&dummyDevAttr, NULL)) {
            PAL_ERR(LOG_TAG, "rm->getDeviceConfig failed for out_dummy device");
            return;
        }
        for (sIter = streams.begin(); sIter != streams.end(); sIter++) {
            streamDevDisconnect.push_back({(*sIter), PAL_DEVICE_OUT_BLUETOOTH_A2DP});
            streamDevConnect.push_back({(*sIter), &dummyDevAttr});
        }
    } else if (inDevAttr->id == PAL_DEVICE_OUT_BLUETOOTH_A2DP) {
        devAttr.id = PAL_DEVICE_IN_BLUETOOTH_BLE;
        dev = Device::getInstance(&devAttr, rm);
        rm->getActiveStream_l(streams, dev);
        if (streams.size() > 0) {
            inDevAttr->id = PAL_DEVICE_OUT_DUMMY;
            if (rm->getDeviceConfig(inDevAttr, NULL)) {
                PAL_ERR(LOG_TAG, "rm->getDeviceConfig failed for out_dummy device");
                inDevAttr->id = PAL_DEVICE_OUT_BLUETOOTH_A2DP;
                return;
            }
            *inDev = Device::getInstance(inDevAttr , rm);
        }
    }
}

int handleBTDeviceConnectionChange(pal_param_device_connection_t connection_state,
                                    std::vector <pal_device_id_t> &avail_devices_)
{
    int status = 0;
    pal_device_id_t device_id = connection_state.id;
    bool is_connected = connection_state.connection_state;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    bool device_available = rm->isDeviceAvailable(device_id);
    struct pal_device dAttr;
    struct pal_device conn_device;
    pal_address_type_t deviceAddress = connection_state.device.addressV1;
    std::shared_ptr<Device> dev = nullptr;
    struct pal_device_info devinfo = {};
    int32_t scoCount = is_connected ? 1 : -1;
    bool removeScoDevice = false;

    if (rm->isBtScoDevice(device_id)) {
        PAL_DBG(LOG_TAG, "Enter: scoOutConnectCount=%d, scoInConnectCount=%d",
                                        scoOutConnectCount, scoInConnectCount);
        if (device_id == PAL_DEVICE_OUT_BLUETOOTH_SCO) {
            scoOutConnectCount += scoCount;
            removeScoDevice = !scoOutConnectCount;
        } else {
            scoInConnectCount += scoCount;
            removeScoDevice = !scoInConnectCount;
        }
    }
    if (device_id == PAL_DEVICE_OUT_BLUETOOTH_A2DP ||
        device_id == PAL_DEVICE_IN_BLUETOOTH_A2DP ||
        device_id == PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST ||
        device_id == PAL_DEVICE_OUT_BLUETOOTH_BLE ||
        device_id == PAL_DEVICE_IN_BLUETOOTH_BLE ||
        rm->isBtScoDevice(device_id)) {
        dAttr.id = device_id;
        dAttr.addressV1 = deviceAddress;
        /* Stream type is irrelevant here as we need device num channels
           which is independent of stype for BT devices */
        status = rm->getDeviceConfig(&dAttr, NULL);
        if (status) {
            PAL_ERR(LOG_TAG, "Device config not overwritten %d", status);
            goto err;
        }
        dev = Device::getInstance(&dAttr, rm);
        if (!dev) {
            PAL_ERR(LOG_TAG, "Device creation failed");
            throw std::runtime_error("failed to create device object");
            status = -EIO;
            goto err;
        }
    }

    PAL_DBG(LOG_TAG, "Enter");
    memset(&conn_device, 0, sizeof(struct pal_device));
    if (is_connected && !device_available) {
        if (!dev) {
            dAttr.id = device_id;
            dAttr.addressV1 = deviceAddress;
            dev = Device::getInstance(&dAttr, rm);
            if (!dev)
                PAL_ERR(LOG_TAG, "get dev instance for %d failed", device_id);
        }
        if (dev) {
            PAL_DBG(LOG_TAG, "Mark device %d as available", device_id);
            avail_devices_.push_back(device_id);
        }
    } else if (!is_connected && device_available) {
        if (rm->isValidDevId(device_id)) {
            auto iter =
                std::find(avail_devices_.begin(), avail_devices_.end(),
                            device_id);

            if (iter != avail_devices_.end()) {
                PAL_INFO(LOG_TAG, "found device id 0x%x in avail_device",
                                        device_id);
                conn_device.id = device_id;
                conn_device.addressV1 = deviceAddress;
                dev = Device::getInstance(&conn_device, rm);
                if (!dev) {
                    PAL_ERR(LOG_TAG, "Device getInstance failed");
                    throw std::runtime_error("failed to get device object");
                    status = -EIO;
                    goto err;
                }
                if (rm->isBtScoDevice(device_id) && (removeScoDevice == false))
                    goto exit;

                dev->setDeviceAttributes(conn_device);
                PAL_INFO(LOG_TAG, "device attribute cleared");
                PAL_DBG(LOG_TAG, "Mark device %d as unavailable", device_id);
            }
        }
        auto iter =
            std::find(avail_devices_.begin(), avail_devices_.end(),
                        device_id);
        if (iter != avail_devices_.end())
            avail_devices_.erase(iter);
    }
    else if (!rm->isBtScoDevice(device_id)) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid operation, Device %d, connection state %d, device avalibilty %d",
                device_id, is_connected, device_available);
    }

    goto exit;

err:
    if (status && rm->isBtScoDevice(device_id)) {
        if (device_id == PAL_DEVICE_OUT_BLUETOOTH_SCO)
            scoOutConnectCount -= scoCount;
        else
            scoInConnectCount -= scoCount;
    }

exit:
    if (rm->isBtScoDevice(device_id))
        PAL_DBG(LOG_TAG, "Exit: scoOutConnectCount=%d, scoInConnectCount=%d",
                                        scoOutConnectCount, scoInConnectCount);
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t a2dpReconfig()
{
    int status = 0;
    uint32_t latencyMs = 0, maxLatencyMs = 0;
    std::shared_ptr<Device> a2dpDev = nullptr;
    struct pal_device a2dpDattr;
    std::vector <Stream*> activeA2dpStreams;
    std::vector <Stream*> activeStreams;
    std::vector <Stream*>::iterator sIter;
    struct pal_volume_data* volume = NULL;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    PAL_DBG(LOG_TAG, "enter");
    volume = (struct pal_volume_data*)calloc(1, (sizeof(uint32_t) +
        (sizeof(struct pal_channel_vol_kv) * (0xFFFF))));
    if (!volume) {
        status = -ENOMEM;
        goto exit;
    }

    rm->lockActiveStream();

    a2dpDattr.id = PAL_DEVICE_OUT_BLUETOOTH_A2DP;
    a2dpDev = Device::getInstance(&a2dpDattr, rm);
    if (!a2dpDev) {
        PAL_ERR(LOG_TAG, "Getting a2dp/ble device instance failed");
        rm->unlockActiveStream();
        goto exit;
    }

    status = a2dpDev->getDeviceAttributes(&a2dpDattr);
    if (status) {
        PAL_ERR(LOG_TAG, "Switch DevAttributes Query Failed");
        rm->unlockActiveStream();
        goto exit;
    }

    rm->getActiveStream_l(activeA2dpStreams, a2dpDev);
    if (activeA2dpStreams.size() == 0) {
        PAL_DBG(LOG_TAG, "no active streams found on a2dp device");
        rm->unlockActiveStream();
        goto exit;
    }
    for (sIter = activeA2dpStreams.begin(); sIter != activeA2dpStreams.end(); sIter++) {
        if (((*sIter) != NULL) && rm->isStreamActive(*sIter)) {
            (*sIter)->lockStreamMutex();
            if (!((*sIter)->a2dpMuted)) {
                struct pal_stream_attributes sAttr;
                (*sIter)->getStreamAttributes(&sAttr);
                if (((sAttr.type == PAL_STREAM_COMPRESSED) ||
                    (sAttr.type == PAL_STREAM_PCM_OFFLOAD))) {
                    /* First mute & then pause
                     * This is to ensure DSP has enough ramp down period in volume module.
                     * If pause is issued firstly, then there's no enough data for processing.
                     * As a result, ramp down will not happen and will only occur after resume,
                     * which is perceived as audio leakage.
                     */
                    (*sIter)->mute_l(true);
                    (*sIter)->a2dpMuted = true;
                    // Pause only if the stream is not explicitly paused.
                    // In some scenarios, stream might have already paused prior to a2dpsuspend.
                    if (((*sIter)->isPaused) == false) {
                        (*sIter)->pause_l();
                        (*sIter)->a2dpPaused = true;
                    }
                } else {
                    latencyMs = (*sIter)->getLatency();
                    if (maxLatencyMs < latencyMs)
                        maxLatencyMs = latencyMs;
                    // Mute
                    (*sIter)->mute_l(true);
                    (*sIter)->a2dpMuted = true;
                }
            }
            (*sIter)->unlockStreamMutex();
        }
    }

    rm->unlockActiveStream();

    // wait for stale pcm drained before switching to speaker
    if (maxLatencyMs > 0) {
        // multiplication factor applied to latency when calculating a safe mute delay
        // TODO: It's not needed if latency is accurate.
        const int latencyMuteFactor = 2;
        usleep(maxLatencyMs * 1000 * latencyMuteFactor);
    }

    rm->forceDeviceSwitch(a2dpDev, &a2dpDattr);

    rm->lockActiveStream();
    for (sIter = activeA2dpStreams.begin(); sIter != activeA2dpStreams.end(); sIter++) {
        if (((*sIter) != NULL) && rm->isStreamActive(*sIter)) {
            (*sIter)->lockStreamMutex();
            struct pal_stream_attributes sAttr;
            (*sIter)->getStreamAttributes(&sAttr);
            if (((sAttr.type == PAL_STREAM_COMPRESSED) ||
                (sAttr.type == PAL_STREAM_PCM_OFFLOAD)) &&
                (!(*sIter)->isActive())) {
                /* Resume only when it was paused during a2dpSuspend.
                 * This is to avoid resuming during regular pause.
                 */
                if (((*sIter)->a2dpPaused) == true) {
                    (*sIter)->resume_l();
                    (*sIter)->a2dpPaused = false;
                }
            }
            status = (*sIter)->getVolumeData(volume);
            if (status) {
                PAL_ERR(LOG_TAG, "getVolumeData failed %d", status);
                (*sIter)->unlockStreamMutex();
                continue;
            }
            /* set a2dpMuted to false so that volume can be applied
             * volume gets cached if a2dpMuted is set to true
             */
            (*sIter)->a2dpMuted = false;
            status = (*sIter)->setVolume(volume);
            if (status) {
                PAL_ERR(LOG_TAG, "setVolume failed %d", status);
                (*sIter)->a2dpMuted = true;
                (*sIter)->unlockStreamMutex();
                continue;
            }
            // set a2dpMuted to true in case unmute failed
            if ((*sIter)->mute_l(false))
                (*sIter)->a2dpMuted = true;
            (*sIter)->unlockStreamMutex();
        }
    }
    rm->unlockActiveStream();

exit:
    PAL_DBG(LOG_TAG, "exit status: %d", status);
    if (volume) {
        free(volume);
    }
    return status;
}

int32_t a2dpSuspendToDummy(pal_device_id_t dev_id)
{
    int status = 0;
    std::shared_ptr<Device> a2dpDev = nullptr;
    struct pal_device a2dpDattr = {};
    std::shared_ptr<Device> switchDev = nullptr;
    struct pal_device switchDevDattr = {};
    std::vector <Stream*> activeA2dpStreams;
    std::vector <Stream*>::iterator sIter;
    std::vector <std::shared_ptr<Device>> associatedDevices;
    std::vector <std::tuple<Stream*, uint32_t>> streamDevDisconnect;
    std::vector <std::tuple<Stream*, struct pal_device *>> streamDevConnect;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    PAL_DBG(LOG_TAG, "enter");

    a2dpDattr.id = dev_id;
    a2dpDev = Device::getInstance(&a2dpDattr, rm);
    if (!a2dpDev) {
        PAL_ERR(LOG_TAG, "getting a2dp/ble device instance failed");
        goto exit;
    }

    switchDevDattr.id = PAL_DEVICE_OUT_DUMMY;
    status = rm->getDeviceConfig(&switchDevDattr, NULL);
    if (status) {
        PAL_ERR(LOG_TAG, "rm->getDeviceConfig failed for out_dummy device");
        goto exit;
    }

    rm->lockActiveStream();
    rm->getActiveStream_l(activeA2dpStreams, a2dpDev);
    if (activeA2dpStreams.size() == 0) {
        PAL_DBG(LOG_TAG, "no active streams found");
        rm->unlockActiveStream();
        goto exit;
    }

    for (sIter = activeA2dpStreams.begin(); sIter != activeA2dpStreams.end(); sIter++) {
        if (((*sIter) != NULL) && rm->isStreamActive(*sIter)) {
            (*sIter)->lockStreamMutex();
            associatedDevices.clear();
            status = (*sIter)->getAssociatedOutDevices(associatedDevices);
            if ((0 != status) ||
                !(rm->isDeviceAvailable(associatedDevices, a2dpDattr.id))) {
                PAL_ERR(LOG_TAG, "error: stream %pK is not associated with a2dp/ble device", *sIter);
                (*sIter)->unlockStreamMutex();
                continue;
            }
            (*sIter)->suspendedOutDevIds.clear();
            (*sIter)->suspendedOutDevIds.push_back(a2dpDattr.id);
            streamDevDisconnect.push_back({*sIter, a2dpDattr.id});
            if (associatedDevices.size() == 1) {
                // perform mute for non combo use-case, if it is not already muted
                if (!((*sIter)->a2dpMuted) && !((*sIter)->mute_l(true))) {
                    (*sIter)->a2dpMuted = true;
                }
                streamDevConnect.push_back({*sIter, &switchDevDattr});
            } else {
                (*sIter)->suspendedOutDevIds.push_back(switchDevDattr.id);
            }
            (*sIter)->unlockStreamMutex();
        }
    }
    rm->unlockActiveStream();

    PAL_DBG(LOG_TAG, "switching a2dp/ble stream to out_dummy device");
    status = rm->streamDevSwitch(streamDevDisconnect, streamDevConnect);

    if (status) {
        PAL_ERR(LOG_TAG, " rm->streamDevSwitch failed %d", status);
        goto exit;
    }

    rm->lockActiveStream();
    for (sIter = activeA2dpStreams.begin(); sIter != activeA2dpStreams.end(); sIter++) {
        if (((*sIter) != NULL) && rm->isStreamActive(*sIter)) {
            (*sIter)->lockStreamMutex();
            (*sIter)->removePalDevice(*sIter, a2dpDattr.id);
            if ((*sIter)->suspendedOutDevIds.size() == 1) {
                (*sIter)->addPalDevice(*sIter, &switchDevDattr);
            }
            (*sIter)->unlockStreamMutex();
        }
    }
    rm->unlockActiveStream();

exit:
    PAL_DBG(LOG_TAG, "exit status: %d", status);
    return status;
}

int32_t a2dpResumeFromDummy(pal_device_id_t dev_id)
{
    int status = 0;
    std::shared_ptr<Device> activeDev = nullptr;
    struct pal_device activeDattr = {};
    struct pal_device a2dpDattr = {};
    struct pal_volume_data *volume = NULL;
    std::vector <Stream*>::iterator sIter;
    std::vector <Stream*> activeStreams;
    std::vector <Stream*> orphanStreams;
    std::vector <Stream*> retryStreams;
    std::vector <Stream*> restoredStreams;
    std::vector <std::tuple<Stream*, uint32_t>> streamDevDisconnect;
    std::vector <std::tuple<Stream*, struct pal_device *>> streamDevConnect;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    PAL_DBG(LOG_TAG, "enter");

    volume = (struct pal_volume_data *)calloc(1, (sizeof(uint32_t) +
                     (sizeof(struct pal_channel_vol_kv) * (0xFFFF))));
    if (!volume) {
        status = -ENOMEM;
        goto exit;
    }

    activeDattr.id = PAL_DEVICE_OUT_DUMMY;
    activeDev = Device::getInstance(&activeDattr, rm);
    if (!activeDev) {
        PAL_ERR(LOG_TAG, "getting out_dummy device instance failed");
        goto exit;
    }

    a2dpDattr.id = dev_id;
    status = rm->getDeviceConfig(&a2dpDattr, NULL);
    if (status) {
        PAL_ERR(LOG_TAG, "rm->getDeviceConfig failed for a2dp/ble device");
        goto exit;
    }

    rm->lockActiveStream();
    rm->getActiveStream_l(activeStreams, activeDev);
    rm->getOrphanStream_l(orphanStreams, retryStreams);
    if (activeStreams.empty() && orphanStreams.empty() && retryStreams.empty()) {
        PAL_DBG(LOG_TAG, "no active streams found");
        rm->unlockActiveStream();
        goto exit;
    }

    /* Check all active streams associated with out_dummy device.
     * Disconnect out_dummy device and connect BT device for active streams.
     */
    for (sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++) {
        (*sIter)->lockStreamMutex();
        if (std::find((*sIter)->suspendedOutDevIds.begin(),
                (*sIter)->suspendedOutDevIds.end(), a2dpDattr.id)
                != (*sIter)->suspendedOutDevIds.end()) {
            restoredStreams.push_back(*sIter);
            streamDevDisconnect.push_back({*sIter, activeDattr.id});
            streamDevConnect.push_back({*sIter, &a2dpDattr});
        }
        (*sIter)->unlockStreamMutex();
    }

    // Retry all orphan streams which failed to restore previously.
    for (sIter = orphanStreams.begin(); sIter != orphanStreams.end(); sIter++) {
        (*sIter)->lockStreamMutex();
        if (std::find((*sIter)->suspendedOutDevIds.begin(),
                (*sIter)->suspendedOutDevIds.end(), a2dpDattr.id)
                != (*sIter)->suspendedOutDevIds.end()) {
            restoredStreams.push_back(*sIter);
            streamDevConnect.push_back({*sIter, &a2dpDattr});
        }
        (*sIter)->unlockStreamMutex();
    }

    /* Restore combo streams as well as streams which failed to switch
     * to desired device previously.
     */
    for (sIter = retryStreams.begin(); sIter != retryStreams.end(); sIter++) {
        (*sIter)->lockStreamMutex();
        if (std::find((*sIter)->suspendedOutDevIds.begin(),
                (*sIter)->suspendedOutDevIds.end(), a2dpDattr.id)
                != (*sIter)->suspendedOutDevIds.end()) {
            std::vector<std::shared_ptr<Device>> devices;
            (*sIter)->getAssociatedOutDevices(devices);
            if ((devices.size() > 0) &&
                ((*sIter)->suspendedOutDevIds.size() == 1 /* non combo */)) {
                for (auto device: devices) {
                    streamDevDisconnect.push_back({*sIter, device->getSndDeviceId()});
                }
            }
            restoredStreams.push_back(*sIter);
            streamDevConnect.push_back({*sIter, &a2dpDattr});
        } else if (std::find((*sIter)->suspendedOutDevIds.begin(),
                (*sIter)->suspendedOutDevIds.end(), PAL_DEVICE_OUT_BLUETOOTH_SCO)
                != (*sIter)->suspendedOutDevIds.end()) {
            std::vector<std::shared_ptr<Device>> devices;
            (*sIter)->getAssociatedOutDevices(devices);
            if (devices.size() > 0) {
                for (auto device: devices) {
                    if (device->getSndDeviceId() == PAL_DEVICE_OUT_BLUETOOTH_SCO) {
                        streamDevDisconnect.push_back({(*sIter), PAL_DEVICE_OUT_BLUETOOTH_SCO});
                        break;
                    }
                }
            }
            restoredStreams.push_back((*sIter));
            streamDevConnect.push_back({(*sIter), &a2dpDattr});
        }
        (*sIter)->unlockStreamMutex();
    }

    if (restoredStreams.empty()) {
        PAL_DBG(LOG_TAG, "no streams to be restored");
        rm->unlockActiveStream();
        goto exit;
    }
    SortAndUnique(restoredStreams);
    rm->unlockActiveStream();

    /* In case of SSR down event if a2dpSuspended = false sets, since sound card state is offline
     * rm->streamDevSwitch() operation will be skipped. Due to this mDevices will remain as speaker
     * and when SSR is up, active streams will route to the speaker only.
     * Thus in this corner scenario, update the mDevices as BT A2DP so that when SSR is up
     * active streams will be routed to BT properly.
     */
    if (PAL_CARD_STATUS_DOWN(rm->getSoundCardState())) {
        PAL_ERR(LOG_TAG, "Sound card offline");
        rm->lockActiveStream();
        for (sIter = restoredStreams.begin(); sIter != restoredStreams.end(); sIter++) {
            if (((*sIter) != NULL) && rm->isStreamActive(*sIter)) {
                (*sIter)->lockStreamMutex();
                if (std::find((*sIter)->suspendedOutDevIds.begin(),
                        (*sIter)->suspendedOutDevIds.end(), a2dpDattr.id)
                        != (*sIter)->suspendedOutDevIds.end()) {
                    if ((*sIter)->suspendedOutDevIds.size() == 1 /* non-combo */) {
                        (*sIter)->removemDevice(activeDattr.id);
                        (*sIter)->removePalDevice(*sIter, activeDattr.id);
                    }
                } else if (std::find((*sIter)->suspendedOutDevIds.begin(),
                        (*sIter)->suspendedOutDevIds.end(), PAL_DEVICE_OUT_BLUETOOTH_SCO)
                        != (*sIter)->suspendedOutDevIds.end()) {
                    (*sIter)->removemDevice(PAL_DEVICE_OUT_BLUETOOTH_SCO);
                    (*sIter)->removePalDevice(*sIter, PAL_DEVICE_OUT_BLUETOOTH_SCO);
                }
                (*sIter)->addmDevice(&a2dpDattr);
                (*sIter)->addPalDevice(*sIter, &a2dpDattr);
                (*sIter)->a2dpMuted = false;
                (*sIter)->unlockStreamMutex();
            }
        }
        rm->unlockActiveStream();
        goto exit;
    } else {
        PAL_DBG(LOG_TAG, "restoring a2dp/ble stream");
        status = rm->streamDevSwitch(streamDevDisconnect, streamDevConnect);
        if (status) {
            PAL_ERR(LOG_TAG, "streamDevSwitch failed %d", status);
            goto exit;
        }
    }

    rm->lockActiveStream();
    for (sIter = restoredStreams.begin(); sIter != restoredStreams.end(); sIter++) {
        if (((*sIter) != NULL) && rm->isStreamActive(*sIter)) {
            (*sIter)->lockStreamMutex();
            // update PAL devices for the restored streams
            if (std::find((*sIter)->suspendedOutDevIds.begin(),
                    (*sIter)->suspendedOutDevIds.end(), a2dpDattr.id)
                    != (*sIter)->suspendedOutDevIds.end()) {
                if ((*sIter)->suspendedOutDevIds.size() == 1 /* non-combo */)
                    (*sIter)->removePalDevice(*sIter, activeDattr.id);
            } else if (std::find((*sIter)->suspendedOutDevIds.begin(),
                    (*sIter)->suspendedOutDevIds.end(), PAL_DEVICE_OUT_BLUETOOTH_SCO)
                    != (*sIter)->suspendedOutDevIds.end()) {
                (*sIter)->removePalDevice(*sIter, PAL_DEVICE_OUT_BLUETOOTH_SCO);
            }
            (*sIter)->addPalDevice(*sIter, &a2dpDattr);
            (*sIter)->suspendedOutDevIds.clear();
            // unmute the streams which were muted during a2dpSuspend
            if ((*sIter)->a2dpMuted) {
                status = (*sIter)->getVolumeData(volume);
                if (status) {
                    PAL_ERR(LOG_TAG, "getVolumeData failed %d", status);
                    (*sIter)->unlockStreamMutex();
                    continue;
                }
                /* set a2dpMuted to false so that volume can be applied
                 * volume gets cached if a2dpMuted is set to true
                 */
                (*sIter)->a2dpMuted = false;
                status = (*sIter)->setVolume(volume);
                if (status) {
                    PAL_ERR(LOG_TAG, "setVolume failed %d", status);
                    (*sIter)->a2dpMuted = true;
                    (*sIter)->unlockStreamMutex();
                    continue;
                }
                // set a2dpMuted to true in case unmute failed
                if ((*sIter)->mute_l(false))
                    (*sIter)->a2dpMuted = true;
            }
            (*sIter)->unlockStreamMutex();
        }
    }
    rm->unlockActiveStream();

exit:
    PAL_DBG(LOG_TAG, "exit status: %d", status);
    if (volume) {
        free(volume);
    }
    return status;
}

int32_t a2dpCaptureSuspendToDummy(pal_device_id_t dev_id)
{
    int status = 0;
    std::shared_ptr<Device> a2dpDev = nullptr;
    struct pal_device a2dpDattr = {};
    struct pal_device switchDevDattr = {};
    std::vector <Stream*> activeA2dpStreams;
    std::shared_ptr<Device> switchDev = nullptr;
    std::vector <Stream*>::iterator sIter;
    std::vector <std::tuple<Stream*, uint32_t>> streamDevDisconnect;
    std::vector <std::tuple<Stream*, struct pal_device *>> streamDevConnect;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    PAL_DBG(LOG_TAG, "enter");

    a2dpDattr.id = dev_id;
    a2dpDev = Device::getInstance(&a2dpDattr, rm);
    if (!a2dpDev) {
        PAL_ERR(LOG_TAG, "getting a2dp/ble device instance failed");
        goto exit;
    }

    switchDevDattr.id = PAL_DEVICE_IN_DUMMY;
    status = rm->getDeviceConfig(&switchDevDattr, NULL);
    if (status) {
        PAL_ERR(LOG_TAG, "rm->getDeviceConfig failed for in_dummy device");
        goto exit;
    }

    rm->lockActiveStream();
    rm->getActiveStream_l(activeA2dpStreams, a2dpDev);
    if (activeA2dpStreams.size() == 0) {
        PAL_DBG(LOG_TAG, "no active streams found");
        rm->unlockActiveStream();
        goto exit;
    }

    for (sIter = activeA2dpStreams.begin(); sIter != activeA2dpStreams.end(); sIter++) {
        if (((*sIter) != NULL) && rm->isStreamActive(*sIter)) {
            (*sIter)->lockStreamMutex();
            if (!((*sIter)->a2dpMuted) && !((*sIter)->mute_l(true))) {
                (*sIter)->a2dpMuted = true;
            }
            (*sIter)->suspendedInDevIds.clear();
            (*sIter)->suspendedInDevIds.push_back(a2dpDattr.id);
            streamDevDisconnect.push_back({*sIter, a2dpDattr.id});
            streamDevConnect.push_back({*sIter, &switchDevDattr});
            (*sIter)->unlockStreamMutex();
        }
    }
    rm->unlockActiveStream();

    PAL_DBG(LOG_TAG, "switching capture streams to in_dummy device");
    status = rm->streamDevSwitch(streamDevDisconnect, streamDevConnect);
    if (status) {
        PAL_ERR(LOG_TAG, " rm->streamDevSwitch failed %d", status);
        goto exit;
    }

    rm->lockActiveStream();
    for (sIter = activeA2dpStreams.begin(); sIter != activeA2dpStreams.end(); sIter++) {
        if (((*sIter) != NULL) && rm->isStreamActive(*sIter)) {
            (*sIter)->lockStreamMutex();
            (*sIter)->removePalDevice(*sIter, a2dpDattr.id);
            (*sIter)->addPalDevice(*sIter, &switchDevDattr);
            (*sIter)->unlockStreamMutex();
        }
    }
    rm->unlockActiveStream();

exit:
    PAL_DBG(LOG_TAG, "exit status: %d", status);
    return status;
}

int32_t a2dpCaptureResumeFromDummy(pal_device_id_t dev_id)
{
    int status = 0;
    std::shared_ptr<Device> activeDev = nullptr;
    struct pal_device activeDattr = {};
    struct pal_device a2dpDattr = {};
    std::vector <Stream*>::iterator sIter;
    std::vector <Stream*> activeStreams;
    std::vector <Stream*> orphanStreams;
    std::vector <Stream*> retryStreams;
    std::vector <Stream*> restoredStreams;
    std::vector <std::tuple<Stream*, uint32_t>> streamDevDisconnect;
    std::vector <std::tuple<Stream*, struct pal_device*>> streamDevConnect;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    PAL_DBG(LOG_TAG, "enter");

    activeDattr.id = PAL_DEVICE_IN_DUMMY;
    activeDev = Device::getInstance(&activeDattr, rm);
    if (!activeDev) {
        PAL_ERR(LOG_TAG, "getting out_dummy device instance failed");
        goto exit;
    }

    a2dpDattr.id = dev_id;
    status = rm->getDeviceConfig(&a2dpDattr, NULL);
    if (status) {
        PAL_ERR(LOG_TAG, "rm->getDeviceConfig failed for a2dp/BLE device");
        goto exit;
    }

    rm->lockActiveStream();
    rm->getActiveStream_l(activeStreams, activeDev);
    rm->getOrphanStream_l(orphanStreams, retryStreams);
    if (activeStreams.empty() && orphanStreams.empty() && retryStreams.empty()) {
        PAL_DBG(LOG_TAG, "no active streams found");
        rm->unlockActiveStream();
        goto exit;
    }

    // check all active streams associated with in_dummy device.
    // Istore into stream vector for device switch.
    for (sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++) {
        if (std::find((*sIter)->suspendedInDevIds.begin(),
                (*sIter)->suspendedInDevIds.end(), a2dpDattr.id)
                != (*sIter)->suspendedInDevIds.end()) {
            restoredStreams.push_back((*sIter));
            streamDevDisconnect.push_back({(*sIter), activeDattr.id });
            streamDevConnect.push_back({(*sIter), &a2dpDattr });
        }
    }

    // retry all orphan streams which failed to restore previously.
    for (sIter = orphanStreams.begin(); sIter != orphanStreams.end(); sIter++) {
        if (std::find((*sIter)->suspendedInDevIds.begin(),
                (*sIter)->suspendedInDevIds.end(), a2dpDattr.id)
                != (*sIter)->suspendedInDevIds.end()) {
            restoredStreams.push_back((*sIter));
            streamDevConnect.push_back({(*sIter), &a2dpDattr });
        }
    }

    // retry all streams which failed to switch to desired device previously.
    for (sIter = retryStreams.begin(); sIter != retryStreams.end(); sIter++) {
        (*sIter)->lockStreamMutex();
        if (std::find((*sIter)->suspendedInDevIds.begin(),
            (*sIter)->suspendedInDevIds.end(), a2dpDattr.id)
            != (*sIter)->suspendedInDevIds.end()) {
            std::vector<std::shared_ptr<Device>> devices;
            (*sIter)->getAssociatedInDevices(devices);
            if (devices.size() > 0) {
                for (auto device : devices) {
                    streamDevDisconnect.push_back({ (*sIter), device->getSndDeviceId() });
                }
            }
            restoredStreams.push_back((*sIter));
            streamDevConnect.push_back({ (*sIter), &a2dpDattr });
        }
        (*sIter)->unlockStreamMutex();
    }

    if (restoredStreams.empty()) {
        PAL_DBG(LOG_TAG, "no streams to be restored");
        rm->unlockActiveStream();
        goto exit;
    }
    rm->unlockActiveStream();

    PAL_DBG(LOG_TAG, "restoring a2dp/ble streams");
    status = rm->streamDevSwitch(streamDevDisconnect, streamDevConnect);
    if (status) {
        PAL_ERR(LOG_TAG, "rm->streamDevSwitch failed %d", status);
        goto exit;
    }

    rm->lockActiveStream();
    for (sIter = restoredStreams.begin(); sIter != restoredStreams.end(); sIter++) {
        if (((*sIter) != NULL) && rm->isStreamActive(*sIter)) {
            (*sIter)->lockStreamMutex();
            (*sIter)->suspendedInDevIds.clear();
            (*sIter)->removePalDevice(*sIter, activeDattr.id);
            (*sIter)->addPalDevice(*sIter, &a2dpDattr);
            if ((*sIter)->a2dpMuted) {
                (*sIter)->mute_l(false);
                (*sIter)->a2dpMuted = false;
            }
            (*sIter)->unlockStreamMutex();
        }
    }
    rm->unlockActiveStream();

exit:
    PAL_DBG(LOG_TAG, "exit status: %d", status);
    return status;
}

int32_t a2dpSuspend(pal_device_id_t dev_id)
{
    int status = 0;
    uint32_t latencyMs = 0, maxLatencyMs = 0;
    std::shared_ptr<Device> a2dpDev = nullptr;
    struct pal_device a2dpDattr;
    struct pal_device switchDevDattr;
    std::shared_ptr<Device> spkrDev = nullptr;
    std::shared_ptr<Device> handsetDev = nullptr;
    struct pal_device spkrDattr;
    struct pal_device handsetDattr;
    std::vector <Stream *> activeA2dpStreams;
    std::vector <Stream *> activeStreams;
    std::vector <Stream*>::iterator sIter;
    std::vector <std::shared_ptr<Device>> associatedDevices;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    PAL_DBG(LOG_TAG, "enter");

    a2dpDattr.id = dev_id;
    a2dpDev = Device::getInstance(&a2dpDattr, rm);
    if (!a2dpDev) {
        PAL_ERR(LOG_TAG, "Getting a2dp/ble device instance failed");
        goto exit;
    }

    spkrDattr.id = PAL_DEVICE_OUT_SPEAKER;
    spkrDev = Device::getInstance(&spkrDattr, rm);
    if (!spkrDev) {
        PAL_ERR(LOG_TAG, "Getting speaker device instance failed");
        goto exit;
    }

    rm->lockActiveStream();
    rm->getActiveStream_l(activeA2dpStreams, a2dpDev);
    if (activeA2dpStreams.size() == 0) {
        PAL_DBG(LOG_TAG, "no active streams found");
        rm->unlockActiveStream();
        goto exit;
    }

    /* Check whether there's active stream associated with handset or speaker
     * when a2dp suspend is called.
     * - Device selected to switch by default is speaker.
     * - Check handset as well if no stream on speaker.
     */
    switchDevDattr.id = PAL_DEVICE_OUT_SPEAKER;
    rm->getActiveStream_l(activeStreams, spkrDev);
    if (activeStreams.empty()) {
        // no stream active on speaker, then check handset as well
        handsetDattr.id = PAL_DEVICE_OUT_HANDSET;
        handsetDev = Device::getInstance(&handsetDattr, rm);
        if (handsetDev) {
            rm->getActiveStream_l(activeStreams, handsetDev);
        } else {
            PAL_ERR(LOG_TAG, "Getting handset device instance failed");
            rm->unlockActiveStream();
            goto exit;
        }

        if (activeStreams.size() != 0) {
            // active streams found on handset
            switchDevDattr.id = PAL_DEVICE_OUT_HANDSET;
            status = handsetDev->getDeviceAttributes(&switchDevDattr);
        } else {
            // no active stream found on speaker or handset, get the deafult
            pal_device_info devInfo;
            memset(&devInfo, 0, sizeof(pal_device_info));
            status = rm->getDeviceConfig(&switchDevDattr, NULL);
            if (!status) {
                // get the default device info and update snd name
                rm->getDeviceInfo(switchDevDattr.id, (pal_stream_type_t)0,
                    switchDevDattr.custom_config.custom_key, &devInfo);
                rm->updateSndName(switchDevDattr.id, devInfo.sndDevName);
            }
        }
    } else {
        // activeStreams found on speaker
        status = spkrDev->getDeviceAttributes(&switchDevDattr);
    }
    if (status) {
        PAL_ERR(LOG_TAG, "Switch DevAttributes Query Failed");
        rm->unlockActiveStream();
        goto exit;
    }

    PAL_DBG(LOG_TAG, "selecting active device_id[%d] and muting streams",
        switchDevDattr.id);

    for (sIter = activeA2dpStreams.begin(); sIter != activeA2dpStreams.end(); sIter++) {
        if (((*sIter) != NULL) && rm->isStreamActive(*sIter)) {
            associatedDevices.clear();
            status = (*sIter)->getAssociatedOutDevices(associatedDevices);
            if ((0 != status) ||
                !(rm->isDeviceAvailable(associatedDevices, a2dpDattr.id))) {
                PAL_ERR(LOG_TAG, "Error: stream %pK is not associated with A2DP/BLE device", *sIter);
                continue;
            }
            /* For a2dp + spkr or handset combo use case,
             * add speaker or handset into suspended devices for restore during a2dpResume
             */
            (*sIter)->lockStreamMutex();
            if (rm->isDeviceAvailable(associatedDevices, switchDevDattr.id)) {
                // combo use-case; just remember to keep the non-a2dp devices when restoring.
                PAL_DBG(LOG_TAG, "Stream %pK is on combo device; Dont Pause/Mute", *sIter);
                (*sIter)->suspendedOutDevIds.clear();
                (*sIter)->suspendedOutDevIds.push_back(switchDevDattr.id);
                (*sIter)->suspendedOutDevIds.push_back(a2dpDattr.id);
            } else if (!((*sIter)->a2dpMuted)) {
                // only perform Mute/Pause for non combo use-case only.
                struct pal_stream_attributes sAttr;
                (*sIter)->getStreamAttributes(&sAttr);
                (*sIter)->suspendedOutDevIds.clear();
                (*sIter)->suspendedOutDevIds.push_back(a2dpDattr.id);
                if (((sAttr.type == PAL_STREAM_COMPRESSED) ||
                     (sAttr.type == PAL_STREAM_PCM_OFFLOAD))) {
                    /* First mute & th is to ensure DSP has enough ramp down period in volume module.
                     * If pause is issued firstly, then there's no enough data for processing.
                     * As a result, ramp down will not happen and will only occur after resume,
                     * which is perceived as audio leakage.
                     */
                    if (!(*sIter)->mute_l(true))
                        (*sIter)->a2dpMuted = true;
                    // Pause only if the stream is not explicitly paused.
                    // In some scenarios, stream might have already paused prior to a2dpsuspend.
                    if (((*sIter)->isPaused) == false) {
                        if (!(*sIter)->pause_l())
                            (*sIter)->a2dpPaused = true;
                    }
                } else {
                    latencyMs = (*sIter)->getLatency();
                    if (maxLatencyMs < latencyMs)
                        maxLatencyMs = latencyMs;
                    // Mute
                    if (!(*sIter)->mute_l(true))
                        (*sIter)->a2dpMuted = true;
                }
            }
            (*sIter)->unlockStreamMutex();
        }
    }

    rm->unlockActiveStream();

    // wait for stale pcm drained before switching to speaker
    if (maxLatencyMs > 0) {
        // multiplication factor applied to latency when calculating a safe mute delay
        // TODO: It's not needed if latency is accurate.
        const int latencyMuteFactor = 2;
        usleep(maxLatencyMs * 1000 * latencyMuteFactor);
    }

    rm->forceDeviceSwitch(a2dpDev, &switchDevDattr, activeA2dpStreams);

    rm->lockActiveStream();
    for (sIter = activeA2dpStreams.begin(); sIter != activeA2dpStreams.end(); sIter++) {
        if (((*sIter) != NULL) && rm->isStreamActive(*sIter)) {
            (*sIter)->lockStreamMutex();
            struct pal_stream_attributes sAttr;
            (*sIter)->getStreamAttributes(&sAttr);
            if (((sAttr.type == PAL_STREAM_COMPRESSED) ||
                (sAttr.type == PAL_STREAM_PCM_OFFLOAD)) &&
                (!(*sIter)->isActive())) {
                /* Resume only when it was paused during a2dpSuspend.
                 * This is to avoid resuming during regular pause.
                 */
                if (((*sIter)->a2dpPaused) == true) {
                    if (!(*sIter)->resume_l())
                        (*sIter)->a2dpPaused = false;
                }
            }
            (*sIter)->unlockStreamMutex();
        }
    }
    rm->unlockActiveStream();

exit:
    PAL_DBG(LOG_TAG, "exit status: %d", status);
    return status;
}

int32_t a2dpResume(pal_device_id_t dev_id)
{
    int status = 0;
    std::shared_ptr<Device> activeDev = nullptr;
    struct pal_device activeDattr;
    struct pal_device a2dpDattr;
    struct pal_device_info devinfo = {};
    struct pal_volume_data *volume = NULL;
    std::vector <Stream*>::iterator sIter;
    std::vector <Stream *> activeStreams;
    std::vector <Stream *> orphanStreams;
    std::vector <Stream *> retryStreams;
    std::vector <Stream *> restoredStreams;
    std::vector <std::tuple<Stream *, uint32_t>> streamDevDisconnect;
    std::vector <std::tuple<Stream *, struct pal_device *>> streamDevConnect;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    PAL_DBG(LOG_TAG, "enter");

    volume = (struct pal_volume_data *)calloc(1, (sizeof(uint32_t) +
                     (sizeof(struct pal_channel_vol_kv) * (0xFFFF))));
    if (!volume) {
        status = -ENOMEM;
        goto exit;
    }

    activeDattr.id = PAL_DEVICE_OUT_SPEAKER;
    activeDev = Device::getInstance(&activeDattr, rm);
    a2dpDattr.id = dev_id;

    status = rm->getDeviceConfig(&a2dpDattr, NULL);
    if (status) {
        goto exit;
    }

    rm->lockActiveStream();
    rm->getActiveStream_l(activeStreams, activeDev);
    /* No-Streams active on Speaker - possibly streams are
     * associated handset device (due to voip/voice sco ended) and
     * device switch did not happen for all the streams
     */
    if (activeStreams.empty()) {
        // Hence try to check handset device as well.
        activeDattr.id = PAL_DEVICE_OUT_HANDSET;
        activeDev = Device::getInstance(&activeDattr, rm);
        rm->getActiveStream_l(activeStreams, activeDev);
    }

    rm->getOrphanStream_l(orphanStreams, retryStreams);
    if (activeStreams.empty() && orphanStreams.empty() && retryStreams.empty()) {
        PAL_DBG(LOG_TAG, "no active streams found");
        rm->unlockActiveStream();
        goto exit;
    }

    /* Check all active streams associated with speaker or handset.
     * If actual device is a2dp only, store into stream vector for device switch.
     * If actual device is combo(a2dp + spkr/handset), restore to combo.
     * That is to connect a2dp and do not disconnect from current associated device.
     */
    for (sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++) {
        (*sIter)->lockStreamMutex();
        if (std::find((*sIter)->suspendedOutDevIds.begin(),
                (*sIter)->suspendedOutDevIds.end(), a2dpDattr.id)
                != (*sIter)->suspendedOutDevIds.end()) {
            restoredStreams.push_back((*sIter));
            if ((*sIter)->suspendedOutDevIds.size() == 1 /* none combo */) {
                streamDevDisconnect.push_back({(*sIter), activeDattr.id});
            }
            streamDevConnect.push_back({(*sIter), &a2dpDattr});
        }
        (*sIter)->unlockStreamMutex();
    }

    // retry all orphan streams which failed to restore previously.
    for (sIter = orphanStreams.begin(); sIter != orphanStreams.end(); sIter++) {
        (*sIter)->lockStreamMutex();
        if (std::find((*sIter)->suspendedOutDevIds.begin(),
                (*sIter)->suspendedOutDevIds.end(), a2dpDattr.id)
                != (*sIter)->suspendedOutDevIds.end()) {
            restoredStreams.push_back((*sIter));
            streamDevConnect.push_back({(*sIter), &a2dpDattr});
        }
        (*sIter)->unlockStreamMutex();
    }

    // retry all streams which failed to switch to desired device previously.
    for (sIter = retryStreams.begin(); sIter != retryStreams.end(); sIter++) {
        (*sIter)->lockStreamMutex();
        if (std::find((*sIter)->suspendedOutDevIds.begin(),
                (*sIter)->suspendedOutDevIds.end(), a2dpDattr.id)
                != (*sIter)->suspendedOutDevIds.end()) {
            std::vector<std::shared_ptr<Device>> devices;
            (*sIter)->getAssociatedOutDevices(devices);
            if ((devices.size() > 0) &&
                    ((*sIter)->suspendedOutDevIds.size() == 1 /* non combo */)) {
                for (auto device: devices) {
                    streamDevDisconnect.push_back({ (*sIter), device->getSndDeviceId() });
                }
            }
            restoredStreams.push_back((*sIter));
            streamDevConnect.push_back({(*sIter), &a2dpDattr});
        } else if (std::find((*sIter)->suspendedOutDevIds.begin(),
                (*sIter)->suspendedOutDevIds.end(), PAL_DEVICE_OUT_BLUETOOTH_SCO)
                != (*sIter)->suspendedOutDevIds.end()) {
            std::vector<std::shared_ptr<Device>> devices;
            (*sIter)->getAssociatedOutDevices(devices);
            if (devices.size() > 0) {
                for (auto device: devices) {
                    if (device->getSndDeviceId() == PAL_DEVICE_OUT_BLUETOOTH_SCO) {
                        streamDevDisconnect.push_back({(*sIter), PAL_DEVICE_OUT_BLUETOOTH_SCO});
                        break;
                    }
                }
            }
            restoredStreams.push_back((*sIter));
            streamDevConnect.push_back({(*sIter), &a2dpDattr});
        }
        (*sIter)->unlockStreamMutex();
    }

    if (restoredStreams.empty()) {
        PAL_DBG(LOG_TAG, "no streams to be restored");
        rm->unlockActiveStream();
        goto exit;
    }
    SortAndUnique(restoredStreams);
    rm->unlockActiveStream();

    /* In case of SSR down event if a2dpSuspended = false sets, since sound card state is offline
    * rm->streamDevSwitch() operation will be skipped. Due to this mDevices will remain as speaker
    * and when SSR is up, active streams will route to the speaker only.
    * Thus in this corner scenario, update the mDevices as BT A2DP so that when SSR is up
    * active streams will be routed to BT properly.
    */
    if (PAL_CARD_STATUS_DOWN(rm->getSoundCardState())) {
        PAL_ERR(LOG_TAG, "Sound card offline");
        rm->lockActiveStream();
        for (sIter = restoredStreams.begin(); sIter != restoredStreams.end(); sIter++) {
            if (((*sIter) != NULL) && rm->isStreamActive(*sIter)) {
                (*sIter)->lockStreamMutex();
                if (std::find((*sIter)->suspendedOutDevIds.begin(),
                        (*sIter)->suspendedOutDevIds.end(), a2dpDattr.id)
                        != (*sIter)->suspendedOutDevIds.end()) {
                    if ((*sIter)->suspendedOutDevIds.size() == 1 /* non-combo */) {
                        (*sIter)->clearmDevices();
                        (*sIter)->clearOutPalDevices(*sIter);
                    }
                    (*sIter)->addmDevice(&a2dpDattr);
                    (*sIter)->addPalDevice(*sIter, &a2dpDattr);
                } else if (std::find((*sIter)->suspendedOutDevIds.begin(),
                        (*sIter)->suspendedOutDevIds.end(), PAL_DEVICE_OUT_BLUETOOTH_SCO)
                        != (*sIter)->suspendedOutDevIds.end()) {
                    (*sIter)->removemDevice(PAL_DEVICE_OUT_BLUETOOTH_SCO);
                    (*sIter)->removePalDevice(*sIter, PAL_DEVICE_OUT_BLUETOOTH_SCO);
                    (*sIter)->addmDevice(&a2dpDattr);
                    (*sIter)->addPalDevice(*sIter, &a2dpDattr);
                }
                (*sIter)->unlockStreamMutex();
            }
        }
        rm->unlockActiveStream();
    } else {
        PAL_DBG(LOG_TAG, "restoring A2dp and unmuting stream");
        status = rm->streamDevSwitch(streamDevDisconnect, streamDevConnect);
        if (status) {
            PAL_ERR(LOG_TAG, "streamDevSwitch failed %d", status);
            goto exit;
        }
    }

    rm->lockActiveStream();
    for (sIter = restoredStreams.begin(); sIter != restoredStreams.end(); sIter++) {
        if (((*sIter) != NULL) && rm->isStreamActive(*sIter)) {
            (*sIter)->lockStreamMutex();
            // update PAL devices for the restored streams
            if ((*sIter)->suspendedOutDevIds.size() == 1 /* non-combo */) {
                (*sIter)->clearOutPalDevices(*sIter);
            } else if (std::find((*sIter)->suspendedOutDevIds.begin(),
                    (*sIter)->suspendedOutDevIds.end(), PAL_DEVICE_OUT_BLUETOOTH_SCO)
                    != (*sIter)->suspendedOutDevIds.end()) {
                (*sIter)->removePalDevice(*sIter, PAL_DEVICE_OUT_BLUETOOTH_SCO);
            }
            (*sIter)->addPalDevice(*sIter, &a2dpDattr);

            (*sIter)->suspendedOutDevIds.clear();
            status = (*sIter)->getVolumeData(volume);
            if (status) {
                PAL_ERR(LOG_TAG, "getVolumeData failed %d", status);
                (*sIter)->unlockStreamMutex();
                continue;
            }
            /* set a2dpMuted to false so that volume can be applied
             * volume gets cached if a2dpMuted is set to true
             */
            (*sIter)->a2dpMuted = false;
            status = (*sIter)->setVolume(volume);
            if (status) {
                PAL_ERR(LOG_TAG, "setVolume failed %d", status);
                (*sIter)->a2dpMuted = true;
                (*sIter)->unlockStreamMutex();
                continue;
            }
            // set a2dpMuted to true in case unmute failed
            if ((*sIter)->mute_l(false))
                (*sIter)->a2dpMuted = true;
            (*sIter)->unlockStreamMutex();
        }
    }
    rm->unlockActiveStream();

exit:
    PAL_DBG(LOG_TAG, "exit status: %d", status);
    if (volume) {
        free(volume);
    }
    return status;
}

int32_t a2dpCaptureSuspend(pal_device_id_t dev_id)
{
    int status = 0;
    uint32_t prio;
    std::shared_ptr<Device> a2dpDev = nullptr;
    struct pal_device a2dpDattr = {};
    struct pal_device handsetmicDattr = {};
    std::vector <Stream*> activeA2dpStreams;
    std::vector <Stream*> activeStreams;
    std::shared_ptr<Device> handsetmicDev = nullptr;
    std::vector <Stream*>::iterator sIter;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    PAL_DBG(LOG_TAG, "enter");

    a2dpDattr.id = dev_id;
    a2dpDev = Device::getInstance(&a2dpDattr, rm);

    if (!a2dpDev) {
        PAL_ERR(LOG_TAG, "Getting a2dp/ble device instance failed");
        goto exit;
    }

    rm->lockActiveStream();
    rm->getActiveStream_l(activeA2dpStreams, a2dpDev);
    if (activeA2dpStreams.size() == 0) {
        PAL_DBG(LOG_TAG, "no active streams found");
        rm->unlockActiveStream();
        goto exit;
    }

    // force switch to handset_mic
    handsetmicDattr.id = PAL_DEVICE_IN_HANDSET_MIC;
    handsetmicDev = Device::getInstance(&handsetmicDattr, rm);
    if (!handsetmicDev) {
        PAL_ERR(LOG_TAG, "Getting handset-mic device instance failed");
        rm->unlockActiveStream();
        goto exit;
    }
    /* Check if any stream device attribute pair is already there for
     * the handset and use its attributes before deciding on
     * using default device info
     */
    status = handsetmicDev->getTopPriorityDeviceAttr(&handsetmicDattr, &prio);
    if (status) {
        // No active streams on handset-mic, get default dev info
        pal_device_info devInfo;
        memset(&devInfo, 0, sizeof(pal_device_info));
        status = rm->getDeviceConfig(&handsetmicDattr, NULL);
        if (!status) {
            // get the default device info and update snd name
            rm->getDeviceInfo(handsetmicDattr.id, (pal_stream_type_t)0,
                handsetmicDattr.custom_config.custom_key, &devInfo);
            rm->updateSndName(handsetmicDattr.id, devInfo.sndDevName);
        }
    }

    for (sIter = activeA2dpStreams.begin(); sIter != activeA2dpStreams.end(); sIter++) {
        if (((*sIter) != NULL) && rm->isStreamActive(*sIter)) {
            (*sIter)->lockStreamMutex();
            if (!((*sIter)->a2dpMuted)) {
                (*sIter)->mute_l(true);
                (*sIter)->a2dpMuted = true;
            }
            (*sIter)->unlockStreamMutex();
        }
    }
    rm->unlockActiveStream();

    PAL_DBG(LOG_TAG, "selecting handset_mic and muting stream");
    rm->forceDeviceSwitch(a2dpDev, &handsetmicDattr, activeA2dpStreams);

    rm->lockActiveStream();
    for (sIter = activeA2dpStreams.begin(); sIter != activeA2dpStreams.end(); sIter++) {
        if (((*sIter) != NULL) && rm->isStreamActive(*sIter)) {
            (*sIter)->suspendedInDevIds.clear();
            (*sIter)->suspendedInDevIds.push_back(a2dpDattr.id);
        }
    }
    rm->unlockActiveStream();

exit:
    PAL_DBG(LOG_TAG, "exit status: %d", status);
    return status;
}

int32_t a2dpCaptureResume(pal_device_id_t dev_id)
{
    int status = 0;
    std::shared_ptr<Device> activeDev = nullptr;
    struct pal_device activeDattr = {};
    struct pal_device a2dpDattr = {};
    struct pal_device_info devinfo = {};
    std::vector <Stream*>::iterator sIter;
    std::vector <Stream*> activeStreams;
    std::vector <Stream*> orphanStreams;
    std::vector <Stream*> retryStreams;
    std::vector <Stream*> restoredStreams;
    std::vector <std::tuple<Stream*, uint32_t>> streamDevDisconnect;
    std::vector <std::tuple<Stream*, struct pal_device*>> streamDevConnect;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    PAL_DBG(LOG_TAG, "enter");

    activeDattr.id = PAL_DEVICE_IN_HANDSET_MIC;
    activeDev = Device::getInstance(&activeDattr, rm);
    a2dpDattr.id = dev_id;

    status = rm->getDeviceConfig(&a2dpDattr, NULL);
    if (status) {
        goto exit;
    }

    rm->lockActiveStream();
    rm->getActiveStream_l(activeStreams, activeDev);

    /* No-Streams active on Handset-mic - possibly streams are
     * associated speaker-mic device (due to camorder app UC) and
     * device switch did not happen for all the streams
     */
    if (activeStreams.empty()) {
        // Hence try to check speaker-mic device as well.
        activeDattr.id = PAL_DEVICE_IN_SPEAKER_MIC;
        activeDev = Device::getInstance(&activeDattr, rm);
        rm->getActiveStream_l(activeStreams, activeDev);
    }

    rm->getOrphanStream_l(orphanStreams, retryStreams);
    if (activeStreams.empty() && orphanStreams.empty() && retryStreams.empty()) {
        PAL_DBG(LOG_TAG, "no active streams found");
        rm->unlockActiveStream();
        goto exit;
    }

    // check all active streams associated with handset_mic.
    // If actual device is a2dp, store into stream vector for device switch.
    for (sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++) {
        if (std::find((*sIter)->suspendedInDevIds.begin(),
                (*sIter)->suspendedInDevIds.end(), a2dpDattr.id)
                != (*sIter)->suspendedInDevIds.end()) {
            restoredStreams.push_back((*sIter));
            streamDevDisconnect.push_back({ (*sIter), activeDattr.id });
            streamDevConnect.push_back({ (*sIter), &a2dpDattr });
        }
    }

    // retry all orphan streams which failed to restore previously.
    for (sIter = orphanStreams.begin(); sIter != orphanStreams.end(); sIter++) {
        if (std::find((*sIter)->suspendedInDevIds.begin(),
                (*sIter)->suspendedInDevIds.end(), a2dpDattr.id)
                != (*sIter)->suspendedInDevIds.end()) {
            restoredStreams.push_back((*sIter));
            streamDevConnect.push_back({ (*sIter), &a2dpDattr });
        }
    }

    // retry all streams which failed to switch to desired device previously.
    for (sIter = retryStreams.begin(); sIter != retryStreams.end(); sIter++) {
        (*sIter)->lockStreamMutex();
        if (std::find((*sIter)->suspendedInDevIds.begin(),
                (*sIter)->suspendedInDevIds.end(), a2dpDattr.id)
                != (*sIter)->suspendedInDevIds.end()) {
            std::vector<std::shared_ptr<Device>> devices;
            (*sIter)->getAssociatedInDevices(devices);
            if (devices.size() > 0) {
                for (auto device : devices) {
                    streamDevDisconnect.push_back({ (*sIter), device->getSndDeviceId() });
                }
            }
            restoredStreams.push_back((*sIter));
            streamDevConnect.push_back({ (*sIter), &a2dpDattr });
        }
        (*sIter)->unlockStreamMutex();
    }

    if (restoredStreams.empty()) {
        PAL_DBG(LOG_TAG, "no streams to be restored");
        rm->unlockActiveStream();
        goto exit;
    }
    rm->unlockActiveStream();

    PAL_DBG(LOG_TAG, "restoring A2dp and unmuting stream");
    status = rm->streamDevSwitch(streamDevDisconnect, streamDevConnect);
    if (status) {
        PAL_ERR(LOG_TAG, "rm->streamDevSwitch failed %d", status);
        goto exit;
    }

    rm->lockActiveStream();
    for (sIter = restoredStreams.begin(); sIter != restoredStreams.end(); sIter++) {
        if ((*sIter) && rm->isStreamActive(*sIter)) {
            (*sIter)->lockStreamMutex();
            (*sIter)->suspendedInDevIds.clear();
            (*sIter)->mute_l(false);
            (*sIter)->a2dpMuted = false;
            (*sIter)->unlockStreamMutex();
        }
    }
    rm->unlockActiveStream();

exit:
    PAL_DBG(LOG_TAG, "exit status: %d", status);
    return status;
}

void updateBtCodecMap(std::pair<uint32_t, std::string> key, std::string value)
{
    btCodecMap.insert(std::make_pair(key, value));
}

std::string getBtCodecLib(uint32_t codecFormat, std::string codecType)
{
    std::map<std::pair<uint32_t, std::string>, std::string>::iterator iter;

    iter = btCodecMap.find(std::make_pair(codecFormat, codecType));
    if (iter != btCodecMap.end()) {
        return iter->second;
    }

    return std::string();
}

void updateBtSlimClockSrcMap(uint32_t key, uint32_t value)
{
    btSlimClockSrcMap.insert(std::make_pair(key, value));
}

uint32_t getBtSlimClockSrc(uint32_t codecFormat)
{
    std::map<uint32_t, std::uint32_t>::iterator iter;

    iter = btSlimClockSrcMap.find(codecFormat);
    if (iter != btSlimClockSrcMap.end())
        return iter->second;

    return CLOCK_SRC_DEFAULT;
}

void processBTCodecInfo(const XML_Char **attr, const int attr_count)
{
    char *saveptr = NULL;
    char *token = NULL;
    std::vector<std::string> codec_formats, codec_types;

    if (strcmp(attr[0], "codec_format") != 0) {
        PAL_ERR(LOG_TAG,"'codec_format' not found");
        goto done;
    }

    if (strcmp(attr[2], "codec_type") != 0) {
        PAL_ERR(LOG_TAG,"'codec_type' not found");
        goto done;
    }

    if (strcmp(attr[4], "codec_library") != 0) {
        PAL_ERR(LOG_TAG,"'codec_library' not found");
        goto done;
    }

    token = strtok_r((char *)attr[1], "|", &saveptr);
    while (token != NULL) {
        if (strlen(token) != 0) {
            codec_formats.push_back(std::string(token));
        }
        token = strtok_r(NULL, "|", &saveptr);
    }

    token = strtok_r((char *)attr[3], "|", &saveptr);
    while (token != NULL) {
        if (strlen(token) != 0) {
            codec_types.push_back(std::string(token));
        }
        token = strtok_r(NULL, "|", &saveptr);
    }

    if (codec_formats.empty() || codec_types.empty()) {
        PAL_INFO(LOG_TAG,"BT codec formats or types empty!");
        goto done;
    }

    for (std::vector<std::string>::iterator iter1 = codec_formats.begin();
            iter1 != codec_formats.end(); ++iter1) {
        if (btFmtTable.find(*iter1) != btFmtTable.end()) {
            for (std::vector<std::string>::iterator iter2 = codec_types.begin();
                    iter2 != codec_types.end(); ++iter2) {
                PAL_VERBOSE(LOG_TAG, "BT Codec Info %s=%s, %s=%s, %s=%s",
                        attr[0], (*iter1).c_str(), attr[2], (*iter2).c_str(), attr[4], attr[5]);

                updateBtCodecMap(std::make_pair(btFmtTable[*iter1], *iter2),  std::string(attr[5]));
            }

            /* Clock is an optional attribute unlike codec_format & codec_type.
             * Check attr count before accessing it.
             * attr[6] & attr[7] reserved for clock source value
             */
            if (attr_count >= 8 && strcmp(attr[6], "clock") == 0) {
                PAL_VERBOSE(LOG_TAG, "BT Codec Info %s=%s, %s=%s",
                        attr[0], (*iter1).c_str(), attr[6], attr[7]);
                updateBtSlimClockSrcMap(btFmtTable[*iter1], atoi(attr[7]));
            }
        }
    }

done:
    return;
}

void reconfigureScoStreams() {
    int status = 0;
    std::shared_ptr <Device> dev = nullptr;
    std::vector <pal_device_id_t> scoDevs;
    std::vector <Stream *> activeScoStreams;
    std::vector <Stream *>::iterator sIter;
    std::vector <std::tuple<Stream*, uint32_t>> streamDevDisconnect;
    std::vector <std::tuple<Stream*, struct pal_device *>> streamDevConnect;
    std::vector <pal_device *> palDevices;
    struct pal_device *dattr = nullptr;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    std::vector <Stream *> reconfigureStreams;

    scoDevs.push_back(PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET);
    scoDevs.push_back(PAL_DEVICE_OUT_BLUETOOTH_SCO);
    for (auto devId : scoDevs) {
        dattr = (pal_device *) malloc(sizeof(struct pal_device));
        if (!dattr) {
            PAL_ERR(LOG_TAG, "malloc failed for SCO device %d", devId);
            goto exit;
        }
        palDevices.push_back(dattr);
        dattr->id = devId;
        status = rm->getDeviceConfig(dattr, NULL);
        if (status) {
            PAL_ERR(LOG_TAG, "getDeviceConfig failed for SCO device %d", devId);
            goto exit;
        }
        dev = Device::getInstance(dattr, rm);
        activeScoStreams.clear();
        rm->lockActiveStream();
        rm->getActiveStream_l(activeScoStreams, dev);
        if (activeScoStreams.size() > 0) {
            for (sIter = activeScoStreams.begin();
                sIter != activeScoStreams.end(); sIter++) {
                status = rm->increaseStreamUserCounter(*sIter);
                reconfigureStreams.push_back(*sIter);
                if (0 != status) {
                    PAL_ERR(LOG_TAG,
                            "Error incrementing the stream counter for the stream handle: %pK",
                            *sIter);
                    continue;
                }
                streamDevDisconnect.push_back({*sIter, devId});
                streamDevConnect.push_back({*sIter, dattr});
            }
        }
        rm->unlockActiveStream();
        dattr = nullptr;
    }

    PAL_DBG(LOG_TAG, "streamDevDisconnect size=%d and streamDevConnect size=%d",
            streamDevDisconnect.size(), streamDevConnect.size());
    status = rm->streamDevSwitch(streamDevDisconnect, streamDevConnect);
    if (status) {
        PAL_ERR(LOG_TAG, "streamDevSwitch failed %d", status);
    }
    rm->lockActiveStream();
    if (reconfigureStreams.size() > 0) {
        for (sIter = reconfigureStreams.begin();
            sIter != reconfigureStreams.end(); sIter++) {
            status = rm->decreaseStreamUserCounter(*sIter);
            if (0 != status) {
                PAL_ERR(LOG_TAG,
                        "Error decrementing the stream counter for the stream handle: %pK",
                        *sIter);
                continue;
            }
        }
    }
    rm->unlockActiveStream();
exit:
    for (auto dAttr : palDevices) free(dAttr);
}

int setBTParameter(uint32_t param_id, void *param_payload,
                     size_t payload_size) {
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    int status = 0;
    std::shared_ptr<Device> dev = nullptr;
    struct pal_device dattr;

    PAL_DBG(LOG_TAG, "Enter param id: %d", param_id);
    switch (param_id) {
        case PAL_PARAM_ID_BT_SCO:
        {
            pal_param_btsco_t* param_bt_sco = nullptr;
            bool isScoOn = false;

            std::vector<pal_device_id_t> scoDev;
            std::vector<pal_device_id_t> hfpDev;

            scoDev.push_back(PAL_DEVICE_OUT_BLUETOOTH_SCO);
            scoDev.push_back(PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET);
            // 1. set SCO on for SCO devices if they are available.
            param_bt_sco = (pal_param_btsco_t*)param_payload;
            for (auto devId: scoDev) {
                dattr.id = devId;
                dev = Device::getInstance(&dattr, rm);
                if (!dev) {
                    PAL_ERR(LOG_TAG, "Device getInstance failed");
                    status = -ENODEV;
                    goto exit;
                }
                isScoOn = rm->isDeviceReady(dattr.id);
                if (isScoOn != param_bt_sco->bt_sco_on) {
                    status = dev->setDeviceParameter(param_id, param_payload);
                    if (status)
                        PAL_ERR(LOG_TAG, "set Parameter %d failed on SCO devices", param_id);
                } else {
                    PAL_INFO(LOG_TAG, "SCO already in requested state, ignoring");
                }
            }

            if (param_bt_sco->bt_sco_on) {
                rm->unlockResourceManagerMutex();
                reconfigureScoStreams();
                rm->lockResourceManagerMutex();
            }
            // 2.set SCO on/HFP on for HFP devices if they are available as well.
            hfpDev.push_back(PAL_DEVICE_OUT_BLUETOOTH_HFP);
            hfpDev.push_back(PAL_DEVICE_IN_BLUETOOTH_HFP);
            for (auto devId: hfpDev) {
                dattr.id = devId;
                if (!rm->isDeviceAvailable(dattr.id))
                    continue;
                dev = Device::getInstance(&dattr, rm);
                if (!dev) {
                    PAL_ERR(LOG_TAG, "Device getInstance failed");
                    status = -ENODEV;
                    goto exit;
                }
                isScoOn = rm->isDeviceReady(dattr.id);
                if (isScoOn != param_bt_sco->bt_sco_on) {
                    status = dev->setDeviceParameter(param_id, param_payload);
                    if (status) {
                        PAL_ERR(LOG_TAG, "set Parameter %d failed on HFP devices", param_id);
                    }
                } else {
                    PAL_INFO(LOG_TAG, "SCO already in requested state for HFP device, ignoring");
                }
            }
        }
        break;
        case PAL_PARAM_ID_BT_SCO_WB:
        case PAL_PARAM_ID_BT_SCO_SWB:
        case PAL_PARAM_ID_BT_SCO_LC3:
        case PAL_PARAM_ID_BT_SCO_NREC:
        {
            std::vector<pal_device_id_t> scoDev;
            scoDev.push_back(PAL_DEVICE_OUT_BLUETOOTH_SCO);
            scoDev.push_back(PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET);
            struct pal_device dattr;
            rm->unlockResourceManagerMutex();
            for (auto devId: scoDev) {
                dattr.id = devId;
                dev = Device::getInstance(&dattr, rm);
                if (dev)
                    status = dev->setDeviceParameter(param_id, param_payload);
            }
            if (rm->isDeviceAvailable(PAL_DEVICE_OUT_BLUETOOTH_HFP)) {
                dattr.id = PAL_DEVICE_OUT_BLUETOOTH_HFP;
                dev = Device::getInstance(&dattr, rm);
                if (dev) {
                    status = dev->setDeviceParameter(param_id, param_payload);
                }
            }
            if (rm->isDeviceAvailable(PAL_DEVICE_IN_BLUETOOTH_HFP)) {
                dattr.id = PAL_DEVICE_IN_BLUETOOTH_HFP;
                dev = Device::getInstance(&dattr, rm);
                if (dev) {
                    status = dev->setDeviceParameter(param_id, param_payload);
                }
            }
            rm->lockResourceManagerMutex();
        }
        break;
        case PAL_PARAM_ID_BT_A2DP_RECONFIG:
        {
            std::shared_ptr<Device> dev = nullptr;
            std::vector <Stream *> activeA2dpStreams;
            struct pal_device dattr;
            pal_param_bta2dp_t *current_param_bt_a2dp = nullptr;
            pal_param_bta2dp_t param_bt_a2dp;
            int retrycnt = 3;
            const int retryPeriodMs = 100;

            if (rm->isDeviceAvailable(PAL_DEVICE_OUT_BLUETOOTH_A2DP)) {
                dattr.id = PAL_DEVICE_OUT_BLUETOOTH_A2DP;
                dev = Device::getInstance(&dattr, rm);
                if (!dev) {
                    PAL_ERR(LOG_TAG, "Device getInstance failed");
                    status = -ENODEV;
                    goto exit;
                }

                if (dev->checkDeviceStatus() == A2DP_STATE_DISCONNECTED) {
                    PAL_ERR(LOG_TAG, "failed to open A2dp source, skip a2dp reconfig.");
                    status = -ENODEV;
                    goto exit;
                }

                rm->getActiveStream_l(activeA2dpStreams, dev);
                if (activeA2dpStreams.size() == 0) {
                    PAL_DBG(LOG_TAG, "no active a2dp stream available, skip a2dp reconfig.");
                    status = 0;
                    goto exit;
                }

                dev->setDeviceParameter(param_id, param_payload);
                dev->getDeviceParameter(param_id, (void **)&current_param_bt_a2dp);
                if ((current_param_bt_a2dp->reconfig == true) &&
                    (current_param_bt_a2dp->a2dp_suspended == false)) {
                    rm->unlockResourceManagerMutex();
                    status = a2dpReconfig();

                    /* During reconfig stage, if a2dp is not in a ready state streamdevswitch
                     * to BT will be failed. Reiterate the a2dpreconfig until it
                     * succeeds with sleep period of 100 msecs and retry count = 20.
                     */
                    while ((status != 0) && (retrycnt > 0)) {
                        if (rm->isDeviceReady(PAL_DEVICE_OUT_BLUETOOTH_A2DP)) {
                            status = a2dpReconfig();
                        }
                        usleep(retryPeriodMs * 1000);
                        retrycnt--;
                    }

                    rm->lockResourceManagerMutex();

                    param_bt_a2dp.reconfig = false;
                    dev->setDeviceParameter(param_id, &param_bt_a2dp);
                }
            }
        }
        break;
        case PAL_PARAM_ID_BT_A2DP_SUSPENDED:
        {
            std::shared_ptr<Device> a2dp_dev = nullptr;
            struct pal_device a2dp_dattr;
            pal_param_bta2dp_t *current_param_bt_a2dp = nullptr;
            pal_param_bta2dp_t *param_bt_a2dp = nullptr;
            bool skip_switch = false;

            rm->unlockResourceManagerMutex();
            param_bt_a2dp = (pal_param_bta2dp_t*)param_payload;
            a2dp_dattr.id = param_bt_a2dp->dev_id;

            a2dp_dev = Device::getInstance(&a2dp_dattr , rm);
            if (!a2dp_dev) {
                PAL_ERR(LOG_TAG, "Failed to get A2DP/BLE instance");
                status = -ENODEV;
                goto exit_no_unlock;
            }

            a2dp_dev->getDeviceParameter(param_id, (void **)&current_param_bt_a2dp);
            /* If device is already suspended from framework, ignore suspend/resume
             * which is sent via reconfig_cb. Honouring the param in such scenario
             * will lead to incorrect stream state.
             */
            if (current_param_bt_a2dp->a2dp_suspended &&
                current_param_bt_a2dp->is_suspend_setparam &&
                !param_bt_a2dp->is_suspend_setparam) {
                PAL_INFO(LOG_TAG, "suspend/resume from reconfig_cb ignored");
                goto exit_no_unlock;
            }

            if (current_param_bt_a2dp->a2dp_suspended ==
                    param_bt_a2dp->a2dp_suspended) {
                PAL_INFO(LOG_TAG, "A2DP/BLE already in requested state, ignoring");
                goto exit_no_unlock;
            }

            if ((!rm->isDeviceAvailable(param_bt_a2dp->dev_id)) ||
                 (param_bt_a2dp->is_suspend_setparam && param_bt_a2dp->is_in_call))
                skip_switch = true;

            if (rm->IsDummyDevEnabled()) {
                if (!skip_switch && param_bt_a2dp->a2dp_suspended == false) {
                    struct pal_device sco_tx_dattr = {};
                    struct pal_device sco_rx_dattr = {};
                    std::shared_ptr<Device> sco_tx_dev = nullptr;
                    std::shared_ptr<Device> sco_rx_dev = nullptr;
                    struct pal_device in_dummy_dattr = {};
                    std::vector<Stream *> activestreams;
                    std::vector<Stream *>::iterator sIter;
                    pal_stream_type_t streamType;

                    /* Handle bt sco mic running usecase */
                    sco_tx_dattr.id = PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET;
                    if (rm->isDeviceAvailable(sco_tx_dattr.id)) {
                        in_dummy_dattr.id = PAL_DEVICE_IN_DUMMY;
                        sco_tx_dev = Device::getInstance(&sco_tx_dattr, rm);
                        rm->lockActiveStream();
                        rm->getActiveStream_l(activestreams, sco_tx_dev);
                        rm->unlockActiveStream();
                        if (activestreams.size() > 0) {
                            /* Mark streams over IN_SCO, so as to give them chance
                             * to resume over A2DP/BLE if a2dpCatureSuspend=false is
                             * received at later stage.
                             */
                            for (sIter = activestreams.begin();
                                     sIter != activestreams.end(); sIter++) {
                                (*sIter)->suspendedInDevIds.clear();
                                if (rm->isDeviceAvailable(PAL_DEVICE_IN_BLUETOOTH_A2DP))
                                    (*sIter)->suspendedInDevIds.
                                                  push_back(PAL_DEVICE_IN_BLUETOOTH_A2DP);
                                else if (rm->isDeviceAvailable(PAL_DEVICE_IN_BLUETOOTH_BLE))
                                    (*sIter)->suspendedInDevIds.
                                                  push_back(PAL_DEVICE_IN_BLUETOOTH_BLE);
                            }
                            PAL_DBG(LOG_TAG, "a2dp resumed, switch bt sco mic to in_dummy device");
                            rm->getDeviceConfig(&in_dummy_dattr, NULL);
                            rm->forceDeviceSwitch(sco_tx_dev, &in_dummy_dattr);
                        }
                    }

                    /* Handle bt sco out running usecase */
                    rm->lockActiveStream();
                    sco_rx_dattr.id = PAL_DEVICE_OUT_BLUETOOTH_SCO;
                    if (rm->isDeviceAvailable(sco_rx_dattr.id)) {
                        sco_rx_dev = Device::getInstance(&sco_rx_dattr, rm);
                        rm->getActiveStream_l(activestreams, sco_rx_dev);
                        for (sIter = activestreams.begin(); sIter != activestreams.end(); sIter++) {
                            status = (*sIter)->getStreamType(&streamType);
                            if (0 != status) {
                                PAL_ERR(LOG_TAG, "getStreamType failed with status = %d", status);
                                continue;
                            }
                            if ((streamType == PAL_STREAM_LOW_LATENCY) ||
                                (streamType == PAL_STREAM_ULTRA_LOW_LATENCY) ||
                                (streamType == PAL_STREAM_VOIP_RX) ||
                                (streamType == PAL_STREAM_PCM_OFFLOAD) ||
                                (streamType == PAL_STREAM_DEEP_BUFFER) ||
                                (streamType == PAL_STREAM_SPATIAL_AUDIO) ||
                                (streamType == PAL_STREAM_COMPRESSED) ||
                                (streamType == PAL_STREAM_GENERIC)) {
                                if ((*sIter)->suspendedOutDevIds.empty()) {
                                    (*sIter)->suspendedOutDevIds.
                                        push_back(PAL_DEVICE_OUT_BLUETOOTH_SCO);
                                    PAL_DBG(LOG_TAG, "a2dp resumed, \
                                        mark sco streams as to route them later");
                                }
                            }
                        }
                    }
                    rm->unlockActiveStream();
                }
            } else {
                if (!skip_switch && param_bt_a2dp->a2dp_suspended == false) {
                    struct pal_device sco_tx_dattr;
                    struct pal_device sco_rx_dattr;
                    std::shared_ptr<Device> sco_tx_dev = nullptr;
                    std::shared_ptr<Device> sco_rx_dev = nullptr;
                    struct pal_device handset_tx_dattr = {};
                    struct pal_device_info devInfo = {};
                    struct pal_stream_attributes sAttr;
                    std::vector<Stream*> activestreams;
                    std::vector<Stream*>::iterator sIter;
                    Stream *stream = NULL;
                    pal_stream_type_t streamType;

                    rm->lockActiveStream();
                    /* Handle bt sco mic running usecase */
                    sco_tx_dattr.id = PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET;
                    if (rm->isDeviceAvailable(sco_tx_dattr.id)) {
                        handset_tx_dattr.id = PAL_DEVICE_IN_HANDSET_MIC;
                        sco_tx_dev = Device::getInstance(&sco_tx_dattr, rm);
                        rm->getActiveStream_l(activestreams, sco_tx_dev);
                        if (activestreams.size() > 0) {
                            /* Mark streams over IN_SCO, so as to give them chance
                             * to resume over A2DP/BLE if a2dpCatureSuspend=false is
                             * received at later stage.
                             */
                            for (sIter = activestreams.begin();
                                     sIter != activestreams.end(); sIter++) {
                                (*sIter)->suspendedInDevIds.clear();
                                if (rm->isDeviceAvailable(PAL_DEVICE_IN_BLUETOOTH_A2DP))
                                    (*sIter)->suspendedInDevIds.
                                                  push_back(PAL_DEVICE_IN_BLUETOOTH_A2DP);
                                else if (rm->isDeviceAvailable(PAL_DEVICE_IN_BLUETOOTH_BLE))
                                    (*sIter)->suspendedInDevIds.
                                                  push_back(PAL_DEVICE_IN_BLUETOOTH_BLE);
                            }
                            PAL_DBG(LOG_TAG, "a2dp resumed, switch bt sco mic to handset mic");
                            stream = static_cast<Stream *>(activestreams[0]);
                            stream->getStreamAttributes(&sAttr);
                            rm->getDeviceConfig(&handset_tx_dattr, &sAttr);
                            rm->getDeviceInfo(handset_tx_dattr.id, sAttr.type,
                                    handset_tx_dattr.custom_config.custom_key, &devInfo);
                            rm->updateSndName(handset_tx_dattr.id, devInfo.sndDevName);
                            rm->unlockActiveStream();
                            rm->forceDeviceSwitch(sco_tx_dev, &handset_tx_dattr);
                            rm->lockActiveStream();
                        }
                    }

                    /* Handle bt sco running usecase */
                    sco_rx_dattr.id = PAL_DEVICE_OUT_BLUETOOTH_SCO;
                    if (rm->isDeviceAvailable(sco_rx_dattr.id)) {
                        sco_rx_dev = Device::getInstance(&sco_rx_dattr, rm);
                        rm->getActiveStream_l(activestreams, sco_rx_dev);
                        for (sIter = activestreams.begin(); sIter != activestreams.end(); sIter++) {
                            status = (*sIter)->getStreamType(&streamType);
                            if (0 != status) {
                                PAL_ERR(LOG_TAG, "getStreamType failed with status = %d", status);
                                continue;
                            }
                            if ((streamType == PAL_STREAM_LOW_LATENCY) ||
                                (streamType == PAL_STREAM_ULTRA_LOW_LATENCY) ||
                                (streamType == PAL_STREAM_VOIP_RX) ||
                                (streamType == PAL_STREAM_PCM_OFFLOAD) ||
                                (streamType == PAL_STREAM_DEEP_BUFFER) ||
                                (streamType == PAL_STREAM_SPATIAL_AUDIO) ||
                                (streamType == PAL_STREAM_COMPRESSED) ||
                                (streamType == PAL_STREAM_GENERIC)) {
                                if ((*sIter)->suspendedOutDevIds.empty()) {
                                    (*sIter)->suspendedOutDevIds.
                                        push_back(PAL_DEVICE_OUT_BLUETOOTH_SCO);
                                    PAL_DBG(LOG_TAG, "a2dp resumed, \
                                        mark sco streams as to route them later");
                                }
                            }
                        }
                    }
                    rm->unlockActiveStream();
                }
            }

            status = a2dp_dev->setDeviceParameter(param_id, param_payload);
            rm->lockResourceManagerMutex();
            if (status) {
                PAL_ERR(LOG_TAG, "set Parameter %d failed\n", param_id);
                goto exit;
            }
        }
        break;
        case PAL_PARAM_ID_BT_A2DP_CAPTURE_SUSPENDED:
        {
            std::shared_ptr<Device> a2dp_dev = nullptr;
            struct pal_device a2dp_dattr = {};
            pal_param_bta2dp_t* current_param_bt_a2dp = nullptr;
            pal_param_bta2dp_t* param_bt_a2dp = nullptr;
            bool skip_switch = false;

            rm->unlockResourceManagerMutex();
            param_bt_a2dp = (pal_param_bta2dp_t*)param_payload;
            a2dp_dattr.id = param_bt_a2dp->dev_id;

            a2dp_dev = Device::getInstance(&a2dp_dattr, rm);
            if (!a2dp_dev) {
                PAL_ERR(LOG_TAG, "failed to get a2dp/ble capture instance");
                status = -ENODEV;
                goto exit_no_unlock;
            }

            a2dp_dev->getDeviceParameter(param_id, (void**)&current_param_bt_a2dp);
            /* If device is already suspended from framework, ignore suspend/resume
             * which is sent via reconfig_cb. Honouring the param in such scenario
             * will lead to incorrect stream state.
             */
            if (current_param_bt_a2dp->a2dp_capture_suspended &&
                current_param_bt_a2dp->is_suspend_setparam &&
                !param_bt_a2dp->is_suspend_setparam) {
                PAL_INFO(LOG_TAG, "suspend/resume from reconfig_cb ignored");
                goto exit_no_unlock;
            }

            if (current_param_bt_a2dp->a2dp_capture_suspended ==
                    param_bt_a2dp->a2dp_capture_suspended) {
                PAL_INFO(LOG_TAG, "a2dp/ble already in requested state, ignoring");
                goto exit_no_unlock;
            }

            if((!rm->isDeviceAvailable(param_bt_a2dp->dev_id)) ||
               (param_bt_a2dp->is_suspend_setparam && param_bt_a2dp->is_in_call))
                skip_switch = true;

            if (rm->IsDummyDevEnabled()) {
                if (!skip_switch && param_bt_a2dp->a2dp_capture_suspended == false) {
                    struct pal_device sco_rx_dattr = {};
                    std::shared_ptr<Device> sco_rx_dev = nullptr;
                    struct pal_device out_dummy_dattr = {};
                    std::vector<Stream*> activestreams;
                    struct pal_device sco_tx_dattr;
                    std::shared_ptr<Device> sco_tx_dev = nullptr;
                    std::vector<Stream*>::iterator sIter;
                    pal_stream_type_t streamType;

                    /* Handle bt sco out running usecase */
                    sco_rx_dattr.id = PAL_DEVICE_OUT_BLUETOOTH_SCO;
                    if (rm->isDeviceAvailable(sco_rx_dattr.id)) {
                        out_dummy_dattr.id = PAL_DEVICE_OUT_DUMMY;
                        sco_rx_dev = Device::getInstance(&sco_rx_dattr, rm);
                        rm->lockActiveStream();
                        rm->getActiveStream_l(activestreams, sco_rx_dev);
                        rm->unlockActiveStream();
                        if (activestreams.size() > 0) {
                            /* Mark streams over OUT_SCO, so as to give them chance
                             * to resume over A2DP/BLE if a2dpSuspend=false is
                             * received at later stage.
                             */
                            for (sIter = activestreams.begin();
                                     sIter != activestreams.end(); sIter++) {
                                (*sIter)->suspendedOutDevIds.clear();
                                if (rm->isDeviceAvailable(PAL_DEVICE_OUT_BLUETOOTH_A2DP))
                                    (*sIter)->suspendedOutDevIds.
                                                  push_back(PAL_DEVICE_OUT_BLUETOOTH_A2DP);
                                else if (rm->isDeviceAvailable(PAL_DEVICE_OUT_BLUETOOTH_BLE))
                                    (*sIter)->suspendedOutDevIds.
                                                  push_back(PAL_DEVICE_OUT_BLUETOOTH_BLE);
                            }
                            PAL_DBG(LOG_TAG, "a2dp resumed, switch bt sco out to out_dummy device");
                            rm->getDeviceConfig(&out_dummy_dattr, NULL);
                            rm->forceDeviceSwitch(sco_rx_dev, &out_dummy_dattr);
                        }
                    }

                    rm->lockActiveStream();
                    /* Handle bt sco running usecase */
                    sco_tx_dattr.id = PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET;
                    if (rm->isDeviceAvailable(sco_tx_dattr.id)) {
                        sco_tx_dev = Device::getInstance(&sco_tx_dattr, rm);
                        rm->getActiveStream_l(activestreams, sco_tx_dev);
                        for (sIter = activestreams.begin(); sIter != activestreams.end(); sIter++) {
                            status = (*sIter)->getStreamType(&streamType);
                            if (0 != status) {
                                PAL_ERR(LOG_TAG, "getStreamType failed with status = %d", status);
                                continue;
                            }
                            if ((streamType == PAL_STREAM_VOIP_TX) ||
                                (streamType == PAL_STREAM_DEEP_BUFFER)) {
                                (*sIter)->suspendedInDevIds.clear();
                                (*sIter)->suspendedInDevIds.push_back(a2dp_dattr.id);
                                PAL_DBG(LOG_TAG, "a2dp resumed, mark sco streams as to route them later");
                            }
                        }
                    }
                    rm->unlockActiveStream();
                }
            } else {
                if (!skip_switch && param_bt_a2dp->a2dp_capture_suspended == false) {
                    /* Handle bt sco out running usecase */
                    struct pal_device sco_rx_dattr;
                    struct pal_stream_attributes sAttr;
                    Stream* stream = NULL;
                    std::vector<Stream*> activestreams;
                    struct pal_device sco_tx_dattr;
                    std::shared_ptr<Device> sco_tx_dev = nullptr;
                    std::vector<Stream*>::iterator sIter;
                    pal_stream_type_t streamType;

                    rm->lockActiveStream();
                    sco_rx_dattr.id = PAL_DEVICE_OUT_BLUETOOTH_SCO;
                    PAL_DBG(LOG_TAG, "a2dp resumed, switch bt sco rx to speaker");
                    if (rm->isDeviceAvailable(sco_rx_dattr.id)) {
                        struct pal_device speaker_dattr;
                        std::shared_ptr<Device> sco_rx_dev = nullptr;

                        speaker_dattr.id = PAL_DEVICE_OUT_SPEAKER;
                        sco_rx_dev = Device::getInstance(&sco_rx_dattr, rm);
                        rm->getActiveStream_l(activestreams, sco_rx_dev);
                        if (activestreams.size() > 0) {
                            /* Mark streams over OUT_SCO, so as to give them chance
                             * to resume over A2DP/BLE if a2dpSuspend=false is
                             * received at later stage.
                             */
                            for (sIter = activestreams.begin();
                                     sIter != activestreams.end(); sIter++) {
                                (*sIter)->suspendedOutDevIds.clear();
                                if (rm->isDeviceAvailable(PAL_DEVICE_OUT_BLUETOOTH_A2DP))
                                    (*sIter)->suspendedOutDevIds.
                                                  push_back(PAL_DEVICE_OUT_BLUETOOTH_A2DP);
                                else if (rm->isDeviceAvailable(PAL_DEVICE_OUT_BLUETOOTH_BLE))
                                    (*sIter)->suspendedOutDevIds.
                                                  push_back(PAL_DEVICE_OUT_BLUETOOTH_BLE);
                            }
                            stream = static_cast<Stream*>(activestreams[0]);
                            stream->getStreamAttributes(&sAttr);
                            rm->getDeviceConfig(&speaker_dattr, &sAttr);
                            rm->unlockActiveStream();
                            rm->forceDeviceSwitch(sco_rx_dev, &speaker_dattr);
                            rm->lockActiveStream();
                        }
                    }

                    /* Handle bt sco running usecase */
                    sco_tx_dattr.id = PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET;
                    if (rm->isDeviceAvailable(sco_tx_dattr.id)) {
                        sco_tx_dev = Device::getInstance(&sco_tx_dattr, rm);
                        rm->getActiveStream_l(activestreams, sco_tx_dev);
                        for (sIter = activestreams.begin(); sIter != activestreams.end(); sIter++) {
                            status = (*sIter)->getStreamType(&streamType);
                            if (0 != status) {
                                PAL_ERR(LOG_TAG, "getStreamType failed with status = %d", status);
                                continue;
                            }
                            if ((streamType == PAL_STREAM_VOIP_TX) ||
                                 (streamType == PAL_STREAM_DEEP_BUFFER)) {
                                (*sIter)->suspendedInDevIds.clear();
                                (*sIter)->suspendedInDevIds.push_back(a2dp_dattr.id);
                                PAL_DBG(LOG_TAG, "a2dp resumed, mark sco streams as to route them later");
                            }
                        }
                    }
                    rm->unlockActiveStream();
                }
            }
            status = a2dp_dev->setDeviceParameter(param_id, param_payload);
            rm->lockResourceManagerMutex();
            if (status) {
                PAL_ERR(LOG_TAG, "set Parameter %d failed\n", param_id);
                goto exit;
            }
         }
         break;
        case PAL_PARAM_ID_BT_A2DP_TWS_CONFIG:
        case PAL_PARAM_ID_BT_A2DP_LC3_CONFIG:
        {
            std::shared_ptr<Device> dev = nullptr;
            struct pal_device dattr;

            if (rm->isDeviceAvailable(PAL_DEVICE_OUT_BLUETOOTH_A2DP)) {
                dattr.id = PAL_DEVICE_OUT_BLUETOOTH_A2DP;
            } else if (rm->isDeviceAvailable(PAL_DEVICE_OUT_BLUETOOTH_BLE)) {
                dattr.id = PAL_DEVICE_OUT_BLUETOOTH_BLE;
            } else if (rm->isDeviceAvailable(PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST)) {
                dattr.id = PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST;
            }
            dev = Device::getInstance(&dattr, rm);
            if (dev) {
                status = dev->setDeviceParameter(param_id, param_payload);
                if (status) {
                    PAL_ERR(LOG_TAG, "set Parameter %d failed\n", param_id);
                    goto exit;
                }
            }
        }
        break;
        case PAL_PARAM_ID_SET_SOURCE_METADATA: //Move to BTUtils?
        {
            struct pal_device dattr;
            std::shared_ptr<Device> dev = nullptr;
            if (rm->isDeviceAvailable(PAL_DEVICE_OUT_BLUETOOTH_BLE)) {
                dattr.id = PAL_DEVICE_OUT_BLUETOOTH_BLE;
                dev = Device::getInstance(&dattr, rm);
                if (!dev) {
                    PAL_ERR(LOG_TAG, "Device getInstance failed");
                    goto exit;
                }
                rm->unlockResourceManagerMutex();
                dev->setDeviceParameter(param_id, param_payload);
                rm->lockResourceManagerMutex();
            }

            if (rm->isDeviceAvailable(PAL_DEVICE_OUT_BLUETOOTH_A2DP)) {
                dattr.id = PAL_DEVICE_OUT_BLUETOOTH_A2DP;
                dev = Device::getInstance(&dattr, rm);
                if (!dev) {
                    PAL_ERR(LOG_TAG, "Device getInstance failed");
                    goto exit;
                }
                rm->unlockResourceManagerMutex();
                dev->setDeviceParameter(param_id, param_payload);
                rm->lockResourceManagerMutex();
            }
        }
        break;
        case PAL_PARAM_ID_SET_SINK_METADATA:
        {
            struct pal_device dattr;
            std::shared_ptr<Device> dev = nullptr;
            if (rm->isDeviceAvailable(PAL_DEVICE_IN_BLUETOOTH_BLE)) {
                dattr.id = PAL_DEVICE_IN_BLUETOOTH_BLE;
            } else {
                PAL_VERBOSE(LOG_TAG, "BLE device is unavailable");
                goto exit;
            }

            dev = Device::getInstance(&dattr, rm);
            if (!dev) {
                PAL_ERR(LOG_TAG, "Device getInstance failed");
                goto exit;
            }
            PAL_INFO(LOG_TAG, "PAL_PARAM_ID_SET_SINK_METADATA device setparam");
            rm->unlockResourceManagerMutex();
            dev->setDeviceParameter(param_id, param_payload);
            rm->lockResourceManagerMutex();
        }
        break;
        default:
            status = -ENOENT;
    }

exit:
    rm->unlockResourceManagerMutex();
exit_no_unlock:
    PAL_DBG(LOG_TAG, "Exit status: %d", status);
    return status;

}

int getBTParameter(uint32_t param_id, void **param_payload,
                     size_t *payload_size, void *query __unused) {
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    int status = 0;

    PAL_DBG(LOG_TAG, "Enter param id: %d", param_id);
    switch (param_id) {
        case PAL_PARAM_ID_BT_A2DP_RECONFIG_SUPPORTED:
        case PAL_PARAM_ID_BT_A2DP_SUSPENDED:
        case PAL_PARAM_ID_BT_A2DP_ENCODER_LATENCY:
        case PAL_PARAM_ID_BT_A2DP_CAPTURE_SUSPENDED:
        case PAL_PARAM_ID_BT_A2DP_DECODER_LATENCY:
        {
            std::shared_ptr<Device> dev = nullptr;
            struct pal_device dattr;
            pal_param_bta2dp_t* param_bt_a2dp = nullptr;
            dattr.id = (*(pal_param_bta2dp_t**)param_payload)->dev_id;

            dev = Device::getInstance(&dattr, rm);
            if (dev) {
                status = dev->getDeviceParameter(param_id, (void**)&param_bt_a2dp);
                if (status) {
                    PAL_ERR(LOG_TAG, "get Parameter %d failed\n", param_id);
                    goto exit;
                }
                *param_payload = param_bt_a2dp;
                *payload_size = sizeof(pal_param_bta2dp_t);
            }
        }
        break;
        default:
            status = -ENOENT;
    }

exit:
    return status;
}
