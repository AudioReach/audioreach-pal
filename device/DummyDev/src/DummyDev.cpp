/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define LOG_TAG "PAL: DummyDev"
#include "DummyDev.h"
#include "ResourceManager.h"
#include "Device.h"
#include "kvh2xml.h"
#include "Stream.h"
#include "Session.h"

extern "C" void CreateDummyDevice(struct pal_device *device,
                                    const std::shared_ptr<ResourceManager> rm,
                                    std::shared_ptr<Device> *dev) {
    *dev = DummyDev::getInstance(device, rm);

}

std::shared_ptr<Device> DummyDev::objRx = nullptr;
std::shared_ptr<Device> DummyDev::objTx = nullptr;

std::shared_ptr<Device> DummyDev::getInstance(struct pal_device *device,
                                              std::shared_ptr<ResourceManager> Rm)
{
    if (device->id == PAL_DEVICE_OUT_DUMMY) {
        if (!objRx) {
            std::lock_guard<std::mutex> lock(Device::mInstMutex);
            if (!objRx) {
                PAL_INFO(LOG_TAG, "creating instance for %d", device->id);
                std::shared_ptr<Device> sp(new DummyDev(device, Rm));
                objRx = sp;
            }
        }
        return objRx;
    } else {
        if (!objTx) {
            std::lock_guard<std::mutex> lock(Device::mInstMutex);
            if (!objTx) {
                PAL_INFO(LOG_TAG, "creating instance for %d", device->id);
                std::shared_ptr<Device> sp(new DummyDev(device, Rm));
                objTx = sp;
            }
        }
        return objTx;
    }
}


DummyDev::DummyDev(struct pal_device *device, std::shared_ptr<ResourceManager> Rm) :
Device(device, Rm)
{

}

DummyDev::~DummyDev()
{

}

