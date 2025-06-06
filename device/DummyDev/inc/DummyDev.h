/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef DUMMY_DEV_H
#define DUMMY_DEV_H

#include "Device.h"

extern "C" void CreateDummyDevice(struct pal_device *device,
                                    const std::shared_ptr<ResourceManager> rm,
                                    std::shared_ptr<Device> *dev);

class DummyDev : public Device
{
protected:
    static std::shared_ptr<Device> objRx;
    static std::shared_ptr<Device> objTx;
    DummyDev(struct pal_device *device, std::shared_ptr<ResourceManager> Rm);
public:
    static std::shared_ptr<Device> getInstance(struct pal_device *device,
                                               std::shared_ptr<ResourceManager> Rm);
    virtual ~DummyDev();
};
#endif //DUMMY_DEV_H
