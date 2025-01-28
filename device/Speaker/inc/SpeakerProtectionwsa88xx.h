/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#include "Device.h"
#include "SpeakerProtection.h"

class SpeakerProtectionwsa883x : public SpeakerProtection
{
protected :
    static bool viTxSetupThrdCreated;

private :

public:
    static std::thread viTxSetupThread;
    std::mutex deviceMutex;
    static std::mutex calibrationMutex;
    int spkrStartCalibration() override;
    int viTxSetupThreadLoop();
    SpeakerProtectionwsa883x(struct pal_device *device,
                      std::shared_ptr<ResourceManager> Rm);
    ~SpeakerProtectionwsa883x();

    int32_t spkrProtProcessingMode(bool flag) override;

    void updateCpsCustomPayload(int miid);
    int getCpsDevNumber(std::string mixer);
};

class SpeakerProtectionwsa884x : public SpeakerProtection
{
protected :
    static bool viTxSetupThrdCreated;

private :

public:
    static std::thread viTxSetupThread;
    std::mutex deviceMutex;
    static std::mutex calibrationMutex;
    int spkrStartCalibration() override;
    int viTxSetupThreadLoop();
    SpeakerProtectionwsa884x(struct pal_device *device,
                      std::shared_ptr<ResourceManager> Rm);
    ~SpeakerProtectionwsa884x();

    int32_t spkrProtProcessingMode(bool flag) override;
};

class SpeakerProtectionwsa885x : public SpeakerProtection
{
protected :
    static bool viTxSetupThrdCreated;

private :
    int wsaTemp;

public:
    static std::thread viTxSetupThread;
    std::mutex deviceMutex;
    int getSpeakerTemperature(int spkr_pos) override;
    static std::mutex calibrationMutex;
    int spkrStartCalibration() override;
    int viTxSetupThreadLoop();
    SpeakerProtectionwsa885x(struct pal_device *device,
                      std::shared_ptr<ResourceManager> Rm);
    ~SpeakerProtectionwsa885x();

    int32_t spkrProtProcessingMode(bool flag) override;
};

class SpeakerProtectionwsa885xI2s : public SpeakerProtection
{
protected :
    static bool viTxSetupThrdCreated;

private :
    int i2sChannels;

public:
    static std::thread viTxSetupThread;
    std::mutex deviceMutex;
    int getSpeakerTemperature(int spkr_pos) override;
    static std::mutex calibrationMutex;
    int spkrStartCalibration() override;
    int viTxSetupThreadLoop();
    SpeakerProtectionwsa885xI2s(struct pal_device *device,
                      std::shared_ptr<ResourceManager> Rm);
    ~SpeakerProtectionwsa885xI2s();

    int32_t spkrProtProcessingMode(bool flag) override;
};
