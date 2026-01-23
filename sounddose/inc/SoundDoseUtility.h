/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SOUND_DOSE_UTILITY_H
#define SOUND_DOSE_UTILITY_H

#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>
#include <atomic>

#include "Device.h"
#include "PalDefs.h"
#include "apm_api.h"

class ResourceManager;

class SoundDoseUtility {
  public:
    /**
     * @brief constructor for  sound dose calculations for a given device.
     *
     * @param devObj Pointer to the device object.
     * @param deviceId pal_device_id_t of the device.
     *
     */
    SoundDoseUtility(Device *devObj, const struct pal_device device);
    ~SoundDoseUtility();

    void startComputation();
    void stopComputation();
    void setDevice(struct pal_device palDevice) { mPalDevice = palDevice; }

    // handle sound dose info
    static void handleSoundDoseCallback(uint64_t hdl, uint32_t event_id, void *event_data,
                                        uint32_t event_size);

    static bool isEnabled(pal_device_id_t deviceId);
    static bool supportsSoundDose(pal_device_id_t deviceId);

    struct pal_device getPalDevice() {
        return mPalDevice;
    }

  private:
    std::shared_ptr<ResourceManager> mResourceManager;
    /**< Associated pal_device for which SoundDose is to be computed.*/
    struct pal_device mPalDevice;
    Device *mDevObj;
    bool mIsEnabled = false;
    std::vector<int> mPcmDevIdRx;
    struct mixer *mVirtualMixer;
    struct pcm *mRxPcm;

    /* Function to retreive any remaining MEL values cached in SPF before closing. */
    void getSoundDoseMelValues();

    void startComputationInternal();
    void stopComputationInternal();

    pal_global_callback getCallback();
    uint64_t getCookie();

    void onSoundDoseEvent(pal_sound_dose_info_t *callbackData);
    // wrapper api for SessionAlsaUtils::registerMixerEvent to register/unregister
    // sound dose events.
    int registerMixerEvent(const bool state, const std::string backendName);

    SoundDoseUtility(const SoundDoseUtility &) = delete;
    SoundDoseUtility &operator=(const SoundDoseUtility &) = delete;

    // don't enable unless want to test standalone
    bool mUseSimulation = false;
    std::atomic<bool> mSimulationRunning{false};
    std::thread mSimulationThread;
    std::atomic<int> mRefCount{0};

    void simulationThreadLoop();
    void startSimulation();
    void stopSimulation();
};

#endif // SOUND_DOSE_UTILITY_H
