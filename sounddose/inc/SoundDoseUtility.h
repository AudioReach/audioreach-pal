/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

//Header file for Sound Dose impl in PAL.

#ifndef SOUND_DOSE_UTILITY_H
#define SOUND_DOSE_UTILITY_H

#include <vector>
#include <memory>
#include <mutex>
#include <unordered_map>
#include "PalDefs.h"
#include "apm_api.h"
#include "Device.h"

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
    SoundDoseUtility(Device *devObj, pal_device_id_t deviceId);
    ~SoundDoseUtility();

    int startSoundDoseComputation();
    void stopSoundDoseComputation();

    //handle sound dose info
    static void handleSoundDoseCallback(uint64_t hdl, uint32_t event_id,
                                        void *event_data, uint32_t event_size);


protected:

    static std::shared_ptr<ResourceManager> rm;
    pal_device_id_t mAssociatedDevId;   /**< Associated device id for which SoundDose is to be computed.*/
    Device *mDevObj;

    std::vector<int> pcmDevIdRx;
    struct pal_device mDeviceAttr;
    struct mixer *virtMixer;
    struct pcm *rxPcm;

    /* Function to retreive any remaining MEL values cached in SPF before closing. */
    void getSoundDoseMelValues();

private:

    SoundDoseUtility(const SoundDoseUtility&) = delete;
    SoundDoseUtility& operator=(const SoundDoseUtility&) = delete;
};

#endif //SOUND_DOSE_UTILITY_H
