/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once
#include <mutex>
#include <optional>
#include <string>

#include "Status.h"

namespace device::bt {

/**
 * @brief Bluetooth Hands-Free Profile (HFP) Abstraction.
 *
 * This class represents the Bluetooth Hands-Free Profile (HFP), which supports either playback
 * (OUTPUT) or recording (INPUT). Only one instance of the HFP profile exists at a time.
 *
 * It operates as a simple state machine with the following states:
 *   CLOSED <--> OPENED -> STARTED <--> STOPPED -> CLOSED
 *
 *   +--------+     +--------+     +---------+     +---------+
 *   | CLOSED | <--> | OPENED | --> | STARTED | <--> | STOPPED |
 *   +--------+     +--------+     +---------+     +---------+
 *        ^                                           |
 *        |-------------------------------------------|
 *
 * Important:** If `X` number of `start()` calls succeed, then exactly `X` number of `stop()`
 * calls are required before invoking `close()`. Failure to do so may leave the profile in an
 * inconsistent state.
 *
 * When the HFP profile is used simultaneously for both playback and recording, the codec must be
 * the same in both directions. That is, the data must be encoded and decoded using the same codec
 * for both output and input. This profile expects such behavior.
 *
 * Stream: A stream acts as a client to the HFP profile.
 *
 * The number of streams connected to this profile is tracked and respected.
 * Each call to the start or stop API increments or decrements the count of active streams,
 * respectively. check more details on 'mStreamCounter'.
 */
class HFPProfile {
  public:
    class Codec {
      public:
        enum class Type {
            INVALID = 1,
            LC3,
            MSBC,
            CVSD,
            APTX_SPEECH,
        };
        bool operator==(const Codec& other) const noexcept;

        Type mType = Type::INVALID;
        /**
         * @brief whether Noise reduction and echo cancellation(NREC) enabled
         */
        using NREC = bool;
        NREC mIsNRECEnabled = false;
    };
    static std::string toString(const Codec::Type type) noexcept;
    static std::string toString(const Codec& codec) noexcept;

    /**
     * @brief Returns the singleton instance of HFPProfile.
     */
    static HFPProfile* getInstance() noexcept;

    /**
     * @brief Requests the BT Host service to open the profile.
     * State Must be in OPENED.
     * @return true if successful; false otherwise.
     */
    Status open() noexcept;

    /**
     * @brief Requests the BT Host service to start the profile.
     * State Must be in OPENED or STARTED.
     * @return true if successful; false otherwise.
     */
    Status start() noexcept;

    /**
     * @brief whether any HFP is still active or not
     * @return true if HFP is in STARTED state, false otherwise
     */
    bool isActive() noexcept;

    /**
     * @brief Requests the BT Host service to stop the profile.
     * State must be in STARTED.
     * @return true if successful; false otherwise.
     */
    Status stop() noexcept;

    /**
     * @brief Requests the BT Host service to close the profile.
     * State must be in STOPPED.
     * @return true if successful; false otherwise.
     */
    Status close() noexcept;
    /**
     * @brief only in STARTED state, we query the codec information.
     * During active HFP session, codec doesn't change.
     */
    std::optional<Codec> getCodec() noexcept;

  private:
    HFPProfile();
    ~HFPProfile() = default;
    HFPProfile(const HFPProfile&) = delete;
    HFPProfile& operator=(const HFPProfile&) = delete;
    HFPProfile(HFPProfile&&) = delete;
    HFPProfile& operator=(HFPProfile&&) = delete;

    enum class State : uint8_t {
        CLOSED = 1,
        OPENED,
        STARTED,
        STOPPED,
    };
    static std::string toString(const State state) noexcept;

    std::mutex mLock;
    State mState = State::CLOSED;
    uint32_t mStreamCounter = 0;
    std::optional<Codec> mCodec = std::nullopt;
};

}  // namespace device::bt
