#pragma once

/** @file lsm6dsl_tap.hpp
 *  @brief ST LSM6DSL double-tap recognition driver for the IOT01A board.
 */

#include <cstdint>

#include "bsp/i2c_interface.hpp"
#include "drivers/sensor_interface.hpp"

namespace drivers {

/** @brief Outcome of one tap-source poll. */
enum class TapEvent : uint8_t {
    None,      ///< no tap event pending
    SingleTap, ///< a single-tap event was consumed (tuning aid: the chip sees taps)
    DoubleTap, ///< a double-tap event was consumed from the tap source
    BusError,  ///< the source-register transaction failed
};

/**
 * @brief Controls the LSM6DSL hardware double-tap detector (0x6A on I2C2).
 *
 * Deliberately not an ISensor: tap events are a continuous gesture stream
 * (see the streaming-device note in sensor_interface.hpp), so this class
 * exposes init() plus an event poll instead of an EnvironmentalSample.
 * The chip performs the gesture recognition itself; the driver only
 * configures the detection windows and reads the TAP_SRC register. The
 * configuration is unlatched (LIR = 0), so the DOUBLE_TAP flag self-clears
 * after the Quiet window (~29 ms) and the poller must sample inside it.
 */
class Lsm6dslTap final {
public:
    static constexpr uint8_t kI2cAddress = 0x6A;

    explicit Lsm6dslTap(bsp::II2cBus& bus) noexcept : bus_(bus) {}

    Lsm6dslTap(const Lsm6dslTap&) = delete;
    Lsm6dslTap& operator=(const Lsm6dslTap&) = delete;

    /** @brief 7-bit I2C address, for diagnostics and the POST probe. */
    [[nodiscard]] uint8_t i2c_address() const noexcept { return kI2cAddress; }

    // Verifies WHO_AM_I (0x6A) and enables tap recognition on X/Y/Z with the
    // ST AN5040 double-tap windows (unlatched; the flag self-clears after
    // the Quiet window, so the task polls inside it).
    InitStatus init() noexcept;

    /** @brief Reads the tap source and decodes the most recent event.
     *
     * With the unlatched configuration the SINGLE_TAP/DOUBLE_TAP flags
     * remain visible for roughly the Quiet window (~29 ms) after a
     * recognized gesture, so a 20 ms poll samples them reliably.
     */
    TapEvent read_event() noexcept;

private:
    bsp::II2cBus& bus_;
};

} // namespace drivers
