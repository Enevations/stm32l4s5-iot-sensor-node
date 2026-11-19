#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "i2c_interface.hpp"

namespace bsp {

/// 7-bit addresses of the onboard sensors, in probe order.
inline constexpr std::array<uint8_t, 4> kExpectedI2CDevices{ 0x1E, // LIS3MDL magnetometer
                                                        0x5F, // HTS221 humidity/temp sensor
                                                        0x6A, // LSM6DSL accelerometer/gyro
                                                        0x5D  // LPS22HB barometric pressure sensor
                                                    };

/**
 * @brief Outcome of the power-on self-test device probe.
 *
 * When a device does not acknowledge, missing_address carries the first
 * failing address so the caller can log exactly which one is absent.
 */
struct PostResult {
    bool all_found{true};
    uint8_t missing_address{0};
};

/**
 * @brief Probes each expected device address on the bus.
 *
 * Pure logic over the II2cBus abstraction: no HAL, no UART, no global state,
 * fully unit-testable with a fake bus.
 *
 * @param bus Initialized bus abstraction to probe on.
 * @param addresses Expected 7-bit device addresses.
 * @param count Number of addresses.
 * @return PostResult::all_found == true when every address ACKs.
 */
inline PostResult run_post_check(II2cBus& bus,
                                 const uint8_t* addresses,
                                 std::size_t count) noexcept {
    PostResult result;
    for (std::size_t i = 0; i < count; ++i) {
        if (!bus.is_device_ready(addresses[i])) {
            result.all_found = false;
            result.missing_address = addresses[i];
            break;
        }
    }
    return result;
}

} // namespace bsp
