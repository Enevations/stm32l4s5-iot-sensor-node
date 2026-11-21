#pragma once

#include <cstdint>

#include "bsp/i2c_interface.hpp"
#include "drivers/sensor_interface.hpp"

namespace drivers {

/// @brief ST LPS22HB barometric pressure sensor (0x5D on the IOT01A I2C2 bus).
class Lps22hbSensor final : public ISensor {
public:
    static constexpr uint8_t kI2cAddress = 0x5D;

    explicit Lps22hbSensor(bsp::II2cBus& bus) noexcept : bus_(bus) {}

    Lps22hbSensor(const Lps22hbSensor&) = delete;
    Lps22hbSensor& operator=(const Lps22hbSensor&) = delete;

    [[nodiscard]] uint8_t i2c_address() const noexcept override { return kI2cAddress; }

    // Verifies WHO_AM_I (0xB1) and configures 1 Hz BDU mode with filters off
    InitStatus init() noexcept override;

    // Reads the 24-bit pressure (hPa) and 16-bit die temperature (C)
    EnvironmentalSample read() noexcept override;

private:
    bsp::II2cBus& bus_;
};

} // namespace drivers
