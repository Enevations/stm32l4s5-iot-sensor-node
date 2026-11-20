#pragma once

#include <cstdint>

#include "i2c_interface.hpp"
#include "sensor_interface.hpp"

namespace drivers {

/// @brief ST HTS221 humidity/temperature sensor (0x5F on the IOT01A I2C2 bus).
class Hts221Sensor final : public ISensor {
public:
    static constexpr uint8_t kI2cAddress = 0x5F;

    explicit Hts221Sensor(bsp::II2cBus& bus) noexcept : bus_(bus) {}

    Hts221Sensor(const Hts221Sensor&) = delete;
    Hts221Sensor& operator=(const Hts221Sensor&) = delete;

    [[nodiscard]] uint8_t i2c_address() const noexcept override { return kI2cAddress; }

    // Verifies WHO_AM_I (0xBC), loads factory calibration trims, and configures 1 Hz BDU mode
    InitStatus init() noexcept override;

    // Reads raw ADC registers and applies linear-interpolation calibration
    EnvironmentalSample read() noexcept override;

private:
    bool load_calibration() noexcept;

    bsp::II2cBus& bus_;

    // Factory calibration coefficients and interpolation slopes (set in init)
    float h0_rh_{0.0f};
    float h1_rh_{0.0f};
    int16_t h0_t0_out_{0};
    int16_t h1_t0_out_{0};

    float t0_degc_{0.0f};
    float t1_degc_{0.0f};
    int16_t t0_out_{0};
    int16_t t1_out_{0};

    float h_slope_{0.0f};
    float t_slope_{0.0f};
};

} // namespace drivers
