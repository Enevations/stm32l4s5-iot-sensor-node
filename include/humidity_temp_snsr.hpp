#pragma once

#include <cstdint>

#include "i2c_interface.hpp"

namespace drivers {

struct EnvironmentalSample {
    float temperature_c{0.0f};
    float humidity_pct{0.0f};
    bool valid{false};
};

/** @brief Why an init() attempt failed, for diagnostics and retry logic. */
enum class InitStatus : uint8_t {
    Ok,
    BusError,           ///< I2C transaction failed (device absent or bus fault)
    WhoAmIMismatch,     ///< device answered but with an unexpected identity
    InvalidCalibration, ///< degenerate calibration coefficients (division guard)
};

class HumidityTempSnsr {
public:
    static constexpr uint8_t kI2cAddress = 0x5F; // HTS221 7-bit I2C address

    explicit HumidityTempSnsr(bsp::II2cBus& bus) noexcept : bus_(bus) {}

    HumidityTempSnsr(const HumidityTempSnsr&) = delete;
    HumidityTempSnsr& operator=(const HumidityTempSnsr&) = delete;

    // Verifies WHO_AM_I (0xBC), loads factory calibration trims, and configures 1 Hz BDU mode
    InitStatus init() noexcept;

    // Reads raw ADC registers and applies linear interpolation calibration
    EnvironmentalSample read() noexcept;

private:
    bool load_calibration() noexcept;

    bsp::II2cBus& bus_;

    // Factory calibration coefficients (loaded once during init)
    float h0_rh_{0.0f};
    float h1_rh_{0.0f};
    int16_t h0_t0_out_{0};
    int16_t h1_t0_out_{0};

    float t0_degc_{0.0f};
    float t1_degc_{0.0f};
    int16_t t0_out_{0};
    int16_t t1_out_{0};
};

} // namespace drivers