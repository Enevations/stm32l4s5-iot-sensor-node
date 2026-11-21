#include "drivers/lps22hb_sensor.hpp"

namespace drivers {

namespace {
constexpr uint8_t kRegWhoAmI      = 0x0F;
constexpr uint8_t kExpectedWhoAmI = 0xB1;

constexpr uint8_t kRegCtrl1       = 0x10;

constexpr uint8_t kRegPressOut    = 0x28; // 0x28 (L), 0x29 (M), 0x2A (H)
constexpr uint8_t kRegTempOut     = 0x2B; // 0x2B (L), 0x2C (H)

// MSB set enables register auto-increment on multi-byte reads
constexpr uint8_t kAutoIncrement  = 0x80;

// CTRL_REG1: ODR=1Hz (bits 6:4 = 001), BDU=1 (bit 0), low-pass filters off -> 0x11
constexpr uint8_t kCtrl1Config    = (1U << 4) | 0x01U;

constexpr float kPressureSensitivity    = 4096.0f; // LSB per hPa
constexpr float kTemperatureSensitivity = 100.0f;  // LSB per °C
} // namespace

InitStatus Lps22hbSensor::init() noexcept {
    uint8_t who_am_i = 0;
    if (!bus_.read_reg(kI2cAddress, kRegWhoAmI, who_am_i)) {
        return InitStatus::BusError;
    }
    if (who_am_i != kExpectedWhoAmI) {
        return InitStatus::WhoAmIMismatch;
    }

    return bus_.write_reg(kI2cAddress, kRegCtrl1, kCtrl1Config)
               ? InitStatus::Ok
               : InitStatus::BusError;
}

EnvironmentalSample Lps22hbSensor::read() noexcept {
    EnvironmentalSample sample{};

    // Burst-read 5 bytes starting at 0x28:
    // [P_OUT_L, P_OUT_M, P_OUT_H, T_OUT_L, T_OUT_H]
    uint8_t raw[5]{};
    if (!bus_.read_regs(kI2cAddress, kRegPressOut | kAutoIncrement, raw, sizeof(raw))) {
        return sample;
    }

    // Pressure is a 24-bit two's-complement value: sign-extend into 32 bits.
    uint32_t press_raw = (static_cast<uint32_t>(raw[2]) << 16) |
                         (static_cast<uint32_t>(raw[1]) << 8) |
                         raw[0];
    if ((press_raw & 0x800000U) != 0U) {
        press_raw |= 0xFF000000U;
    }
    sample.pressure_hpa = static_cast<int32_t>(press_raw) / kPressureSensitivity;

    const int16_t temp_raw = static_cast<int16_t>(
        (static_cast<uint16_t>(raw[4]) << 8) | raw[3]);
    sample.temperature_c = static_cast<float>(temp_raw) / kTemperatureSensitivity;

    sample.valid_pressure = true;
    sample.valid_temperature = true;
    return sample;
}

} // namespace drivers
