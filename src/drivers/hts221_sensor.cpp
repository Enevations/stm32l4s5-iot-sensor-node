#include "drivers/hts221_sensor.hpp"

namespace drivers {

namespace {
constexpr uint8_t kRegWhoAmI      = 0x0F;
constexpr uint8_t kExpectedWhoAmI = 0xBC;

constexpr uint8_t kRegCtrl1       = 0x20;
constexpr uint8_t kRegHumidityOut = 0x28; // 0x28 (L), 0x29 (H)
constexpr uint8_t kRegTempOut     = 0x2A; // 0x2A (L), 0x2B (H)

// Calibration registers (0x30 to 0x3F)
constexpr uint8_t kRegCalibStart  = 0x30;

// MSB set enables register auto-increment on multi-byte reads
constexpr uint8_t kAutoIncrement  = 0x80;

constexpr int16_t combine_i16(uint8_t lsb, uint8_t msb) noexcept {
    return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
}
} // namespace

InitStatus Hts221Sensor::init() noexcept {
    uint8_t who_am_i = 0;
    if (!bus_.read_reg(kI2cAddress, kRegWhoAmI, who_am_i)) {
        return InitStatus::BusError;
    }
    if (who_am_i != kExpectedWhoAmI) {
        return InitStatus::WhoAmIMismatch;
    }

    if (!load_calibration()) {
        return InitStatus::InvalidCalibration;
    }

    // CTRL_REG1: PD=1 (Active), BDU=1 (Block Data Update), ODR=1Hz -> 0x85
    constexpr uint8_t kCtrl1Config = (1U << 7) | (1U << 2) | 0x01U;
    return bus_.write_reg(kI2cAddress, kRegCtrl1, kCtrl1Config)
               ? InitStatus::Ok
               : InitStatus::BusError;
}

bool Hts221Sensor::load_calibration() noexcept {
    uint8_t cal[16]{};
    if (!bus_.read_regs(kI2cAddress, kRegCalibStart | kAutoIncrement, cal, sizeof(cal))) {
        return false;
    }

    // Humidity calibration points (stored as 2x %RH)
    h0_rh_ = static_cast<float>(cal[0]) / 2.0f; // 0x30: H0_rH_x2
    h1_rh_ = static_cast<float>(cal[1]) / 2.0f; // 0x31: H1_rH_x2
    h0_t0_out_ = combine_i16(cal[6], cal[7]);   // 0x36..0x37: H0_T0_OUT
    h1_t0_out_ = combine_i16(cal[10], cal[11]); // 0x3A..0x3B: H1_T0_OUT

    // Temperature calibration points (10-bit unsigned stored across 0x32, 0x33, and 0x35, divided by 8)
    const uint16_t t0_msb = static_cast<uint16_t>(cal[5] & 0x03U) << 8;
    const uint16_t t1_msb = static_cast<uint16_t>(cal[5] & 0x0CU) << 6;
    t0_degc_ = static_cast<float>(t0_msb | cal[2]) / 8.0f;
    t1_degc_ = static_cast<float>(t1_msb | cal[3]) / 8.0f;

    t0_out_ = combine_i16(cal[12], cal[13]); // 0x3C..0x3D: T0_OUT
    t1_out_ = combine_i16(cal[14], cal[15]); // 0x3E..0x3F: T1_OUT

    // Division guard: degenerate calibration would produce inf/nan slopes.
    if (h1_t0_out_ == h0_t0_out_ || t1_out_ == t0_out_) {
        return false;
    }

    // Interpolation slopes are constant per device: compute them once here
    // instead of on every read().
    h_slope_ = (h1_rh_ - h0_rh_) / static_cast<float>(h1_t0_out_ - h0_t0_out_);
    t_slope_ = (t1_degc_ - t0_degc_) / static_cast<float>(t1_out_ - t0_out_);
    return true;
}

EnvironmentalSample Hts221Sensor::read() noexcept {
    EnvironmentalSample sample{};

    // Burst-read 4 bytes starting at 0x28: [H_OUT_L, H_OUT_H, T_OUT_L, T_OUT_H]
    uint8_t raw[4]{};
    if (!bus_.read_regs(kI2cAddress, kRegHumidityOut | kAutoIncrement, raw, sizeof(raw))) {
        return sample;
    }

    const int16_t h_out = combine_i16(raw[0], raw[1]);
    const int16_t t_out = combine_i16(raw[2], raw[3]);

    // Linear interpolation for Relative Humidity (%)
    sample.humidity_pct = h0_rh_ + static_cast<float>(h_out - h0_t0_out_) * h_slope_;

    if (sample.humidity_pct < 0.0f) sample.humidity_pct = 0.0f;
    if (sample.humidity_pct > 100.0f) sample.humidity_pct = 100.0f;

    // Linear interpolation for Temperature (°C)
    sample.temperature_c = t0_degc_ + static_cast<float>(t_out - t0_out_) * t_slope_;

    sample.valid_temperature = true;
    sample.valid_humidity = true;
    return sample;
}

} // namespace drivers
