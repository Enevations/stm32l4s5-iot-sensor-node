#include "drivers/lsm6dsl_tap.hpp"

namespace drivers {

namespace {
constexpr uint8_t kRegWhoAmI      = 0x0F;
constexpr uint8_t kExpectedWhoAmI = 0x6A;

constexpr uint8_t kRegCtrl1Xl     = 0x10;
constexpr uint8_t kRegCtrl3C      = 0x12;
constexpr uint8_t kRegTapSrc      = 0x1C;
constexpr uint8_t kRegTapCfg      = 0x58;
constexpr uint8_t kRegTapThs6D    = 0x59;
constexpr uint8_t kRegIntDur2     = 0x5A;
constexpr uint8_t kRegWakeUpThs   = 0x5B;
constexpr uint8_t kRegMd1Cfg      = 0x5E;

// CTRL3_C: BDU (bit 6) + IF_INC (bit 2) -> deterministic register reads.
constexpr uint8_t kCtrl3CConfig   = 0x44;
// CTRL1_XL: ODR 416 Hz (0110b, the tap-recommended rate) + FS +-2 g -> 0x60.
constexpr uint8_t kCtrl1XlConfig  = 0x60;

// TAP_SRC bit 4: DOUBLE_TAP. Bit 5: SINGLE_TAP.
constexpr uint8_t kTapSrcDouble   = 0x10;
constexpr uint8_t kTapSrcSingle   = 0x20;

// Double-tap configuration per ST AN5040 section 5.5.5.
// TAP_CFG: interrupts enable (bit 7), tap on X/Y/Z (bits 3:1), LIR (bit 0)
// -> 0x8E. LIR is deliberately 0: with latched mode plus the task's
// continuous 20 ms TAP_SRC polling, the second tap of a gesture never
// paired on this board; unlatched (the app-note example configuration)
// works - the DOUBLE_TAP flag then self-clears after the Quiet window
// (~29 ms), which the poll samples inside that window.
constexpr uint8_t kTapCfgConfig    = 0x8E;
// TAP_THS_6D: threshold 9 (~600 mg at +-2 g) + D4D bit -> 0x89. Below the
// AN5040 example value (12): a desk tap must exceed the slope threshold, and
// +-2 g clipping makes firm taps saturate before building a larger slope.
constexpr uint8_t kTapThs6DConfig  = 0x89;
// INT_DUR2 at 416 Hz: duration 7 (~538 ms max gap between taps), quiet 3
// (~29 ms), shock 3 (~58 ms) -> 0x7F. Validated on hardware; the maximum
// DUR window (15, ~1153 ms) was tried and paired worse in practice.
constexpr uint8_t kIntDur2Config   = 0x7F;
// WAKE_UP_THS: SINGLE_DOUBLE_TAP = 1 enables double-tap recognition -> 0x80.
constexpr uint8_t kWakeUpThsConfig = 0x80;
// MD1_CFG: route the double-tap event to INT1 so the latch engages -> 0x08.
constexpr uint8_t kMd1CfgConfig    = 0x08;
} // namespace

InitStatus Lsm6dslTap::init() noexcept {
    uint8_t who_am_i = 0;
    if (!bus_.read_reg(kI2cAddress, kRegWhoAmI, who_am_i)) {
        return InitStatus::BusError;
    }
    if (who_am_i != kExpectedWhoAmI) {
        return InitStatus::WhoAmIMismatch;
    }

    struct ConfigStep {
        uint8_t reg;
        uint8_t value;
    };
    constexpr ConfigStep kConfigSteps[] = {
        {kRegCtrl3C,    kCtrl3CConfig},
        {kRegCtrl1Xl,   kCtrl1XlConfig},   // accelerometer on: slope data source
        {kRegTapCfg,    kTapCfgConfig},
        {kRegTapThs6D,  kTapThs6DConfig},
        {kRegIntDur2,   kIntDur2Config},
        {kRegWakeUpThs, kWakeUpThsConfig},
        {kRegMd1Cfg,    kMd1CfgConfig},
    };
    for (const ConfigStep& step : kConfigSteps) {
        if (!bus_.write_reg(kI2cAddress, step.reg, step.value)) {
            return InitStatus::BusError;
        }
    }
    return InitStatus::Ok;
}

TapEvent Lsm6dslTap::read_event() noexcept {
    uint8_t src = 0;
    if (!bus_.read_reg(kI2cAddress, kRegTapSrc, src)) {
        return TapEvent::BusError;
    }
    if ((src & kTapSrcDouble) != 0U) {
        return TapEvent::DoubleTap;
    }
    if ((src & kTapSrcSingle) != 0U) {
        return TapEvent::SingleTap;
    }
    return TapEvent::None;
}

} // namespace drivers
