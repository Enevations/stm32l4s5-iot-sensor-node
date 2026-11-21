/** @file test_lsm6dsl_tap.cpp
 *  @brief Unit tests for the LSM6DSL tap driver against a fake I2C bus.
 *
 * Runs on host (via tests/host_main.cpp) and on target (STM32_SELF_TEST).
 */

#include <cstdio>
#include <cstddef>
#include <cstdint>

#include "fake_i2c_bus.hpp"
#include "drivers/lsm6dsl_tap.hpp"
#include "test_report.hpp"

namespace {

constexpr const char* kSuite = "lsm6dsl_tap";

std::size_t passed = 0;
std::size_t failures = 0;

void expect_true(bool condition, const char* what, char* error_buffer, std::size_t error_size) {
    test::report_case(kSuite, what, condition);
    if (condition) {
        ++passed;
    }
    else if (++failures == 1U && error_buffer != nullptr && error_size > 0U) {
        std::snprintf(error_buffer, error_size, "%s", what);
    }
}

void configure_lsm6dsl(test::FakeI2cDevice& device) {
    device.at(0x0F) = 0x6A; // WHO_AM_I
}

} // namespace

bool run_lsm6dsl_tap_tests(char* error_buffer, std::size_t error_size) {
    if (error_buffer != nullptr && error_size > 0U) {
        error_buffer[0] = '\0';
    }

    // Init: correct identity succeeds.
    {
        test::FakeI2cBus bus;
        configure_lsm6dsl(bus.add_device(drivers::Lsm6dslTap::kI2cAddress));
        drivers::Lsm6dslTap tap(bus);
        expect_true(tap.init() == drivers::InitStatus::Ok,
                    "init succeeds with valid identity", error_buffer, error_size);
    }

    // Init: wrong WHO_AM_I fails with the specific reason.
    {
        test::FakeI2cBus bus;
        test::FakeI2cDevice& device = bus.add_device(drivers::Lsm6dslTap::kI2cAddress);
        configure_lsm6dsl(device);
        device.at(0x0F) = 0x69; // LSM6DS3 identity, not the LSM6DSL one
        drivers::Lsm6dslTap tap(bus);
        expect_true(tap.init() == drivers::InitStatus::WhoAmIMismatch,
                    "init rejects wrong WHO_AM_I", error_buffer, error_size);
    }

    // Init: absent device (NACK) fails with a bus error.
    {
        test::FakeI2cBus bus;
        drivers::Lsm6dslTap tap(bus);
        expect_true(tap.init() == drivers::InitStatus::BusError,
                    "init fails on absent device", error_buffer, error_size);
    }

    // Init: writes the AN5040 double-tap configuration registers.
    {
        test::FakeI2cBus bus;
        test::FakeI2cDevice& device = bus.add_device(drivers::Lsm6dslTap::kI2cAddress);
        configure_lsm6dsl(device);
        drivers::Lsm6dslTap tap(bus);
        expect_true(tap.init() == drivers::InitStatus::Ok,
                    "init for config test", error_buffer, error_size);

        struct ConfigCheck {
            uint8_t reg;
            uint8_t expected;
            const char* what;
        };
        const ConfigCheck checks[] = {
            {0x10, 0x60, "CTRL1_XL: 416 Hz ODR, +-2 g full scale"},
            {0x12, 0x44, "CTRL3_C: BDU and auto-increment set"},
            {0x58, 0x8E, "TAP_CFG: X/Y/Z taps, interrupts enabled, unlatched"},
            {0x59, 0x89, "TAP_THS_6D: threshold 9 with D4D bit"},
            {0x5A, 0x7F, "INT_DUR2: duration 7 (~538 ms), quiet 3, shock 3"},
            {0x5B, 0x80, "WAKE_UP_THS: single and double tap enabled"},
            {0x5E, 0x08, "MD1_CFG: double-tap routed to INT1"},
        };
        for (const ConfigCheck& check : checks) {
            expect_true(device.at(check.reg) == check.expected, check.what,
                        error_buffer, error_size);
        }
    }

    // Read: a double-tap event is decoded from the tap source.
    {
        test::FakeI2cBus bus;
        test::FakeI2cDevice& device = bus.add_device(drivers::Lsm6dslTap::kI2cAddress);
        configure_lsm6dsl(device);
        drivers::Lsm6dslTap tap(bus);
        expect_true(tap.init() == drivers::InitStatus::Ok,
                    "init for event test", error_buffer, error_size);

        device.at(0x1C) = 0x10; // TAP_SRC DOUBLE_TAP bit
        expect_true(tap.read_event() == drivers::TapEvent::DoubleTap,
                    "double-tap event decoded", error_buffer, error_size);

        // Axis and TAP_IA bits accompany the double-tap event.
        device.at(0x1C) = 0x5F; // TAP_IA | SINGLE_TAP | DOUBLE_TAP | Z/Y/X
        expect_true(tap.read_event() == drivers::TapEvent::DoubleTap,
                    "double-tap wins over single-tap bits", error_buffer, error_size);

        // Single tap without the double-tap bit decodes as a single tap.
        device.at(0x1C) = 0x27; // single tap on X/Y/Z, no double-tap bit
        expect_true(tap.read_event() == drivers::TapEvent::SingleTap,
                    "single-tap bits decode as single tap", error_buffer, error_size);

        device.at(0x1C) = 0x00;
        expect_true(tap.read_event() == drivers::TapEvent::None,
                    "no event when source is clear", error_buffer, error_size);
    }

    // Read: bus error is reported instead of a spurious event.
    {
        test::FakeI2cBus bus;
        test::FakeI2cDevice& device = bus.add_device(drivers::Lsm6dslTap::kI2cAddress);
        configure_lsm6dsl(device);
        drivers::Lsm6dslTap tap(bus);
        expect_true(tap.init() == drivers::InitStatus::Ok,
                    "init for bus-error test", error_buffer, error_size);

        bus.remove_device(drivers::Lsm6dslTap::kI2cAddress);
        expect_true(tap.read_event() == drivers::TapEvent::BusError,
                    "read_event reports bus error", error_buffer, error_size);
    }

    return failures == 0U;
}
