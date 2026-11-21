/** @file test_post_check.cpp
 *  @brief Unit tests for the I2C power-on self-test probe logic.
 *
 * Runs on host (via tests/host_main.cpp) and on target (STM32_SELF_TEST).
 */

#include <cstdio>
#include <cstddef>
#include <cstdint>

#include "fake_i2c_bus.hpp"
#include "bsp/post_check.hpp"
#include "test_report.hpp"

namespace {

constexpr const char* kSuite = "post_check";

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

} // namespace

bool run_post_check_tests(char* error_buffer, std::size_t error_size) {
    if (error_buffer != nullptr && error_size > 0U) {
        error_buffer[0] = '\0';
    }

    // All four onboard devices present: POST passes.
    {
        test::FakeI2cBus bus;
        for (const uint8_t address : bsp::kExpectedI2CDevices) {
            bus.add_device(address);
        }
        const auto result = bsp::run_post_check(bus, bsp::kExpectedI2CDevices.data(),
                                                bsp::kExpectedI2CDevices.size());
        expect_true(result.all_found, "POST passes with all devices present",
                    error_buffer, error_size);
    }

    // First device missing: reported with its address.
    {
        test::FakeI2cBus bus;
        bus.add_device(0x5F);
        bus.add_device(0x6A);
        bus.add_device(0x5D);
        const auto result = bsp::run_post_check(bus, bsp::kExpectedI2CDevices.data(),
                                                bsp::kExpectedI2CDevices.size());
        expect_true(!result.all_found, "POST fails when a device is missing",
                    error_buffer, error_size);
        expect_true(result.missing_address == 0x1E, "first missing address reported",
                    error_buffer, error_size);
    }

    // Middle device missing: probe stops at the failure.
    {
        test::FakeI2cBus bus;
        bus.add_device(0x1E);
        bus.add_device(0x5D); // later device present does not matter
        const auto result = bsp::run_post_check(bus, bsp::kExpectedI2CDevices.data(),
                                                bsp::kExpectedI2CDevices.size());
        expect_true(!result.all_found, "POST fails when a mid-list device is missing",
                    error_buffer, error_size);
        expect_true(result.missing_address == 0x5F, "HTS221 address reported when absent",
                    error_buffer, error_size);
    }

    // Empty device list trivially passes.
    {
        test::FakeI2cBus bus;
        const auto result = bsp::run_post_check(bus, nullptr, 0);
        expect_true(result.all_found, "empty POST list passes", error_buffer, error_size);
    }

    return failures == 0U;
}
