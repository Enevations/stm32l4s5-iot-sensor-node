/** @file test_lps22hb.cpp
 *  @brief Unit tests for the LPS22HB driver against a fake I2C bus.
 *
 * Runs on host (via tests/host_main.cpp) and on target (STM32_SELF_TEST).
 */

#include <cstdio>
#include <cstddef>
#include <cstdint>

#include "fake_i2c_bus.hpp"
#include "lps22hb_sensor.hpp"

namespace {

std::size_t passed = 0;
std::size_t failures = 0;

void record_failure(const char* what, char* error_buffer, std::size_t error_size) {
    if (++failures == 1U && error_buffer != nullptr && error_size > 0U) {
        std::snprintf(error_buffer, error_size, "%s", what);
    }
}

void expect_true(bool condition, const char* what, char* error_buffer, std::size_t error_size) {
    if (condition) {
        ++passed;
    }
    else if (++failures == 1U && error_buffer != nullptr && error_size > 0U) {
        std::snprintf(error_buffer, error_size, "%s", what);
    }
}

void expect_near(float actual,
                 float expected,
                 const char* what,
                 char* error_buffer,
                 std::size_t error_size) {
    const float delta = actual - expected;
    if (delta < -0.05f || delta > 0.05f) {
        record_failure(what, error_buffer, error_size);
    }
    else {
        ++passed;
    }
}

void configure_lps22hb(test::FakeI2cDevice& device) {
    device.at(0x0F) = 0xB1; // WHO_AM_I
}

/// @brief Writes pressure and temperature output registers for one sample.
void set_output_registers(test::FakeI2cDevice& device, int32_t press_raw, int16_t temp_raw) {
    const auto p = static_cast<uint32_t>(press_raw);
    const auto t = static_cast<uint16_t>(temp_raw);
    device.at(0x28) = static_cast<uint8_t>(p & 0xFFU);        // P_OUT LSB
    device.at(0x29) = static_cast<uint8_t>((p >> 8) & 0xFFU); // P_OUT mid
    device.at(0x2A) = static_cast<uint8_t>((p >> 16) & 0xFFU);// P_OUT MSB
    device.at(0x2B) = static_cast<uint8_t>(t & 0xFFU);        // T_OUT LSB
    device.at(0x2C) = static_cast<uint8_t>(t >> 8);           // T_OUT MSB
}

} // namespace

bool run_lps22hb_tests(char* error_buffer, std::size_t error_size) {
    if (error_buffer != nullptr && error_size > 0U) {
        error_buffer[0] = '\0';
    }

    // Init: correct identity succeeds.
    {
        test::FakeI2cBus bus;
        configure_lps22hb(bus.add_device(drivers::Lps22hbSensor::kI2cAddress));
        drivers::Lps22hbSensor sensor(bus);
        expect_true(sensor.init() == drivers::InitStatus::Ok,
                    "init succeeds with valid identity", error_buffer, error_size);
    }

    // Init: wrong WHO_AM_I fails with the specific reason.
    {
        test::FakeI2cBus bus;
        test::FakeI2cDevice& device = bus.add_device(drivers::Lps22hbSensor::kI2cAddress);
        configure_lps22hb(device);
        device.at(0x0F) = 0xFF;
        drivers::Lps22hbSensor sensor(bus);
        expect_true(sensor.init() == drivers::InitStatus::WhoAmIMismatch,
                    "init rejects wrong WHO_AM_I", error_buffer, error_size);
    }

    // Init: absent device (NACK) fails with a bus error.
    {
        test::FakeI2cBus bus;
        drivers::Lps22hbSensor sensor(bus);
        expect_true(sensor.init() == drivers::InitStatus::BusError,
                    "init fails on absent device", error_buffer, error_size);
    }

    // Read: pressure and temperature convert with per-channel valid flags.
    {
        test::FakeI2cBus bus;
        test::FakeI2cDevice& device = bus.add_device(drivers::Lps22hbSensor::kI2cAddress);
        configure_lps22hb(device);
        drivers::Lps22hbSensor sensor(bus);
        expect_true(sensor.init() == drivers::InitStatus::Ok,
                    "init for read test", error_buffer, error_size);

        // 4,150,272 LSB = 1013.25 hPa; 2350 LSB = 23.5 C
        set_output_registers(device, 4150272, 2350);
        const auto sample = sensor.read();
        expect_true(sample.valid_pressure && sample.valid_temperature,
                    "pressure and temperature flagged valid", error_buffer, error_size);
        expect_true(!sample.valid_humidity, "humidity not flagged by pressure sensor",
                    error_buffer, error_size);
        expect_near(sample.pressure_hpa, 1013.25f, "pressure converts to hPa",
                    error_buffer, error_size);
        expect_near(sample.temperature_c, 23.5f, "temperature converts to C",
                    error_buffer, error_size);
    }

    // Read: 24-bit sign extension and negative temperature keep their signs.
    {
        test::FakeI2cBus bus;
        test::FakeI2cDevice& device = bus.add_device(drivers::Lps22hbSensor::kI2cAddress);
        configure_lps22hb(device);
        drivers::Lps22hbSensor sensor(bus);
        expect_true(sensor.init() == drivers::InitStatus::Ok,
                    "init for sign test", error_buffer, error_size);

        // 0xFFF000 sign-extends to -4096 LSB = -1.0 hPa; -1500 LSB = -15.0 C
        set_output_registers(device, -4096, -1500);
        const auto sample = sensor.read();
        expect_near(sample.pressure_hpa, -1.0f, "negative pressure sign-extends",
                    error_buffer, error_size);
        expect_near(sample.temperature_c, -15.0f, "negative temperature",
                    error_buffer, error_size);
    }

    // Read: bus error produces an all-invalid sample, not a crash.
    {
        test::FakeI2cBus bus;
        test::FakeI2cDevice& device = bus.add_device(drivers::Lps22hbSensor::kI2cAddress);
        configure_lps22hb(device);
        drivers::Lps22hbSensor sensor(bus);
        expect_true(sensor.init() == drivers::InitStatus::Ok,
                    "init for bus-error test", error_buffer, error_size);

        bus.remove_device(drivers::Lps22hbSensor::kI2cAddress);
        const auto sample = sensor.read();
        expect_true(!sample.valid_pressure && !sample.valid_temperature,
                    "read after bus loss is invalid", error_buffer, error_size);
    }

    return failures == 0U;
}
