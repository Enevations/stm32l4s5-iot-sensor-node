/** @file test_hts221.cpp
 *  @brief Unit tests for the HTS221 driver against a fake I2C bus.
 *
 * Uses a synthetic calibration image with hand-computed reference values:
 *   H0 = 32 %RH at H0_T0_OUT = 0,  H1 = 100 %RH at H1_T0_OUT = 1600
 *   T0 = 0 °C  at T0_OUT   = 0,    T1 = 100 °C  at T1_OUT   = 800
 *
 * Runs on host (via tests/host_main.cpp) and on target (STM32_SELF_TEST).
 */

#include <cstdio>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "fake_i2c_bus.hpp"
#include "hts221_sensor.hpp"
#include "test_report.hpp"

namespace {

constexpr const char* kSuite = "hts221";

std::size_t passed = 0;
std::size_t failures = 0;

void record_failure(const char* what, char* error_buffer, std::size_t error_size) {
    if (++failures == 1U && error_buffer != nullptr && error_size > 0U) {
        std::snprintf(error_buffer, error_size, "%s", what);
    }
}

void expect_true(bool condition, const char* what, char* error_buffer, std::size_t error_size) {
    test::report_case(kSuite, what, condition);
    if (condition) {
        ++passed;
    }
    else {
        record_failure(what, error_buffer, error_size);
    }
}

void expect_near(float actual,
                 float expected,
                 const char* what,
                 char* error_buffer,
                 std::size_t error_size) {
    const float delta = actual - expected;
    const bool ok = delta >= -0.05f && delta <= 0.05f;
    test::report_case(kSuite, what, ok);
    if (ok) {
        ++passed;
    }
    else {
        record_failure(what, error_buffer, error_size);
    }
}

/// @brief Writes the synthetic calibration image and identity into the fake.
void configure_hts221(test::FakeI2cDevice& device) {
    device.at(0x0F) = 0xBC;             // WHO_AM_I

    device.at(0x30) = 64;               // H0_rH_x2 -> 32 %RH
    device.at(0x31) = 200;              // H1_rH_x2 -> 100 %RH
    device.at(0x32) = 0x00;             // T0_deg LSB (msb bits in 0x35)
    device.at(0x33) = 0x20;             // T1_deg LSB (32)
    device.at(0x35) = 0x0C;             // T0 msb = 0, T1 msb = 3 -> T1 = 100 C

    device.at(0x36) = 0x00;             // H0_T0_OUT LSB
    device.at(0x37) = 0x00;             // H0_T0_OUT MSB
    device.at(0x3A) = 0x40;             // H1_T0_OUT LSB (1600)
    device.at(0x3B) = 0x06;             // H1_T0_OUT MSB
    device.at(0x3C) = 0x00;             // T0_OUT LSB
    device.at(0x3D) = 0x00;             // T0_OUT MSB
    device.at(0x3E) = 0x20;             // T1_OUT LSB (800)
    device.at(0x3F) = 0x03;             // T1_OUT MSB
}

void set_output_registers(test::FakeI2cDevice& device, int16_t h_out, int16_t t_out) {
    const auto h = static_cast<uint16_t>(h_out);
    const auto t = static_cast<uint16_t>(t_out);
    device.at(0x28) = static_cast<uint8_t>(h & 0xFFU); // H_OUT LSB
    device.at(0x29) = static_cast<uint8_t>(h >> 8);    // H_OUT MSB
    device.at(0x2A) = static_cast<uint8_t>(t & 0xFFU); // T_OUT LSB
    device.at(0x2B) = static_cast<uint8_t>(t >> 8);    // T_OUT MSB
}

} // namespace

bool run_hts221_tests(char* error_buffer, std::size_t error_size) {
    if (error_buffer != nullptr && error_size > 0U) {
        error_buffer[0] = '\0';
    }

    // Init: correct identity and sane calibration succeed.
    {
        test::FakeI2cBus bus;
        configure_hts221(bus.add_device(drivers::Hts221Sensor::kI2cAddress));
        drivers::Hts221Sensor sensor(bus);
        expect_true(sensor.init() == drivers::InitStatus::Ok,
                    "init succeeds with valid image", error_buffer, error_size);
    }

    // Init: wrong WHO_AM_I fails with a specific reason.
    {
        test::FakeI2cBus bus;
        test::FakeI2cDevice& device = bus.add_device(drivers::Hts221Sensor::kI2cAddress);
        configure_hts221(device);
        device.at(0x0F) = 0xFF;
        drivers::Hts221Sensor sensor(bus);
        expect_true(sensor.init() == drivers::InitStatus::WhoAmIMismatch,
                    "init rejects wrong WHO_AM_I", error_buffer, error_size);
    }

    // Init: missing device (NACK) fails with a bus error.
    {
        test::FakeI2cBus bus;
        drivers::Hts221Sensor sensor(bus);
        expect_true(sensor.init() == drivers::InitStatus::BusError,
                    "init fails on absent device", error_buffer, error_size);
    }

    // Init: degenerate calibration (division guard) fails with a specific reason.
    {
        test::FakeI2cBus bus;
        test::FakeI2cDevice& device = bus.add_device(drivers::Hts221Sensor::kI2cAddress);
        configure_hts221(device);
        device.at(0x3E) = 0x00; // T1_OUT = T0_OUT = 0
        device.at(0x3F) = 0x00;
        drivers::Hts221Sensor sensor(bus);
        expect_true(sensor.init() == drivers::InitStatus::InvalidCalibration,
                    "init rejects degenerate calibration", error_buffer, error_size);
    }

    // Read: linear interpolation at calibration midpoints.
    {
        test::FakeI2cBus bus;
        test::FakeI2cDevice& device = bus.add_device(drivers::Hts221Sensor::kI2cAddress);
        configure_hts221(device);
        drivers::Hts221Sensor sensor(bus);
        expect_true(sensor.init() == drivers::InitStatus::Ok, "init for read test", error_buffer, error_size);

        set_output_registers(device, 800, 400); // halfway between both points
        const auto sample = sensor.read();
        expect_true(sample.valid_temperature && sample.valid_humidity, "sample at midpoint is valid", error_buffer, error_size);
        expect_near(sample.humidity_pct, 66.0f, "humidity midpoint 66 %RH", error_buffer, error_size);
        expect_near(sample.temperature_c, 50.0f, "temperature midpoint 50 C", error_buffer, error_size);

        // Calibration endpoints reproduce the calibration values exactly.
        set_output_registers(device, 0, 0);
        const auto low = sensor.read();
        expect_near(low.humidity_pct, 32.0f, "humidity at H0", error_buffer, error_size);
        expect_near(low.temperature_c, 0.0f, "temperature at T0", error_buffer, error_size);

        set_output_registers(device, 1600, 800);
        const auto high = sensor.read();
        expect_near(high.humidity_pct, 100.0f, "humidity at H1", error_buffer, error_size);
        expect_near(high.temperature_c, 100.0f, "temperature at T1", error_buffer, error_size);
    }

    // Read: negative temperature keeps its sign.
    {
        test::FakeI2cBus bus;
        test::FakeI2cDevice& device = bus.add_device(drivers::Hts221Sensor::kI2cAddress);
        configure_hts221(device);
        drivers::Hts221Sensor sensor(bus);
        expect_true(sensor.init() == drivers::InitStatus::Ok, "init for negative test", error_buffer, error_size);

        set_output_registers(device, 400, -400);
        const auto sample = sensor.read();
        expect_near(sample.humidity_pct, 49.0f, "humidity below midpoint", error_buffer, error_size);
        expect_near(sample.temperature_c, -50.0f, "negative temperature", error_buffer, error_size);
    }

    // Read: humidity clamped to the physical range.
    {
        test::FakeI2cBus bus;
        test::FakeI2cDevice& device = bus.add_device(drivers::Hts221Sensor::kI2cAddress);
        configure_hts221(device);
        drivers::Hts221Sensor sensor(bus);
        expect_true(sensor.init() == drivers::InitStatus::Ok, "init for clamp test", error_buffer, error_size);

        set_output_registers(device, -4000, 0);
        const auto dry = sensor.read();
        expect_near(dry.humidity_pct, 0.0f, "humidity clamped to 0", error_buffer, error_size);

        set_output_registers(device, 16000, 0);
        const auto wet = sensor.read();
        expect_near(wet.humidity_pct, 100.0f, "humidity clamped to 100", error_buffer, error_size);
    }

    // Read: bus error produces an invalid sample, not a crash.
    {
        test::FakeI2cBus bus;
        test::FakeI2cDevice& device = bus.add_device(drivers::Hts221Sensor::kI2cAddress);
        configure_hts221(device);
        drivers::Hts221Sensor sensor(bus);
        expect_true(sensor.init() == drivers::InitStatus::Ok, "init for bus-error test", error_buffer, error_size);

        bus.remove_device(drivers::Hts221Sensor::kI2cAddress);
        const auto sample = sensor.read();
        expect_true(!sample.valid_humidity && !sample.valid_temperature, "read after bus loss is invalid", error_buffer, error_size);
    }

    return failures == 0U;
}
