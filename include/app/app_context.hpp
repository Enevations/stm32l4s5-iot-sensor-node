#pragma once

/** @file app_context.hpp
 *  @brief Shared application state: bus, drivers, telemetry queue, and the
 *         board peripherals the tasks print to and blink.
 */

#include <cstdint>

#include "bsp/led.hpp"
#include "rtos/rtos.hpp"
#include "drivers/sensor_interface.hpp"
#include "bsp/uart.hpp"
#include "bsp/i2c_bus.hpp"
#include "drivers/hts221_sensor.hpp"
#include "drivers/lps22hb_sensor.hpp"
#include "drivers/lsm6dsl_tap.hpp"

/// @brief One telemetry queue item: a merged sensor reading plus its stamp.
struct TelemetryMessage {
    drivers::EnvironmentalSample sample;
    uint32_t timestamp_ms{0};
};

/**
 * @brief Shared application state passed to every task as its context.
 *
 * Must be constructed with static storage duration in main(): the
 * Cortex-M4F FreeRTOS port resets MSP from the vector table when the first
 * task starts, so anything tasks reference must not live in main's stack
 * frame. The peripheral references must outlive it (they are file statics
 * in the same translation unit).
 */
struct AppContext {
    explicit AppContext(Uart& uart_ref, const Led& status_led_ref) noexcept
        : uart(uart_ref), status_led(status_led_ref) {}

    /// Telemetry lines, fault lines, and tap messages go here.
    Uart& uart;

    /// Heartbeat status LED (blinks fast in the hardware-fault state).
    const Led& status_led;

    bsp::I2cBus i2c_bus;
    drivers::Hts221Sensor hts221{i2c_bus};
    drivers::Lps22hbSensor lps22hb{i2c_bus};
    drivers::Lsm6dslTap lsm6dsl_tap{i2c_bus};
    rtos::Queue<TelemetryMessage> telemetry_queue{8};

    /// Native task handles mirrored from main()'s rtos::Task objects so the
    /// heartbeat task can query each task's stack high-water mark; a null
    /// entry makes the report fall back to heartbeat's own stack for that
    /// slot. Slots: 0 heartbeat, 1 sensor, 2 telemetry, 3 tap. Written once
    /// by main() between task creation and scheduler start (no race: the
    /// scheduler is not running), read by the heartbeat task afterwards.
    TaskHandle_t task_handles[4]{};
};
