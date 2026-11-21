#pragma once

/** @file fault_report.hpp
 *  @brief Fail-stop hooks and hard-fault reporting for boot and kernel
 *         failures.
 *
 * Every hook prints its cause over the registered UART before halting; a
 * silent freeze means the fault occurred before the UART was initialized or
 * before register_fault_diagnostics() ran.
 */

#include "uart.hpp"

namespace bsp {

/**
 * @brief Registers the objects the fail-stop hooks report about.
 *
 * Call once from main() after the UART, I2C bus, and telemetry queue exist
 * and before the scheduler starts (single-threaded context, no locking
 * needed). The handles are captured by value: they are stable pointers that
 * do not change after kernel-object creation. Hooks firing before
 * registration stay silent instead of touching uninitialized hardware.
 *
 * @param uart UART used for all fault output.
 * @param i2c_mutex_handle `I2cBus::mutex().handle()` of the app's I2C bus.
 * @param telemetry_queue_handle `Queue::handle()` of the telemetry queue.
 */
void register_fault_diagnostics(Uart& uart,
                                const void* i2c_mutex_handle,
                                const void* telemetry_queue_handle) noexcept;

} // namespace bsp
