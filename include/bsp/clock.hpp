#pragma once

/** @file clock.hpp
 *  @brief Clock tree, port, and power-on self-test initialization.
 */

#include "bsp/i2c_bus.hpp"
#include "bsp/led.hpp"
#include "bsp/uart.hpp"

namespace bsp {

/**
 * @brief Configures the MSI/PLL clock tree for an 80 MHz system clock.
 *
 * @return true on success; false when any stage failed, in which case the
 *         MCU keeps running on the default 4 MHz MSI and the caller should
 *         flag the degraded state.
 */
[[nodiscard]] bool system_clock_config() noexcept;

/**
 * @brief Configures GPIO and the USART1 ST-LINK virtual COM port pins.
 *
 * @param led Board status LED to initialize.
 * @param uart ST-LINK virtual COM port to initialize.
 */
void ports_init(const Led& led, Uart& uart) noexcept;

/**
 * @brief Initializes the I2C bus and verifies that all expected onboard
 *        sensors acknowledge.
 *
 * Runs from main() before the scheduler starts; the I2C bus mutex is not
 * required there because execution is single-threaded (see SchedulerLock).
 * The probing logic itself lives in bsp::run_post_check, which is
 * unit-tested against a fake bus. Sets the global hardware-fault state when
 * the bus or any expected device fails to respond; the heartbeat task
 * signals the fault with fast blinks.
 *
 * @param bus The application's I2C bus.
 * @param uart UART used for the POST result lines.
 */
void run_i2c_scan_check(I2cBus& bus, Uart& uart) noexcept;

} // namespace bsp
