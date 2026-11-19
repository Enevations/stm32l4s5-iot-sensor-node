#pragma once

/** @file i2c_bus.hpp
 *  @brief Thread-safe I2C2 bus interface for STM32 board peripherals.
 */

#include <cstdint>
#include <cstddef>

#include "stm32l4xx_hal.h"
#include "rtos.hpp"
#include "i2c_interface.hpp"

namespace bsp {

/**
 * @brief Controls the STM32 I2C2 peripheral and serializes bus transactions.
 *
 * The bus is intended for 7-bit peripheral addresses. Initialization uses PB10
 * and PB11 for I2C2 in 100 kHz Standard Mode. Implements the HAL-free II2cBus
 * abstraction so drivers and POST logic stay hardware-independent.
 */
class I2cBus final : public II2cBus {
public:

    /** Creates an uninitialized I2C bus wrapper. */
    I2cBus() noexcept = default;

    /// Hardware handles are non-copyable.
    I2cBus(const I2cBus&) = delete;

    /// Hardware handles are non-copyable.
    I2cBus& operator=(const I2cBus&) = delete;

    /**
     * @brief Initializes GPIOB pins PB10/PB11 and I2C2.
     * @return true when the peripheral initializes successfully.
     */
    bool init() noexcept;

    /** @brief Exposes the bus mutex for diagnostics. */
    rtos::Mutex& mutex() noexcept { return mutex_; }

    /**
     * @brief Checks whether a 7-bit device address acknowledges the bus.
     * @param dev_addr_7bit 7-bit peripheral address.
     * @param trials Number of readiness probes.
     * @param timeout_ms Timeout for each probe in milliseconds.
     * @return true when the device acknowledges.
     */
    bool is_device_ready(uint8_t dev_addr_7bit, uint32_t trials, uint32_t timeout_ms) noexcept override;

    /**
     * @brief Writes one value to an 8-bit register.
     * @param dev_addr_7bit 7-bit peripheral address.
     * @param reg_addr Device register address.
     * @param value Value to write.
     * @param timeout_ms HAL transaction timeout in milliseconds.
     * @return true on successful transmission.
     */
    bool write_reg(uint8_t dev_addr_7bit, uint8_t reg_addr, uint8_t value, uint32_t timeout_ms) noexcept override;

    /**
     * @brief Reads one value from an 8-bit register.
     * @param dev_addr_7bit 7-bit peripheral address.
     * @param reg_addr Device register address.
     * @param[out] out_value Receives the read value.
     * @param timeout_ms HAL transaction timeout in milliseconds.
     * @return true on successful reception.
     */
    bool read_reg(uint8_t dev_addr_7bit, uint8_t reg_addr, uint8_t& out_value, uint32_t timeout_ms) noexcept override;

    /**
     * @brief Reads a burst of bytes beginning at an 8-bit register address.
     * @param dev_addr_7bit 7-bit peripheral address.
     * @param start_reg_addr Address of the first register.
     * @param[out] buffer Destination buffer.
     * @param length Number of bytes to read.
     * @param timeout_ms HAL transaction timeout in milliseconds.
     * @return true on successful reception.
     */
    bool read_regs(uint8_t dev_addr_7bit, uint8_t start_reg_addr, uint8_t* buffer, std::size_t length, uint32_t timeout_ms) noexcept override;

private:
    I2C_HandleTypeDef hi2c_{};
    rtos::Mutex mutex_{};
};

} // namespace bsp