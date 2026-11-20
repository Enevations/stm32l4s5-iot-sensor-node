#pragma once

#include <cstddef>
#include <cstdint>

namespace bsp {

/**
 * @brief HAL-free I2C bus abstraction.
 *
 * Implemented by I2cBus on the STM32 target and by test fakes in the unit
 * tests, so driver and POST logic can be exercised without hardware.
 * Default timeout values declared here apply to calls made through this
 * interface; implementations must not redeclare them.
 */
class II2cBus {
public:
    virtual ~II2cBus() = default;

    II2cBus(const II2cBus&) = delete;
    II2cBus& operator=(const II2cBus&) = delete;

    /** @copydoc I2cBus::is_device_ready */
    virtual bool is_device_ready(uint8_t dev_addr_7bit,
                                 uint32_t trials = 2,
                                 uint32_t timeout_ms = 50) noexcept = 0;

    /** @copydoc I2cBus::write_reg */
    virtual bool write_reg(uint8_t dev_addr_7bit,
                           uint8_t reg_addr,
                           uint8_t value,
                           uint32_t timeout_ms = 50) noexcept = 0;

    /** @copydoc I2cBus::read_reg */
    virtual bool read_reg(uint8_t dev_addr_7bit,
                          uint8_t reg_addr,
                          uint8_t& out_value,
                          uint32_t timeout_ms = 50) noexcept = 0;

    /** @copydoc I2cBus::read_regs
     *
     * Implementations must reject length values above UINT16_MAX (returning
     * false) rather than truncating into a partial read.
     */
    virtual bool read_regs(uint8_t dev_addr_7bit,
                           uint8_t start_reg_addr,
                           uint8_t* buffer,
                           std::size_t length,
                           uint32_t timeout_ms = 50) noexcept = 0;

protected:
    II2cBus() = default;
};

} // namespace bsp
