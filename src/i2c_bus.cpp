#include "i2c_bus.hpp"

namespace bsp {

bool I2cBus::init() noexcept {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();

    // Configure PB10 (SCL) and PB11 (SDA) as Open-Drain Alternate Function 4
    GPIO_InitTypeDef gpio_init{};
    gpio_init.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio_init.Mode = GPIO_MODE_AF_OD;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    // Reset I2C2 peripheral to clear any stuck bus state after soft-reset
    __HAL_RCC_I2C2_FORCE_RESET();
    __HAL_RCC_I2C2_RELEASE_RESET();

    hi2c_.Instance = I2C2;
    hi2c_.Init.Timing = 0x10909CEC; // 100 kHz Standard Mode @ 80 MHz PCLK1
    hi2c_.Init.OwnAddress1 = 0;
    hi2c_.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c_.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c_.Init.OwnAddress2 = 0;
    hi2c_.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c_.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c_.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c_) != HAL_OK) {
        return false;
    }

    // Enable analog noise filter (recommended by ST reference manual)
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c_, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        return false;
    }

    return true;
}

bool I2cBus::is_device_ready(uint8_t dev_addr_7bit, uint32_t trials, uint32_t timeout_ms) noexcept {
    rtos::SchedulerLock lock(mutex_);
    if (!lock) {
        return false;
    }
    const uint16_t addr_8bit = static_cast<uint16_t>(dev_addr_7bit) << 1;
    return HAL_I2C_IsDeviceReady(&hi2c_, addr_8bit, trials, timeout_ms) == HAL_OK;
}

bool I2cBus::write_reg(uint8_t dev_addr_7bit, uint8_t reg_addr, uint8_t value, uint32_t timeout_ms) noexcept {
    rtos::SchedulerLock lock(mutex_);
    if (!lock) {
        return false;
    }
    const uint16_t addr_8bit = static_cast<uint16_t>(dev_addr_7bit) << 1;
    return HAL_I2C_Mem_Write(&hi2c_, addr_8bit, reg_addr, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, timeout_ms) == HAL_OK;
}

bool I2cBus::read_reg(uint8_t dev_addr_7bit, uint8_t reg_addr, uint8_t& out_value, uint32_t timeout_ms) noexcept {
    rtos::SchedulerLock lock(mutex_);
    if (!lock) {
        return false;
    }
    const uint16_t addr_8bit = static_cast<uint16_t>(dev_addr_7bit) << 1;
    return HAL_I2C_Mem_Read(&hi2c_, addr_8bit, reg_addr, I2C_MEMADD_SIZE_8BIT,
                            &out_value, 1, timeout_ms) == HAL_OK;
}

bool I2cBus::read_regs(uint8_t dev_addr_7bit, uint8_t start_reg_addr, uint8_t* buffer, std::size_t length, uint32_t timeout_ms) noexcept {
    if (buffer == nullptr || length == 0) {
        return false;
    }

    rtos::SchedulerLock lock(mutex_);
    if (!lock) {
        return false;
    }
    const uint16_t addr_8bit = static_cast<uint16_t>(dev_addr_7bit) << 1;
    return HAL_I2C_Mem_Read(&hi2c_, addr_8bit, start_reg_addr, I2C_MEMADD_SIZE_8BIT,
                            buffer, static_cast<uint16_t>(length), timeout_ms) == HAL_OK;
}

} // namespace bsp