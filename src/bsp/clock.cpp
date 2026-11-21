/** @file clock.cpp
 *  @brief Clock tree, port, and power-on self-test initialization.
 */

#include "bsp/clock.hpp"

#include <cstdio>
#include <cstring>

#include "bsp/board_state.hpp"
#include "bsp/post_check.hpp"
#include "stm32l4xx_hal.h"

namespace bsp {

bool system_clock_config() noexcept {
    RCC_OscInitTypeDef osc{};
    RCC_ClkInitTypeDef clk{};

    // Scale 1 supports the selected 80 MHz system clock.
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        return false;
    }

    osc.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    osc.MSIState = RCC_MSI_ON;
    osc.MSICalibrationValue = 0;
    osc.MSIClockRange = RCC_MSIRANGE_6; // 4 MHz
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    osc.PLL.PLLM = 1;
    osc.PLL.PLLN = 40; // 4 MHz MSI / 1 * 40 = 160 MHz VCO
    osc.PLL.PLLP = RCC_PLLP_DIV7;
    osc.PLL.PLLQ = RCC_PLLQ_DIV2;
    osc.PLL.PLLR = RCC_PLLR_DIV2; // SYSCLK = 160 MHz / 2 = 80 MHz
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        return false;
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    return HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) == HAL_OK;
}

void ports_init(const Led& led, Uart& uart) noexcept {
    led.init();
    uart.init();
}

void run_i2c_scan_check(I2cBus& bus, Uart& uart) noexcept {
    if (!bus.init()) {
        uart.write("I2C2 init failed\n");
        set_hardware_fault();
        return;
    }

    const PostResult post = run_post_check(bus,
                                           kExpectedI2CDevices.data(),
                                           kExpectedI2CDevices.size());
    if (post.all_found) {
        uart.write("[Post Success] All expected devices found\n");
        return;
    }

    char msg[64];
    std::snprintf(msg, sizeof(msg), "[Post Fail] Missing device at 0x%02X\n",
                  post.missing_address);
    uart.write(msg, std::strlen(msg));
    uart.write("[Post Fail] One or more expected devices missing\n");
    set_hardware_fault();
}

} // namespace bsp
