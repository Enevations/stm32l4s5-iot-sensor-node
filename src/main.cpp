/** @file main.cpp
 *  @brief Firmware entry point for the STM32L4S5I-IOT01A blink example.
 */

#include <cstdio>
#include <cstring>

#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c_bus.hpp"
#include "uart.hpp"
#include "led.hpp"

#ifndef NDEBUG
// OpenOCD uses this symbol to determine the priority range for FreeRTOS thread
// awareness; keep it in debug builds even when the linker would otherwise drop it.
extern "C" {
__attribute__((used)) int uxTopUsedPriority = configMAX_PRIORITIES - 1;
}
#endif

// --- FreeRTOS Kernel Hooks ---
extern "C" {

/** @brief Advances HAL millisecond time from the FreeRTOS tick. */
void vApplicationTickHook() {
    HAL_IncTick();
}

/** @brief Fail-stop hook invoked when FreeRTOS cannot allocate memory. */
void vApplicationMallocFailedHook() {
    taskDISABLE_INTERRUPTS();
    while (true) {}
}

/** @brief Fail-stop hook invoked when a task overflow is detected. */
void vApplicationStackOverflowHook(TaskHandle_t, char*) {
    taskDISABLE_INTERRUPTS();
    while (true) {}
}

} // extern "C"

static constexpr Led green_led(GPIOB_BASE, GPIO_PIN_14);
static Uart uart1(USART1, GPIOB_BASE, GPIO_PIN_6, GPIO_PIN_7, GPIO_AF7_USART1, 115200);


// --- Hardware Initialization ---
/** @brief Configures the MSI/PLL clock tree for an 80 MHz system clock. */
static void system_clock_config() {
    RCC_OscInitTypeDef osc{};
    RCC_ClkInitTypeDef clk{};

    // Scale 1 supports the selected 80 MHz system clock.
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

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
    HAL_RCC_OscConfig(&osc);

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4);
}

/** @brief Configures GPIO and the USART1 ST-LINK virtual COM port pins. */
static void gpio_and_uart_init() {
    green_led.init();
    uart1.init();
}

static void heartbeat(void*) {

    const char* banner = "\r\nSTM32L4S5I-IOT01A Sensor Node\r\n";
    uart1.write(banner, std::strlen(banner));

    for(;;) {
        green_led.toggle();
        rtos::sleep_ms(1000);
    }
}

static void i2c_task(void*) {

    char msg[64];

    bsp::I2cBus i2c2;
    if (!i2c2.init()) {
        for(;;) {
            std::snprintf(msg, sizeof(msg), "I2C2 init failed\n");
            uart1.write(msg);
            rtos::sleep_ms(1000);
        }
    }

    for(uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        if (i2c2.is_device_ready(addr)) {
            std::snprintf(msg, sizeof(msg), "I2C device found at 0x%02X\n", addr);
            uart1.write(msg, std::strlen(msg));
        }
    }

    for(;;) {
        rtos::sleep_ms(1000);
    }

}

/**
 * @brief Initializes the MCU and starts the FreeRTOS scheduler.
 * @return This function normally never returns.
 */
int main() {
    HAL_Init();
    system_clock_config();
    gpio_and_uart_init();

    rtos::Task scan_task("I2C Scan", 512, tskIDLE_PRIORITY + 1, i2c_task);
    rtos::Task heartbeat_task("Heartbeat", 256, tskIDLE_PRIORITY + 2, heartbeat);
    vTaskStartScheduler();

    // The scheduler should not return; reaching here indicates startup failure.
    while (true) {}
}