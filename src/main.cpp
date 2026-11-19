/** @file main.cpp
 *  @brief Firmware entry point for the STM32L4S5I-IOT01A blink example.
 */

#include <cstdio>
#include <cstring>
#include <atomic>

#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c_bus.hpp"
#include "uart.hpp"
#include "led.hpp"
#include "humidity_temp_snsr.hpp"

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

enum class BoardState : uint8_t {
    Healthy = 0,
    HardwareFault = 1,
};

struct TelemetryMessage {
    uint32_t timestamp_ms{0};
    float temperature_c{0.0f};
    float humidity_pct{0.0f};
};

struct AppContext {
    bsp::I2cBus i2c_bus;
    rtos::Queue<TelemetryMessage> telemetry_queue{8};
};

static constexpr Led green_led(GPIOB_BASE, GPIO_PIN_14);
static Uart uart1(USART1, GPIOB_BASE, GPIO_PIN_6, GPIO_PIN_7, GPIO_AF7_USART1, 115200);
inline std::atomic<BoardState> gboard_state{BoardState::Healthy};

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
static void ports_init() {
    green_led.init();
    uart1.init();
}

///** @brief Scans the I2C bus for devices and prints their addresses to UART1. If fail inform user and fail. */
static void i2c_scan_check(void* raw_ctx) {
    auto* ctx = static_cast<AppContext*>(raw_ctx);
    char msg[64];

    if (!ctx->i2c_bus.init()) {
        std::snprintf(msg, sizeof(msg), "I2C2 init failed\n");
        uart1.write(msg, std::strlen(msg));
        gboard_state.store(BoardState::HardwareFault, std::memory_order_release);
        return;
    }

    bool all_devices_found = true;

    for(uint8_t addr: bsp::kExpectedI2CDevices) {
        if (!ctx->i2c_bus.is_device_ready(addr)) {
            std::snprintf(msg, sizeof(msg), "[Post Fail] Missing device at 0x%02X\n", addr);
            uart1.write(msg, std::strlen(msg));
            all_devices_found = false;
        }
    }

    if (all_devices_found) {
        std::snprintf(msg, sizeof(msg), "[Post Success] All expected devices found\n");
        uart1.write(msg, std::strlen(msg));
    }
    else {
        std::snprintf(msg, sizeof(msg), "[Post Fail] One or more expected devices missing\n");
        uart1.write(msg, std::strlen(msg));
        gboard_state.store(BoardState::HardwareFault, std::memory_order_release);
    }

}

/** @brief Toggles the green LED at a regular interval. */
static void heartbeat(void*) {

    const char* banner = "\r\nSTM32L4S5I-IOT01A Sensor Node\r\n";
    uart1.write(banner);


    for(;;) {
        if (gboard_state.load(std::memory_order_acquire) == BoardState::HardwareFault) {
            green_led.toggle();
            rtos::sleep_ms(250);
        }
        else {
            green_led.toggle();
            rtos::sleep_ms(1000);
        }
    }
}

static void sensor_task(void* raw_ctx) {
    auto* ctx = static_cast<AppContext*>(raw_ctx);
    drivers::HumidityTempSnsr sensor(ctx->i2c_bus);

    while (!sensor.init()) {
        uart1.write("Sensor init failed\n");
        gboard_state.store(BoardState::HardwareFault, std::memory_order_release);
        for(;;) {
            rtos::sleep_ms(1000);
        }
    }

    for(;;) {
        const auto sample = sensor.read();
        if (sample.valid) {
            TelemetryMessage msg{
                .timestamp_ms = HAL_GetTick(),
                .temperature_c = sample.temperature_c,
                .humidity_pct = sample.humidity_pct
            };
            ctx->telemetry_queue.send(msg);
        }
        else {
            uart1.write("Sensor read failed! Try again.\n");
        }
        rtos::sleep_ms(1000);
    }
}

/**
 * @brief Components of a signed float rendered with two fractional digits.
 *
 * Fraction digits are rounded, not truncated, so 9.999 renders as "10.00";
 * the carry from a rounded fraction of 100 is applied to whole. The sign is
 * kept separate from whole so values in (-1, 0) render as "-0.40" instead of
 * "0.40".
 */
struct SplitFloat {
    /** @brief True when the original value was negative. */
    bool negative{false};
    /** @brief Integer part of the absolute value. */
    int whole{0};
    /** @brief Fraction part in hundredths, always in [0, 99]. */
    int frac{0};
};

/**
 * @brief Splits a signed float into sign, integer part, and rounded hundredths.
 *
 * Avoids the float formatter in std::snprintf, which would pull the heavyweight
 * newlib float-printf path into the firmware.
 *
 * @param value Value to split, rounded to two fractional digits.
 * @return Components suitable for "%s%d.%02d"-style formatting; the sign is a
 *         plain "-" prefix, so tiny negatives such as -0.001 render "-0.00".
 */
static SplitFloat split_float(float value) noexcept {
    SplitFloat out;
    if (value < 0.0f) {
        out.negative = true;
        value = -value;
    }
    out.whole = static_cast<int>(value);
    const int rounded_frac =
        static_cast<int>((value - static_cast<float>(out.whole)) * 100.0f + 0.5f);
    if (rounded_frac >= 100) {
        out.whole += 1;
    } else {
        out.frac = rounded_frac;
    }
    return out;
}

static void telemetry_task(void* raw_ctx) {
    auto* ctx = static_cast<AppContext*>(raw_ctx);
    TelemetryMessage sample{};
    char msg[96];

    for(;;) {
        if (ctx->telemetry_queue.receive(sample)) {
            const SplitFloat temp = split_float(sample.temperature_c);
            const SplitFloat hum  = split_float(sample.humidity_pct);

            const int len = std::snprintf(
                msg, sizeof(msg),
                "[%lu ms] Temp: %s%d.%02d C | Humidity: %d.%02d %%\r\n",
                sample.timestamp_ms,
                (temp.negative ? "-" : ""), temp.whole, temp.frac,
                hum.whole, hum.frac
            );

            if (len > 0) {
                uart1.write(msg, static_cast<std::size_t>(len));
            }
        }
    }
}

/**
 * @brief Initializes the MCU and starts the FreeRTOS scheduler.
 * @return This function normally never returns.
 */
int main() {
    HAL_Init();
    system_clock_config();
    ports_init();

    AppContext app{};

    i2c_scan_check(&app);
    
    
    rtos::Task heartbeat_task("Heartbeat", 256, tskIDLE_PRIORITY + 2, heartbeat);
    rtos::Task sensor_task_handle("SensorTask", 512, tskIDLE_PRIORITY + 1, sensor_task, &app);
    rtos::Task telemetry_task_handle("TelemetryTask", 512, tskIDLE_PRIORITY + 1, telemetry_task, &app);
    vTaskStartScheduler();

    // The scheduler should not return; reaching here indicates startup failure.
    while (true) {}
}