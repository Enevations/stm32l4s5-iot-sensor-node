/** @file main.cpp
 *  @brief Firmware entry point for the STM32L4S5I-IOT01A IoT sensor node.
 *
 * Three FreeRTOS tasks cooperate through a telemetry queue: a heartbeat task
 * (LED and stack-margin reports), a sensor task reading the onboard HTS221
 * humidity/temperature sensor over I2C2, and a telemetry task printing
 * readings over the ST-LINK virtual COM port.
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

/// Native task handles mirrored from main()'s rtos::Task objects so heartbeat()
/// can query each task's stack high-water mark; a null entry makes the report
/// fall back to heartbeat's own stack for that slot.
static TaskHandle_t g_task_handles[3]{};

/// Application context pointer for fault-time diagnostics.
static AppContext* g_app_context{nullptr};

/// @brief Hex-dumps a kernel object so corruption can be compared over time.
static void dump_kernel_object(const char* tag, const void* object) {
    char line[72];
    const int header_len = std::snprintf(line, sizeof(line), "[%s] @%p:", tag, object);
    if (header_len > 0) {
        uart1.write(line, static_cast<std::size_t>(header_len));
    }
    const auto* bytes = static_cast<const uint8_t*>(object);
    for (std::size_t i = 0; i < 64U; ++i) {
        char hex[8];
        const int len = std::snprintf(hex, sizeof(hex), " %02X", bytes[i]);
        if (len > 0) {
            uart1.write(hex, static_cast<std::size_t>(len));
        }
    }
    uart1.write("\r\n", 2U);
}

/// @brief Dumps the three kernel objects involved in the crash for comparison.
static void dump_rtos_objects(const char* when) {
    char header[64];
    const int len = std::snprintf(header, sizeof(header), "[DUMP] %s\r\n", when);
    if (len > 0) {
        uart1.write(header, static_cast<std::size_t>(len));
    }
    dump_kernel_object("uart_mutex", uart1.mutex().handle());
    if (g_app_context != nullptr) {
        dump_kernel_object("i2c_mutex", g_app_context->i2c_bus.mutex().handle());
        dump_kernel_object("telemetry_q", g_app_context->telemetry_queue.handle());
    }
}

// --- Fail-Stop Handlers ---
// Each path prints the cause over UART1 before halting; a silent freeze means
// the fault occurred before UART1 was initialized.
extern "C" {

/** @brief Fail-stop hook invoked when FreeRTOS cannot allocate memory. */
void vApplicationMallocFailedHook() {
    taskDISABLE_INTERRUPTS();
    static const char msg[] = "\r\n[FATAL] FreeRTOS heap exhausted (malloc failed hook)\r\n";
    uart1.write(msg);
    while (true) {}
}

/** @brief Fail-stop hook invoked when a task overflow is detected. */
void vApplicationStackOverflowHook(TaskHandle_t, char* task_name) {
    taskDISABLE_INTERRUPTS();
    char msg[80];
    const int len = std::snprintf(msg, sizeof(msg),
                                  "\r\n[FATAL] Stack overflow in task: %s\r\n",
                                  (task_name != nullptr) ? task_name : "?");
    if (len > 0) {
        uart1.write(msg, static_cast<std::size_t>(len));
    }
    while (true) {}
}

/** @brief Fail-stop handler invoked on a hard fault.
 *
 * Runs in handler mode on the untouched MSP stack, so it stays safe even when
 * the fault was caused by a task overflowing its own (PSP) stack. Prints the
 * stacked PC plus fault-status registers; decode PC with:
 *   arm-none-eabi-addr2line -e build/debug/stm32l4s5-base.elf -f -C 0x<PC>
 */
[[gnu::naked]] void HardFault_Handler() {
    __asm volatile(
        "tst   lr, #4          \n"
        "ite   eq              \n"
        "mrseq r0, msp         \n"
        "mrsne r0, psp         \n"
        "mov   r1, lr          \n"
        "b     hard_fault_report \n");
}

/** @brief Reports hard-fault context: task, full stacked frame, and fault registers.
 *
 * @param frame Basic exception frame: r0, r1, r2, r3, r12, lr, pc, xpsr.
 * @param exc_return EXC_RETURN value captured at handler entry (link register).
 */
extern "C" [[noreturn]] void hard_fault_report(uint32_t* frame, uint32_t exc_return) {
    taskDISABLE_INTERRUPTS();
    uint32_t psp{0};
    __asm volatile("mrs %0, psp" : "=r"(psp));
    const char* task_name = pcTaskGetName(nullptr);

    char msg[256];
    const int len = std::snprintf(msg, sizeof(msg),
        "\r\n[FATAL] HardFault task=%s EXC_RET=0x%08lX PSP=0x%08lX\r\n"
        "PC=0x%08lX LR=0x%08lX xPSR=0x%08lX\r\n"
        "r0=0x%08lX r1=0x%08lX r2=0x%08lX r3=0x%08lX r12=0x%08lX\r\n"
        "CFSR=0x%08lX HFSR=0x%08lX BFAR=0x%08lX MMFAR=0x%08lX\r\n",
        (task_name != nullptr) ? task_name : "?",
        static_cast<unsigned long>(exc_return),
        static_cast<unsigned long>(psp),
        static_cast<unsigned long>(frame[6]),   // PC at fault
        static_cast<unsigned long>(frame[5]),   // LR in faulting code
        static_cast<unsigned long>(frame[7]),   // xPSR
        static_cast<unsigned long>(frame[0]),
        static_cast<unsigned long>(frame[1]),
        static_cast<unsigned long>(frame[2]),
        static_cast<unsigned long>(frame[3]),
        static_cast<unsigned long>(frame[4]),
        static_cast<unsigned long>(SCB->CFSR),
        static_cast<unsigned long>(SCB->HFSR),
        static_cast<unsigned long>(SCB->BFAR),
        static_cast<unsigned long>(SCB->MMFAR));
    if (len > 0) {
        uart1.write(msg, static_cast<std::size_t>(len));
    }
    dump_rtos_objects("at fault");
    while (true) {}
}

} // extern "C"

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

/**
 * @brief Initializes I2C2 and verifies that all expected onboard sensors ACK.
 *
 * Runs from main() before the scheduler starts; the I2C bus mutex is not
 * required there because execution is single-threaded (see SchedulerLock).
 * Sets the global hardware-fault state when the bus or any expected device
 * fails to respond; the heartbeat task signals the fault with fast blinks.
 *
 * @param raw_ctx Pointer to the AppContext owning the I2C bus.
 */
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

/** @brief Toggles the green LED at a regular interval and reports stack margins. */
static void heartbeat(void* raw_ctx) {
    (void)raw_ctx;

    const char* banner = "\r\nSTM32L4S5I-IOT01A Sensor Node\r\n";
    uart1.write(banner);

    uint32_t loops{0};
    char msg[112];

    for(;;) {
        green_led.toggle();
        const bool hardware_fault =
            gboard_state.load(std::memory_order_acquire) == BoardState::HardwareFault;
        rtos::sleep_ms(hardware_fault ? 250u : 1000u);

        // Report remaining stack in words every ~5 s; near-zero values identify
        // the task at risk of overflow.
        if (++loops < (hardware_fault ? 20u : 5u)) {
            continue;
        }
        loops = 0;
        const unsigned long hb =
            static_cast<unsigned long>(uxTaskGetStackHighWaterMark(g_task_handles[0]));
        const unsigned long sn =
            static_cast<unsigned long>(uxTaskGetStackHighWaterMark(g_task_handles[1]));
        const unsigned long tm =
            static_cast<unsigned long>(uxTaskGetStackHighWaterMark(g_task_handles[2]));
        const int len = std::snprintf(msg, sizeof(msg),
                                      "Stack free words: HB=%lu SN=%lu TM=%lu\r\n", hb, sn, tm);
        if (len > 0) {
            uart1.write(msg, static_cast<std::size_t>(len));
        }
    }
}

/**
 * @brief Samples the HTS221 humidity/temperature sensor once per second.
 *
 * Loads the sensor's factory calibration during init; if that fails, the task
 * reports a hardware fault and parks forever (the heartbeat keeps running so
 * the fault pattern stays visible). Valid samples are queued for the
 * telemetry task; failed reads are logged and retried on the next cycle.
 *
 * @param raw_ctx Pointer to the AppContext owning the I2C bus and queue.
 */
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

/**
 * @brief Prints queued sensor samples as text lines over UART1.
 *
 * Blocks on the telemetry queue and formats each sample with integer-only
 * math (see split_float) to avoid pulling the newlib float formatter into
 * the firmware image.
 *
 * @param raw_ctx Pointer to the AppContext owning the telemetry queue.
 */
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

    // Static storage duration is mandatory: the FreeRTOS Cortex-M4F port
    // resets MSP from the vector table when the first task starts, so any
    // object referenced by tasks must not live in main's stack frame.
    static AppContext app{};
    g_app_context = &app;

    i2c_scan_check(&app);

    rtos::Task heartbeat_task("Heartbeat", 512, tskIDLE_PRIORITY + 2, heartbeat, &app);
    rtos::Task sensor_task_handle("SensorTask", 1024, tskIDLE_PRIORITY + 1, sensor_task, &app);
    rtos::Task telemetry_task_handle("TelemetryTask", 1024, tskIDLE_PRIORITY + 1, telemetry_task, &app);
    
    // Mirror the task handles into g_task_handles for heartbeat()'s stack
    // report: the handles live in main()'s rtos::Task locals, which heartbeat
    // cannot reach. Must run after task creation (xTaskCreate writes them)
    // and before vTaskStartScheduler(), so no race is possible.
    g_task_handles[0] = heartbeat_task.handle();
    g_task_handles[1] = sensor_task_handle.handle();
    g_task_handles[2] = telemetry_task_handle.handle();

    // Belt-and-suspenders creation checks. Heap exhaustion normally trips
    // vApplicationMallocFailedHook inside the kernel allocation itself, but
    // this check attributes a failure to the exact object and keeps the
    // fail-stop guarantee independent of that hook.
    if (!uart1.mutex().valid() || !app.i2c_bus.mutex().valid() ||
        !app.telemetry_queue.valid() || !heartbeat_task.valid() ||
        !sensor_task_handle.valid() || !telemetry_task_handle.valid()) {
        uart1.write("\r\n[FATAL] RTOS object creation failed\r\n");
        while (true) {}
    }
    vTaskStartScheduler();

    // The scheduler should not return; reaching here indicates startup failure.
    uart1.write("\r\n[FATAL] vTaskStartScheduler returned\r\n");
    while (true) {}
}