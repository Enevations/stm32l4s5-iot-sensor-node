/** @file main.cpp
 *  @brief Firmware entry point for the STM32L4S5I-IOT01A IoT sensor node.
 *
 * Composition root only: hardware bring-up, boot-time self-test, I2C POST,
 * task creation, and scheduler start. The responsibilities live in their own
 * modules: clock/POST in src/clock.cpp, fail-stop hooks in
 * src/fault_report.cpp, the self-test harness in src/self_test.cpp, the
 * application tasks in src/heartbeat.cpp, src/environment_tasks.cpp, and
 * src/tap_task.cpp, and the shared state in include/app_context.hpp.
 *
 * Four FreeRTOS tasks cooperate through a telemetry queue: a heartbeat task
 * (LED and stack-margin reports), a sensor task reading the onboard HTS221
 * humidity/temperature and LPS22HB pressure sensors over I2C2, a telemetry
 * task printing readings over the ST-LINK virtual COM port, and a tap task
 * polling the LSM6DSL hardware double-tap detector on the same bus.
 */

#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "app/app_context.hpp"
#include "bsp/board_state.hpp"
#include "bsp/clock.hpp"
#include "bsp/fault_report.hpp"
#include "bsp/led.hpp"
#include "rtos/rtos.hpp"
#include "app/self_test.hpp"
#include "app/tasks.hpp"
#include "bsp/uart.hpp"

// --- FreeRTOS Kernel Hooks ---
extern "C" {

/** @brief Advances HAL millisecond time from the FreeRTOS tick. */
void vApplicationTickHook() {
    HAL_IncTick();
}

} // extern "C"

static constexpr Led green_led(GPIOB_BASE, GPIO_PIN_14);
static Uart uart1(USART1, GPIOB_BASE, GPIO_PIN_6, GPIO_PIN_7, GPIO_AF7_USART1, 115200);

/// @brief Busy-wait approximation of a few hundred milliseconds.
///
/// Used only before HAL_Init succeeded, where SysTick and the HAL tick are
/// unavailable and HAL_Delay() would hang forever on the frozen tick.
static void busy_wait_no_tick() {
    for (volatile uint32_t i = 0; i < 250000U; ++i) {
        // Roughly 250 ms at the 4 MHz default clock.
    }
}

/// @brief Best-effort failure report before HAL_Init succeeded.
///
/// The LED and UART do not depend on SysTick or the HAL tick, and the clock
/// is still the default 4 MHz MSI, so both can be initialized and driven
/// even in this state.
static void fail_before_hal_init() {
    green_led.init();
    uart1.init();
    uart1.write("\r\n[FATAL] HAL_Init failed\r\n");
    while (true) {
        green_led.toggle();
        busy_wait_no_tick();
    }
}

/**
 * @brief Initializes the MCU and starts the FreeRTOS scheduler.
 * @return This function normally never returns.
 */
int main() {
    if (HAL_Init() != HAL_OK) {
        fail_before_hal_init();
    }

    const bool clock_ok = bsp::system_clock_config();
    bsp::ports_init(green_led, uart1);

    if (!clock_ok) {
        // Boot continues on the MSI fallback clock: HAL derives the UART
        // baud and I2C timing from the actual clock, and the heartbeat task
        // signals the degraded state with fast blinks.
        bsp::set_hardware_fault();
        uart1.write("\r\n[WARN] Clock config failed; running on fallback clock\r\n");
    }

#ifdef STM32_SELF_TEST
    app::run_self_tests(uart1);
#endif

    // Static storage duration is mandatory: the FreeRTOS Cortex-M4F port
    // resets MSP from the vector table when the first task starts, so any
    // object referenced by tasks must not live in main's stack frame. The
    // peripheral references it holds are the file statics above, declared
    // earlier in this translation unit, so initialization order is safe.
    static AppContext app{uart1, green_led};

    // Register the fault-time diagnostics (UART plus the two kernel-object
    // handles whose hex dumps make corruption comparable across faults).
    // Handles are stable after construction; must run before the scheduler
    // starts and before anything that can trip a fail-stop hook.
    bsp::register_fault_diagnostics(uart1,
                                    app.i2c_bus.mutex().handle(),
                                    app.telemetry_queue.handle());

    bsp::run_i2c_scan_check(app.i2c_bus, uart1);

    rtos::Task heartbeat_task("Heartbeat", 512, tskIDLE_PRIORITY + 2, app::heartbeat, &app);
    rtos::Task sensor_task_handle("SensorTask", 1024, tskIDLE_PRIORITY + 1, app::sensor_task, &app);
    rtos::Task telemetry_task_handle("TelemetryTask", 1024, tskIDLE_PRIORITY + 1, app::telemetry_task, &app);
    rtos::Task tap_task_handle("TapTask", 512, tskIDLE_PRIORITY + 1, app::tap_task, &app);

    // Mirror the task handles into app.task_handles for the heartbeat task's
    // stack report: the handles live in main()'s rtos::Task locals, which the
    // heartbeat task cannot reach. Must run after task creation (xTaskCreate
    // writes them) and before vTaskStartScheduler(), so no race is possible.
    app.task_handles[0] = heartbeat_task.handle();
    app.task_handles[1] = sensor_task_handle.handle();
    app.task_handles[2] = telemetry_task_handle.handle();
    app.task_handles[3] = tap_task_handle.handle();

    // Belt-and-suspenders creation checks. Heap exhaustion normally trips
    // vApplicationMallocFailedHook inside the kernel allocation itself, but
    // this check attributes a failure to the exact object and keeps the
    // fail-stop guarantee independent of that hook.
    if (!uart1.mutex().valid() || !app.i2c_bus.mutex().valid() ||
        !app.telemetry_queue.valid() || !heartbeat_task.valid() ||
        !sensor_task_handle.valid() || !telemetry_task_handle.valid() ||
        !tap_task_handle.valid()) {
        uart1.write("\r\n[FATAL] RTOS object creation failed\r\n");
        while (true) {}
    }
    vTaskStartScheduler();

    // The scheduler should not return; reaching here indicates startup failure.
    uart1.write("\r\n[FATAL] vTaskStartScheduler returned\r\n");
    while (true) {}
}
