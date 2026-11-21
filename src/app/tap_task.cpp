/** @file tap_task.cpp
 *  @brief Task reporting double taps detected by the LSM6DSL hardware.
 *
 * The task sleeps blocked on a direct-to-task notification until the
 * LSM6DSL INT1 output pulses PD11 (EXTI11) — which happens exactly when the
 * chip's double-tap recognizer fires — so it consumes no CPU between
 * gestures. After waking it reads and decodes TAP_SRC over I2C in task
 * context (the bus wrapper is blocking, so the ISR itself only notifies).
 */

#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32l4xx_hal.h"

#include "app/app_context.hpp"
#include "bsp/board_state.hpp"
#include "bsp/tap_interrupt.hpp"
#include "drivers/lsm6dsl_tap.hpp"
#include "drivers/sensor_interface.hpp"
#include "app/tasks.hpp"
#include "rtos/rtos.hpp"

namespace app {

void tap_task(void* raw_ctx) {
    auto* ctx = static_cast<AppContext*>(raw_ctx);

    bool ready = false;
    uint32_t attempts = 0;
    uint32_t next_try_ms = 0;
    bool fault_flagged = false;

    char msg[112];

    for(;;) {
        const uint32_t now = HAL_GetTick();

        // Bring the tap detector up; retry with backoff until it succeeds.
        if (!ready) {
            if (now < next_try_ms) {
                rtos::sleep_ms(50);
                continue;
            }
            ++attempts;
            const auto status = ctx->lsm6dsl_tap.init();
            if (status == drivers::InitStatus::Ok) {
                ready = true;
                // Enable the PD11/EXTI11 interrupt after the chip is
                // configured (MD1_CFG routes the double-tap event to INT1).
                // Idempotent: re-registration after a bus-loss reinit only
                // restores the same task handle.
                bsp::tap_interrupt_init(xTaskGetCurrentTaskHandle());
                const int len = std::snprintf(msg, sizeof(msg),
                    "LSM6DSL 0x%02X tap detection ready (attempt %lu)\r\n",
                    ctx->lsm6dsl_tap.i2c_address(),
                    static_cast<unsigned long>(attempts));
                if (len > 0) {
                    ctx->uart.write(msg, static_cast<std::size_t>(len));
                }
            }
            else {
                fault_flagged = true;
                bsp::set_hardware_fault();
                const uint32_t backoff_ms = 1000U << ((attempts < 3U) ? attempts : 3U);
                next_try_ms = now + backoff_ms;
                const int len = std::snprintf(msg, sizeof(msg),
                    "LSM6DSL 0x%02X tap init failed (attempt %lu: %s); retrying in %lu s\r\n",
                    ctx->lsm6dsl_tap.i2c_address(),
                    static_cast<unsigned long>(attempts),
                    drivers::init_status_text(status),
                    static_cast<unsigned long>(backoff_ms / 1000U));
                if (len > 0) {
                    ctx->uart.write(msg, static_cast<std::size_t>(len));
                }
            }
        }

        // Same clearing rule as sensor_task: only faults this task raised.
        if (fault_flagged && ready) {
            fault_flagged = false;
            bsp::clear_hardware_fault();
        }

        if (!ready) {
            rtos::sleep_ms(50);
            continue;
        }

        // Sleep at 0% CPU until the INT1 pulse: the chip asserts INT1 for
        // the Quiet window (~29 ms) on each recognized double tap, and the
        // unlatched TAP_SRC flag stays readable for that whole window, so
        // the wake-read latency cannot miss the event.
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        const auto event = ctx->lsm6dsl_tap.read_event();
        if (event == drivers::TapEvent::DoubleTap) {
            const int len = std::snprintf(msg, sizeof(msg),
                "[%lu ms] Double tap detected\r\n",
                static_cast<unsigned long>(HAL_GetTick()));
            if (len > 0) {
                ctx->uart.write(msg, static_cast<std::size_t>(len));
            }
        }
        else if (event == drivers::TapEvent::BusError) {
            ready = false;
            fault_flagged = true;
            bsp::set_hardware_fault();
            const uint32_t backoff_ms = 1000U << ((attempts < 3U) ? attempts : 3U);
            next_try_ms = HAL_GetTick() + backoff_ms;
            ctx->uart.write("LSM6DSL tap source read failed; reinitializing\r\n");
        }
        // TapEvent::None: a stale notification from before a reinit window;
        // the unlatched flag already self-cleared, so there is nothing to do.
    }
}

} // namespace app
