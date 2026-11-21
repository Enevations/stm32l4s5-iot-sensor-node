/** @file tap_task.cpp
 *  @brief Task reporting double taps detected by the LSM6DSL hardware.
 */

#include <cstdio>

#include "FreeRTOS.h"
#include "stm32l4xx_hal.h"

#include "app/app_context.hpp"
#include "bsp/board_state.hpp"
#include "drivers/lsm6dsl_tap.hpp"
#include "rtos/rtos.hpp"
#include "drivers/sensor_interface.hpp"
#include "app/tasks.hpp"

namespace app {

void tap_task(void* raw_ctx) {
    auto* ctx = static_cast<AppContext*>(raw_ctx);

    bool ready = false;
    uint32_t attempts = 0;
    uint32_t next_try_ms = 0;
    bool fault_flagged = false;
    // Rate limit for the single-tap tuning aid; low enough that consecutive
    // knocks of a double attempt show their real spacing in the log.
    uint32_t last_single_log_ms = 0;
    constexpr uint32_t kSingleLogMinGapMs = 200U;
    // Unlatched double-tap flags stay visible ~29 ms, so consecutive polls
    // can report the same gesture twice; suppress repeats inside 100 ms.
    uint32_t last_double_log_ms = 0;
    constexpr uint32_t kDoubleLogMinGapMs = 100U;

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

        // The double-tap flag is visible ~29 ms after recognition, inside
        // the 20 ms poll interval, so gestures are not missed.
        const auto event = ctx->lsm6dsl_tap.read_event();
        if (event == drivers::TapEvent::DoubleTap) {
            const uint32_t now_ms = HAL_GetTick();
            if (now_ms - last_double_log_ms >= kDoubleLogMinGapMs) {
                last_double_log_ms = now_ms;
                const int len = std::snprintf(msg, sizeof(msg),
                    "[%lu ms] Double tap detected\r\n",
                    static_cast<unsigned long>(now_ms));
                if (len > 0) {
                    ctx->uart.write(msg, static_cast<std::size_t>(len));
                }
            }
        }
        else if (event == drivers::TapEvent::SingleTap) {
            // Tuning aid: the chip sees taps but did not pair them into a
            // double (second tap too weak or too far apart). Rate-limited.
            const uint32_t now_ms = HAL_GetTick();
            if (now_ms - last_single_log_ms >= kSingleLogMinGapMs) {
                last_single_log_ms = now_ms;
                const int len = std::snprintf(msg, sizeof(msg),
                    "[%lu ms] Single tap detected (no second tap within 0.5 s)\r\n",
                    static_cast<unsigned long>(now_ms));
                if (len > 0) {
                    ctx->uart.write(msg, static_cast<std::size_t>(len));
                }
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

        rtos::sleep_ms(20);
    }
}

} // namespace app
