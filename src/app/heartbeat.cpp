/** @file heartbeat.cpp
 *  @brief Status LED task: blink cadence and per-task stack-margin reports.
 */

#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"

#include "app/app_context.hpp"
#include "bsp/board_state.hpp"
#include "app/tasks.hpp"

namespace app {

void heartbeat(void* raw_ctx) {
    auto* ctx = static_cast<AppContext*>(raw_ctx);

    const char* banner = "\r\nSTM32L4S5I-IOT01A Sensor Node\r\n";
    ctx->uart.write(banner);

    uint32_t loops{0};
    char msg[112];

    for(;;) {
        ctx->status_led.toggle();
        const bool hardware_fault = bsp::is_hardware_fault_active();
        rtos::sleep_ms(hardware_fault ? 250u : 1000u);

        // Report remaining stack in words every ~5 s; near-zero values identify
        // the task at risk of overflow.
        if (++loops < (hardware_fault ? 20u : 5u)) {
            continue;
        }
        loops = 0;
        const unsigned long hb =
            static_cast<unsigned long>(uxTaskGetStackHighWaterMark(ctx->task_handles[0]));
        const unsigned long sn =
            static_cast<unsigned long>(uxTaskGetStackHighWaterMark(ctx->task_handles[1]));
        const unsigned long tm =
            static_cast<unsigned long>(uxTaskGetStackHighWaterMark(ctx->task_handles[2]));
        const unsigned long tp =
            static_cast<unsigned long>(uxTaskGetStackHighWaterMark(ctx->task_handles[3]));
        const int len = std::snprintf(msg, sizeof(msg),
                                      "Stack free words: HB=%lu SN=%lu TM=%lu TP=%lu\r\n",
                                      hb, sn, tm, tp);
        if (len > 0) {
            ctx->uart.write(msg, static_cast<std::size_t>(len));
        }
    }
}

} // namespace app
