/** @file environment_tasks.cpp
 *  @brief Sensor sampling task and telemetry printing task.
 */

#include <cstdio>
#include <cstddef>

#include "FreeRTOS.h"
#include "stm32l4xx_hal.h"

#include "app/app_context.hpp"
#include "bsp/board_state.hpp"
#include "rtos/rtos.hpp"
#include "drivers/sensor_interface.hpp"
#include "split_float.hpp"
#include "stall_tracker.hpp"
#include "app/tasks.hpp"

namespace app {
namespace {

/**
 * @brief Prints one telemetry line for a single sample channel.
 *
 * @param uart UART to print on.
 * @param stamp_ms Uptime of the reading cycle.
 * @param label Channel label, e.g. "Temp".
 * @param unit Unit suffix, e.g. "C".
 * @param value Channel value, printed with two fractional digits.
 */
void print_channel(Uart& uart, uint32_t stamp_ms, const char* label, const char* unit, float value) {
    const SplitFloat split = split_float(value);
    char msg[96];
    const int len = std::snprintf(msg, sizeof(msg), "[%lu ms] %s: %s%d.%02d %s\r\n",
                                  static_cast<unsigned long>(stamp_ms),
                                  label,
                                  split.negative ? "-" : "",
                                  split.whole,
                                  split.frac,
                                  unit);
    if (len > 0) {
        uart.write(msg, static_cast<std::size_t>(len));
    }
}

} // namespace

void sensor_task(void* raw_ctx) {
    auto* ctx = static_cast<AppContext*>(raw_ctx);

    constexpr std::size_t kSensorCount = 2;
    drivers::ISensor* const sensors[kSensorCount] = { &ctx->hts221, &ctx->lps22hb };

    bool ready[kSensorCount]{};
    uint32_t attempts[kSensorCount]{};
    uint32_t next_try_ms[kSensorCount]{};
    bool fault_flagged = false;
    telemetry::StallTracker stall;

    char msg[128];
    char log_line[96];

    for(;;) {
        const uint32_t now = HAL_GetTick();

        // Bring up any sensor whose retry window has elapsed.
        for (std::size_t i = 0; i < kSensorCount; ++i) {
            if (ready[i] || now < next_try_ms[i]) {
                continue;
            }
            ++attempts[i];
            const auto status = sensors[i]->init();
            if (status == drivers::InitStatus::Ok) {
                ready[i] = true;
                const int len = std::snprintf(msg, sizeof(msg),
                    "Sensor 0x%02X ready (attempt %lu)\r\n",
                    sensors[i]->i2c_address(), static_cast<unsigned long>(attempts[i]));
                if (len > 0) {
                    ctx->uart.write(msg, static_cast<std::size_t>(len));
                }
            }
            else {
                fault_flagged = true;
                bsp::set_hardware_fault();
                const uint32_t backoff_ms = 1000U << ((attempts[i] < 3U) ? attempts[i] : 3U);
                next_try_ms[i] = now + backoff_ms;
                const int len = std::snprintf(msg, sizeof(msg),
                    "Sensor 0x%02X init failed (attempt %lu: %s); retrying in %lu s\r\n",
                    sensors[i]->i2c_address(),
                    static_cast<unsigned long>(attempts[i]),
                    drivers::init_status_text(status),
                    static_cast<unsigned long>(backoff_ms / 1000U));
                if (len > 0) {
                    ctx->uart.write(msg, static_cast<std::size_t>(len));
                }
            }
        }

        // Clear the fault only when every sensor this task flagged is up, so
        // POST or clock failures reported elsewhere are not masked.
        if (fault_flagged) {
            bool all_ready = true;
            for (std::size_t i = 0; i < kSensorCount; ++i) {
                all_ready = all_ready && ready[i];
            }
            if (all_ready) {
                fault_flagged = false;
                bsp::clear_hardware_fault();
                ctx->uart.write("All sensors ready\r\n");
            }
        }

        // Sample every ready sensor into one merged reading.
        drivers::EnvironmentalSample merged{};
        bool any_channel = false;
        const uint32_t stamp = HAL_GetTick();
        for (std::size_t i = 0; i < kSensorCount; ++i) {
            if (!ready[i]) {
                continue;
            }
            const auto part = sensors[i]->read();
            if (part.valid_temperature) {
                merged.temperature_c = part.temperature_c;
                merged.valid_temperature = true;
                any_channel = true;
            }
            if (part.valid_humidity) {
                merged.humidity_pct = part.humidity_pct;
                merged.valid_humidity = true;
                any_channel = true;
            }
            if (part.valid_pressure) {
                merged.pressure_hpa = part.pressure_hpa;
                merged.valid_pressure = true;
                any_channel = true;
            }
            if (!part.valid_temperature && !part.valid_humidity && !part.valid_pressure) {
                const int len = std::snprintf(log_line, sizeof(log_line),
                    "Sensor 0x%02X read failed; retrying next cycle\r\n",
                    sensors[i]->i2c_address());
                if (len > 0) {
                    ctx->uart.write(log_line, static_cast<std::size_t>(len));
                }
            }
        }

        if (any_channel) {
            TelemetryMessage msg{merged, stamp};
            // A stalled telemetry task must not freeze sampling: wait at most
            // 2 s for queue space, then drop the sample and keep measuring.
            if (ctx->telemetry_queue.send(msg, pdMS_TO_TICKS(2000))) {
                const uint32_t dropped = stall.consecutive_drops();
                stall.note_enqueued();
                if (dropped > 0) {
                    const int len = std::snprintf(log_line, sizeof(log_line),
                        "Telemetry queue recovered (dropped %lu samples)\r\n",
                        static_cast<unsigned long>(dropped));
                    if (len > 0) {
                        ctx->uart.write(log_line, static_cast<std::size_t>(len));
                    }
                }
            }
            else {
                const uint32_t total = stall.note_dropped();
                const int len = std::snprintf(log_line, sizeof(log_line),
                    "Telemetry queue full; sample dropped (total %lu)\r\n",
                    static_cast<unsigned long>(total));
                if (len > 0) {
                    ctx->uart.write(log_line, static_cast<std::size_t>(len));
                }
            }
        }
        rtos::sleep_ms(5000);
    }
}

void telemetry_task(void* raw_ctx) {
    auto* ctx = static_cast<AppContext*>(raw_ctx);
    TelemetryMessage msg{};

    for(;;) {
        if (ctx->telemetry_queue.receive(msg)) {
            if (msg.sample.valid_temperature) {
                print_channel(ctx->uart, msg.timestamp_ms, "Temp", "C", msg.sample.temperature_c);
            }
            if (msg.sample.valid_humidity) {
                print_channel(ctx->uart, msg.timestamp_ms, "Humidity", "%", msg.sample.humidity_pct);
            }
            if (msg.sample.valid_pressure) {
                print_channel(ctx->uart, msg.timestamp_ms, "Pressure", "hPa", msg.sample.pressure_hpa);
            }
        }
    }
}

} // namespace app
