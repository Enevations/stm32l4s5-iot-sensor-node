/** @file main.cpp
 *  @brief Firmware entry point for the STM32L4S5I-IOT01A IoT sensor node.
 *
 * Four FreeRTOS tasks cooperate through a telemetry queue: a heartbeat task
 * (LED and stack-margin reports), a sensor task reading the onboard HTS221
 * humidity/temperature and LPS22HB pressure sensors over I2C2, a telemetry
 * task printing readings over the ST-LINK virtual COM port, and a tap task
 * polling the LSM6DSL hardware double-tap detector on the same bus.
 */

#include <cstdio>
#include <cstring>
#include <atomic>

#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c_bus.hpp"
#include "post_check.hpp"
#include "uart.hpp"
#include "led.hpp"
#include "sensor_interface.hpp"
#include "hts221_sensor.hpp"
#include "lps22hb_sensor.hpp"
#include "lsm6dsl_tap.hpp"
#include "self_test.hpp"
#include "split_float.hpp"
#include "stall_tracker.hpp"

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

/// @brief One telemetry queue item: a merged sensor reading plus its stamp.
struct TelemetryMessage {
    drivers::EnvironmentalSample sample;
    uint32_t timestamp_ms{0};
};

struct AppContext {
    bsp::I2cBus i2c_bus;
    drivers::Hts221Sensor hts221{i2c_bus};
    drivers::Lps22hbSensor lps22hb{i2c_bus};
    drivers::Lsm6dslTap lsm6dsl_tap{i2c_bus};
    rtos::Queue<TelemetryMessage> telemetry_queue{8};
};

static constexpr Led green_led(GPIOB_BASE, GPIO_PIN_14);
static Uart uart1(USART1, GPIOB_BASE, GPIO_PIN_6, GPIO_PIN_7, GPIO_AF7_USART1, 115200);
inline std::atomic<BoardState> gboard_state{BoardState::Healthy};

/// Native task handles mirrored from main()'s rtos::Task objects so heartbeat()
/// can query each task's stack high-water mark; a null entry makes the report
/// fall back to heartbeat's own stack for that slot. Slots: 0 heartbeat,
/// 1 sensor, 2 telemetry, 3 tap.
static TaskHandle_t g_task_handles[4]{};

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
 *   arm-none-eabi-addr2line -e build/debug/stm32l4s5-iot-sensor-node.elf -f -C 0x<PC>
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
 * Called from HardFault_Handler's inline assembly, which the optimizer cannot
 * see: used + externally_visible keep the symbol alive and external under
 * -flto release builds.
 *
 * @param frame Basic exception frame: r0, r1, r2, r3, r12, lr, pc, xpsr.
 * @param exc_return EXC_RETURN value captured at handler entry (link register).
 */
extern "C" [[gnu::used, gnu::externally_visible, noreturn]] void hard_fault_report(uint32_t* frame, uint32_t exc_return) {
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
/**
 * @brief Configures the MSI/PLL clock tree for an 80 MHz system clock.
 *
 * @return true on success; false when any stage failed, in which case the
 *         MCU keeps running on the default 4 MHz MSI and the caller should
 *         flag the degraded state.
 */
static bool system_clock_config() {
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
 * The probing logic itself lives in bsp::run_post_check, which is unit-tested
 * against a fake bus. Sets the global hardware-fault state when the bus or
 * any expected device fails to respond; the heartbeat task signals the fault
 * with fast blinks.
 *
 * @param ctx Application context owning the I2C bus.
 */
static void i2c_scan_check(AppContext& ctx) {
    if (!ctx.i2c_bus.init()) {
        uart1.write("I2C2 init failed\n");
        gboard_state.store(BoardState::HardwareFault, std::memory_order_release);
        return;
    }

    const bsp::PostResult post = bsp::run_post_check(ctx.i2c_bus,
                                                     bsp::kExpectedI2CDevices.data(),
                                                     bsp::kExpectedI2CDevices.size());
    if (post.all_found) {
        uart1.write("[Post Success] All expected devices found\n");
        return;
    }

    char msg[64];
    std::snprintf(msg, sizeof(msg), "[Post Fail] Missing device at 0x%02X\n",
                  post.missing_address);
    uart1.write(msg, std::strlen(msg));
    uart1.write("[Post Fail] One or more expected devices missing\n");
    gboard_state.store(BoardState::HardwareFault, std::memory_order_release);
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
        const unsigned long tp =
            static_cast<unsigned long>(uxTaskGetStackHighWaterMark(g_task_handles[3]));
        const int len = std::snprintf(msg, sizeof(msg),
                                      "Stack free words: HB=%lu SN=%lu TM=%lu TP=%lu\r\n",
                                      hb, sn, tm, tp);
        if (len > 0) {
            uart1.write(msg, static_cast<std::size_t>(len));
        }
    }
}

/**
 * @brief Maps a driver init failure to a human-readable reason.
 */
static const char* init_status_text(drivers::InitStatus status) noexcept {
    switch (status) {
        case drivers::InitStatus::Ok:                return "ok";
        case drivers::InitStatus::BusError:          return "bus error";
        case drivers::InitStatus::WhoAmIMismatch:    return "WHO_AM_I mismatch";
        case drivers::InitStatus::InvalidCalibration: return "invalid calibration";
    }
    return "unknown";
}

/**
 * @brief Brings every registered sensor up and samples them once every
 * five seconds.
 *
 * Each sensor initializes independently with its own retry backoff (2 s -> 4 s
 * -> 8 s cap): one failed sensor never blocks the others, and recovery clears
 * the fault flag only when every sensor this task flagged is up again. Ready
 * sensors are read each cycle into a merged sample (last sensor wins on
 * shared channels such as temperature), which is queued for the telemetry
 * task with a bounded wait: a stalled telemetry task drops samples (logged
 * with a running total) instead of freezing sampling. Failed reads are
 * logged and retried on the next cycle.
 *
 * Adding a sensor means implementing drivers::ISensor and adding it to the
 * list below — this task never changes.
 *
 * @param raw_ctx Pointer to the AppContext owning the sensors and queue.
 */
static void sensor_task(void* raw_ctx) {
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
                    uart1.write(msg, static_cast<std::size_t>(len));
                }
            }
            else {
                fault_flagged = true;
                gboard_state.store(BoardState::HardwareFault, std::memory_order_release);
                const uint32_t backoff_ms = 1000U << ((attempts[i] < 3U) ? attempts[i] : 3U);
                next_try_ms[i] = now + backoff_ms;
                const int len = std::snprintf(msg, sizeof(msg),
                    "Sensor 0x%02X init failed (attempt %lu: %s); retrying in %lu s\r\n",
                    sensors[i]->i2c_address(),
                    static_cast<unsigned long>(attempts[i]),
                    init_status_text(status),
                    static_cast<unsigned long>(backoff_ms / 1000U));
                if (len > 0) {
                    uart1.write(msg, static_cast<std::size_t>(len));
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
                gboard_state.store(BoardState::Healthy, std::memory_order_release);
                uart1.write("All sensors ready\r\n");
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
                    uart1.write(log_line, static_cast<std::size_t>(len));
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
                        uart1.write(log_line, static_cast<std::size_t>(len));
                    }
                }
            }
            else {
                const uint32_t total = stall.note_dropped();
                const int len = std::snprintf(log_line, sizeof(log_line),
                    "Telemetry queue full; sample dropped (total %lu)\r\n",
                    static_cast<unsigned long>(total));
                if (len > 0) {
                    uart1.write(log_line, static_cast<std::size_t>(len));
                }
            }
        }
        rtos::sleep_ms(5000);
    }
}

/**
 * @brief Reports double taps detected by the LSM6DSL hardware.
 *
 * The chip recognizes the gesture itself (slope threshold plus quiet/shock/
 * duration windows configured in drivers::Lsm6dslTap::init per ST AN5040),
 * so this task only polls the TAP_SRC register every 20 ms and prints a
 * message each time a double-tap event is consumed. The configuration is
 * unlatched, so the DOUBLE_TAP flag self-clears after the Quiet window
 * (~29 ms) and the poll reads inside that window. Init follows the same
 * retry backoff as the environmental sensors; a bus failure during polling
 * drops back into that reinit loop instead of spinning on errors.
 *
 * @param raw_ctx Pointer to the AppContext owning the tap driver.
 */
static void tap_task(void* raw_ctx) {
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
                    uart1.write(msg, static_cast<std::size_t>(len));
                }
            }
            else {
                fault_flagged = true;
                gboard_state.store(BoardState::HardwareFault, std::memory_order_release);
                const uint32_t backoff_ms = 1000U << ((attempts < 3U) ? attempts : 3U);
                next_try_ms = now + backoff_ms;
                const int len = std::snprintf(msg, sizeof(msg),
                    "LSM6DSL 0x%02X tap init failed (attempt %lu: %s); retrying in %lu s\r\n",
                    ctx->lsm6dsl_tap.i2c_address(),
                    static_cast<unsigned long>(attempts),
                    init_status_text(status),
                    static_cast<unsigned long>(backoff_ms / 1000U));
                if (len > 0) {
                    uart1.write(msg, static_cast<std::size_t>(len));
                }
            }
        }

        // Same clearing rule as sensor_task: only faults this task raised.
        if (fault_flagged && ready) {
            fault_flagged = false;
            gboard_state.store(BoardState::Healthy, std::memory_order_release);
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
                    uart1.write(msg, static_cast<std::size_t>(len));
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
                    uart1.write(msg, static_cast<std::size_t>(len));
                }
            }
        }
        else if (event == drivers::TapEvent::BusError) {
            ready = false;
            fault_flagged = true;
            gboard_state.store(BoardState::HardwareFault, std::memory_order_release);
            const uint32_t backoff_ms = 1000U << ((attempts < 3U) ? attempts : 3U);
            next_try_ms = HAL_GetTick() + backoff_ms;
            uart1.write("LSM6DSL tap source read failed; reinitializing\r\n");
        }

        rtos::sleep_ms(20);
    }
}

/**
 * @brief Prints one telemetry line for a single sample channel.
 *
 * @param stamp_ms Uptime of the reading cycle.
 * @param label Channel label, e.g. "Temp".
 * @param unit Unit suffix, e.g. "C".
 * @param value Channel value, printed with two fractional digits.
 */
static void print_channel(uint32_t stamp_ms, const char* label, const char* unit, float value) {
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
        uart1.write(msg, static_cast<std::size_t>(len));
    }
}

/**
 * @brief Prints queued sensor readings as text lines over UART1.
 *
 * Blocks on the telemetry queue and prints one line per valid channel of the
 * merged sample, formatting values with integer-only math (see
 * include/split_float.hpp) to avoid pulling the newlib float formatter into
 * the firmware image.
 *
 * @param raw_ctx Pointer to the AppContext owning the telemetry queue.
 */
static void telemetry_task(void* raw_ctx) {
    auto* ctx = static_cast<AppContext*>(raw_ctx);
    TelemetryMessage msg{};

    for(;;) {
        if (ctx->telemetry_queue.receive(msg)) {
            if (msg.sample.valid_temperature) {
                print_channel(msg.timestamp_ms, "Temp", "C", msg.sample.temperature_c);
            }
            if (msg.sample.valid_humidity) {
                print_channel(msg.timestamp_ms, "Humidity", "%", msg.sample.humidity_pct);
            }
            if (msg.sample.valid_pressure) {
                print_channel(msg.timestamp_ms, "Pressure", "hPa", msg.sample.pressure_hpa);
            }
        }
    }
}

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

    const bool clock_ok = system_clock_config();
    ports_init();

    if (!clock_ok) {
        // Boot continues on the MSI fallback clock: HAL derives the UART
        // baud and I2C timing from the actual clock, and the heartbeat task
        // signals the degraded state with fast blinks.
        gboard_state.store(BoardState::HardwareFault, std::memory_order_release);
        uart1.write("\r\n[WARN] Clock config failed; running on fallback clock\r\n");
    }

#ifdef STM32_SELF_TEST
    app::run_self_tests(uart1);
#endif

    // Static storage duration is mandatory: the FreeRTOS Cortex-M4F port
    // resets MSP from the vector table when the first task starts, so any
    // object referenced by tasks must not live in main's stack frame.
    static AppContext app{};
    g_app_context = &app;

    i2c_scan_check(app);

    rtos::Task heartbeat_task("Heartbeat", 512, tskIDLE_PRIORITY + 2, heartbeat, &app);
    rtos::Task sensor_task_handle("SensorTask", 1024, tskIDLE_PRIORITY + 1, sensor_task, &app);
    rtos::Task telemetry_task_handle("TelemetryTask", 1024, tskIDLE_PRIORITY + 1, telemetry_task, &app);
    rtos::Task tap_task_handle("TapTask", 512, tskIDLE_PRIORITY + 1, tap_task, &app);

    // Mirror the task handles into g_task_handles for heartbeat()'s stack
    // report: the handles live in main()'s rtos::Task locals, which heartbeat
    // cannot reach. Must run after task creation (xTaskCreate writes them)
    // and before vTaskStartScheduler(), so no race is possible.
    g_task_handles[0] = heartbeat_task.handle();
    g_task_handles[1] = sensor_task_handle.handle();
    g_task_handles[2] = telemetry_task_handle.handle();
    g_task_handles[3] = tap_task_handle.handle();

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