/** @file fault_report.cpp
 *  @brief Fail-stop hooks, hard-fault context reporting, and kernel-object
 *         dumps.
 *
 * Every path prints the cause over the registered UART before halting. The
 * hooks must never acquire a mutex (see the SchedulerLock masking rules in
 * rtos.hpp and finding #4 in docs/findings.md): they only write through the
 * registered UART, whose own lock already skips locking when masked.
 */

#include "fault_report.hpp"

#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32l4xx_hal.h"

namespace bsp {

namespace {

// Set once by register_fault_diagnostics(); the hooks are invoked by the
// kernel with no context parameter, so they reach the diagnostics through
// these pointers. Null until registration: hooks firing before it (e.g.
// heap exhaustion during early task creation) stay silent by design.
Uart* s_uart = nullptr;
const void* s_i2c_mutex_handle = nullptr;
const void* s_telemetry_queue_handle = nullptr;

/// @brief Hex-dumps a kernel object so corruption can be compared over time.
void dump_kernel_object(const char* tag, const void* object) {
    char line[72];
    const int header_len = std::snprintf(line, sizeof(line), "[%s] @%p:", tag, object);
    if (header_len > 0 && s_uart != nullptr) {
        s_uart->write(line, static_cast<std::size_t>(header_len));
    }
    const auto* bytes = static_cast<const uint8_t*>(object);
    for (std::size_t i = 0; i < 64U; ++i) {
        char hex[8];
        const int len = std::snprintf(hex, sizeof(hex), " %02X", bytes[i]);
        if (len > 0 && s_uart != nullptr) {
            s_uart->write(hex, static_cast<std::size_t>(len));
        }
    }
    if (s_uart != nullptr) {
        s_uart->write("\r\n", 2U);
    }
}

/// @brief Dumps the three kernel objects involved in the crash for comparison.
void dump_rtos_objects(const char* when) {
    if (s_uart == nullptr) {
        return;
    }
    char header[64];
    const int len = std::snprintf(header, sizeof(header), "[DUMP] %s\r\n", when);
    if (len > 0) {
        s_uart->write(header, static_cast<std::size_t>(len));
    }
    dump_kernel_object("uart_mutex", s_uart->mutex().handle());
    if (s_i2c_mutex_handle != nullptr) {
        dump_kernel_object("i2c_mutex", s_i2c_mutex_handle);
    }
    if (s_telemetry_queue_handle != nullptr) {
        dump_kernel_object("telemetry_q", s_telemetry_queue_handle);
    }
}

} // namespace

/// @brief Internal-linkage access for the C hooks defined below this block.
static Uart* fault_uart() noexcept {
    return s_uart;
}

void register_fault_diagnostics(Uart& uart,
                                const void* i2c_mutex_handle,
                                const void* telemetry_queue_handle) noexcept {
    s_uart = &uart;
    s_i2c_mutex_handle = i2c_mutex_handle;
    s_telemetry_queue_handle = telemetry_queue_handle;
}

} // namespace bsp

// --- Fail-Stop Hooks ---
// C linkage: called by the FreeRTOS kernel and the vector table.
extern "C" {

/** @brief Fail-stop hook invoked when FreeRTOS cannot allocate memory. */
void vApplicationMallocFailedHook() {
    taskDISABLE_INTERRUPTS();
    static const char msg[] = "\r\n[FATAL] FreeRTOS heap exhausted (malloc failed hook)\r\n";
    Uart* uart = bsp::fault_uart();
    if (uart != nullptr) {
        uart->write(msg);
    }
    while (true) {}
}

/** @brief Fail-stop hook invoked when a task overflow is detected. */
void vApplicationStackOverflowHook(TaskHandle_t, char* task_name) {
    taskDISABLE_INTERRUPTS();
    char msg[80];
    const int len = std::snprintf(msg, sizeof(msg),
                                  "\r\n[FATAL] Stack overflow in task: %s\r\n",
                                  (task_name != nullptr) ? task_name : "?");
    Uart* uart = bsp::fault_uart();
    if (len > 0 && uart != nullptr) {
        uart->write(msg, static_cast<std::size_t>(len));
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
[[gnu::used, gnu::externally_visible, noreturn]] void hard_fault_report(uint32_t* frame, uint32_t exc_return) {
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
    Uart* uart = bsp::fault_uart();
    if (len > 0 && uart != nullptr) {
        uart->write(msg, static_cast<std::size_t>(len));
    }
    bsp::dump_rtos_objects("at fault");
    while (true) {}
}

} // extern "C"
