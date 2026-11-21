/** @file self_test.cpp
 *  @brief On-target self-test harness: runs every unit-test suite and the
 *  real-kernel queue probe during boot, reporting per case over the UART.
 *
 * Compiled into the firmware only when CMake is configured with
 * -DSTM32_SELF_TEST=ON; the suites themselves live in tests/ and are
 * shared with the host unit-test binary.
 */

#include "self_test.hpp"

#include <cstdio>
#include <cstring>

#include "rtos.hpp"

#ifdef STM32_SELF_TEST

#include "test_report.hpp"

// Provided by tests/*.cpp, compiled into the firmware when CMake is
// configured with -DSTM32_SELF_TEST=ON.
extern bool run_split_float_tests(char* error_buffer, std::size_t error_size);
extern bool run_hts221_tests(char* error_buffer, std::size_t error_size);
extern bool run_lps22hb_tests(char* error_buffer, std::size_t error_size);
extern bool run_post_check_tests(char* error_buffer, std::size_t error_size);
extern bool run_stall_tracker_tests(char* error_buffer, std::size_t error_size);
extern bool run_lsm6dsl_tap_tests(char* error_buffer, std::size_t error_size);

namespace app {
namespace {

// Installed once by run_self_tests(); the test::ReportFn signature carries
// no context, so the reporter reaches the UART through this pointer. The
// harness runs pre-scheduler from a single call site, so no guarding needed.
Uart* s_report_uart = nullptr;

/// @brief Per-case self-test reporter: prints each case over the UART as it
/// runs, so a target failure names the exact case instead of only the first.
void uart_report_case(const char* suite, const char* what, bool passed) {
    char line[128];
    const int len = std::snprintf(line, sizeof(line), "[Self-test] %s %s: %s\r\n",
                                  passed ? "PASS" : "FAIL", suite, what);
    if (len > 0 && s_report_uart != nullptr) {
        s_report_uart->write(line, static_cast<std::size_t>(len));
    }
}

/// @brief Runs one suite, printing its [FATAL] line on failure.
void run_suite(Uart& uart,
               bool& self_test_ok,
               const char* name,
               bool (*suite)(char*, std::size_t)) {
    char self_test_error[128];
    if (!suite(self_test_error, sizeof(self_test_error))) {
        self_test_ok = false;
        uart.write("[FATAL] Self-test ");
        uart.write(name);
        uart.write(" FAILED: ");
        uart.write(self_test_error, std::strlen(self_test_error));
        uart.write("\r\n");
    }
}

/// @brief Real-queue probe: a full queue must reject a zero-tick send and
/// draining it must make room again — the bounded-send mechanism the drop
/// policy in sensor_task relies on. The single slot is filled first; the
/// rejection check must run against a full queue.
bool run_queue_probe(Uart& uart) {
    rtos::Queue<uint32_t> probe_queue(1);
    const uint32_t value = 42;
    uint32_t drained = 0;
    const bool fill_ok = probe_queue.send(value, 0);
    const bool full_rejected = !probe_queue.send(value, 0);
    const bool receive_ok = probe_queue.receive(drained, 0) && drained == value;
    const bool refill_ok = probe_queue.send(value, 0);
    test::report_case("queue_probe", "zero-tick send fills the empty slot", fill_ok);
    test::report_case("queue_probe", "full queue rejects zero-tick send", full_rejected);
    test::report_case("queue_probe", "drain returns the sent value", receive_ok);
    test::report_case("queue_probe", "drained slot accepts again", refill_ok);
    if (!fill_ok || !full_rejected || !receive_ok || !refill_ok) {
        uart.write("[FATAL] Self-test queue probe FAILED\r\n");
        return false;
    }
    return true;
}

} // namespace

void run_self_tests(Uart& uart) noexcept {
    s_report_uart = &uart;
    test::set_reporter(uart_report_case);

    bool self_test_ok = true;
    run_suite(uart, self_test_ok, "split_float", run_split_float_tests);
    run_suite(uart, self_test_ok, "hts221", run_hts221_tests);
    run_suite(uart, self_test_ok, "lps22hb", run_lps22hb_tests);
    run_suite(uart, self_test_ok, "post_check", run_post_check_tests);
    run_suite(uart, self_test_ok, "stall_tracker", run_stall_tracker_tests);
    run_suite(uart, self_test_ok, "lsm6dsl_tap", run_lsm6dsl_tap_tests);

    if (!run_queue_probe(uart)) {
        self_test_ok = false;
    }

    if (!self_test_ok) {
        while (true) {}
    }
    uart.write("Self-test PASS (split_float, hts221, lps22hb, post_check, stall_tracker, lsm6dsl_tap, queue probe)\r\n");
}

} // namespace app

#endif // STM32_SELF_TEST
