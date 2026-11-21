/** @file host_main.cpp
 *  @brief Entry point for the host-built unit-test binary.
 *
 * Not compiled on the STM32 target: the on-target self-test calls each
 * run_*_tests() function directly (see STM32_SELF_TEST in CMakeLists.txt).
 */

#include <cstdio>
#include <cstddef>

#include "test_report.hpp"

bool run_split_float_tests(char* error_buffer, std::size_t error_size);
bool run_hts221_tests(char* error_buffer, std::size_t error_size);
bool run_lps22hb_tests(char* error_buffer, std::size_t error_size);
bool run_post_check_tests(char* error_buffer, std::size_t error_size);
bool run_stall_tracker_tests(char* error_buffer, std::size_t error_size);
bool run_lsm6dsl_tap_tests(char* error_buffer, std::size_t error_size);

namespace {

/// @brief Prints one PASS/FAIL line per test case as the suites run.
void printf_report(const char* suite, const char* what, bool passed) {
    std::printf("%s: %s: %s\n", passed ? "PASS" : "FAIL", suite, what);
}

} // namespace

int main() {
    test::set_reporter(printf_report);
    char error[128];
    struct Suite {
        const char* name;
        bool (*run)(char*, std::size_t);
    };
    const Suite suites[] = {
        {"split_float", run_split_float_tests},
        {"hts221", run_hts221_tests},
        {"lps22hb", run_lps22hb_tests},
        {"post_check", run_post_check_tests},
        {"stall_tracker", run_stall_tracker_tests},
        {"lsm6dsl_tap", run_lsm6dsl_tap_tests},
    };

    bool all_ok = true;
    for (const Suite& suite : suites) {
        if (!suite.run(error, sizeof(error))) {
            std::printf("FAIL: %s tests: %s\n", suite.name, error);
            all_ok = false;
        }
        else {
            std::printf("PASS: %s tests\n", suite.name);
        }
    }
    return all_ok ? 0 : 1;
}
