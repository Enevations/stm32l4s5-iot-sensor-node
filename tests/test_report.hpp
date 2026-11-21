#pragma once

/** @file test_report.hpp
 *  @brief Per-case pass/fail reporting shared by the test suites.
 *
 * Suites call report_case() for every case so a failing run shows exactly
 * which case failed, where it happened. The output sink is installed by the
 * entry point: printf in the host binary (tests/host_main.cpp) and the
 * ST-LINK UART in the on-target self-test (src/main.cpp). Without an
 * installed reporter the suites stay silent and keep their historical
 * error-buffer-only behavior.
 */

#include <cstdint>

namespace test {

/** @brief Signature of the per-case reporter installed by the entry point. */
using ReportFn = void (*)(const char* suite, const char* what, bool passed);

/// Single reporter slot shared by all translation units (C++17 inline variable).
inline ReportFn reporter_{nullptr};

/** @brief Installs the reporter used by report_case(); nullptr restores silence. */
inline void set_reporter(ReportFn reporter) noexcept {
    reporter_ = reporter;
}

/** @brief Reports one test case outcome to the installed reporter. */
inline void report_case(const char* suite, const char* what, bool passed) noexcept {
    if (reporter_ != nullptr) {
        reporter_(suite, what, passed);
    }
}

} // namespace test
