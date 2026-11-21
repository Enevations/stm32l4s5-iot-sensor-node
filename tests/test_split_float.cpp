/** @file test_split_float.cpp
 *  @brief Unit tests for the telemetry float formatting logic.
 *
 * Host build (requires a host C++ toolchain):
 *   cmake -B build/host-tests -S tests -G Ninja
 *   cmake --build build/host-tests && ./build/host-tests/test_split_float
 *
 * On-target build: configure the firmware with -DSTM32_SELF_TEST=ON; the
 * suite then runs during boot and reports over the ST-LINK UART.
 */

#include <cstddef>
#include <cstdio>
#include <cstring>

#include "split_float.hpp"
#include "test_report.hpp"

namespace {

constexpr const char* kSuite = "split_float";

std::size_t passed = 0;
std::size_t failures = 0;

/// @brief Records one case; keeps only the first failure message.
void expect(const char* input_text,
            float value,
            const char* expected,
            char* error_buffer,
            std::size_t error_size) {
    const SplitFloat split = split_float(value);
    char actual[32];
    std::snprintf(actual, sizeof(actual), "%s%d.%02d",
                  split.negative ? "-" : "", split.whole, split.frac);

    const bool ok = std::strcmp(actual, expected) == 0;
    test::report_case(kSuite, input_text, ok);
    if (ok) {
        ++passed;
        return;
    }
    if (++failures == 1U && error_buffer != nullptr && error_size > 0U) {
        std::snprintf(error_buffer, error_size, "input %s: expected \"%s\" got \"%s\"",
                      input_text, expected, actual);
    }
}

} // namespace

bool run_split_float_tests(char* error_buffer, std::size_t error_size) {
    if (error_buffer != nullptr && error_size > 0U) {
        error_buffer[0] = '\0';
    }

    // Compile-time checks: split_float is constexpr.
    static_assert(split_float(-0.4f).negative, "sign must survive in (-1, 0)");
    static_assert(split_float(9.999f).whole == 10 && split_float(9.999f).frac == 0,
                  "fraction carry must roll into whole");

    // The original sign-loss bug: values in (-1, 0) must keep the sign.
    expect("-0.4", -0.4f, "-0.40", error_buffer, error_size);
    expect("-0.001", -0.001f, "-0.00", error_buffer, error_size);
    expect("-5.3", -5.3f, "-5.30", error_buffer, error_size);
    expect("-9.999", -9.999f, "-10.00", error_buffer, error_size);
    expect("-273.15", -273.15f, "-273.15", error_buffer, error_size);

    // Rounding instead of truncation.
    expect("23.999", 23.999f, "24.00", error_buffer, error_size);
    expect("9.999", 9.999f, "10.00", error_buffer, error_size);
    expect("99.999", 99.999f, "100.00", error_buffer, error_size);
    expect("0.999", 0.999f, "1.00", error_buffer, error_size);

    // Ordinary values and boundaries.
    expect("0", 0.0f, "0.00", error_buffer, error_size);
    expect("31.71", 31.71f, "31.71", error_buffer, error_size);
    expect("46.21", 46.21f, "46.21", error_buffer, error_size);
    expect("100", 100.0f, "100.00", error_buffer, error_size);

    // Invariant sweep: fraction always within [0, 99], magnitude non-negative.
    // Reported as one aggregate case: thousands of lines on the UART would
    // bury the interesting output.
    for (int i = -4000; i <= 10000; i += 7) {
        const SplitFloat split = split_float(static_cast<float>(i) / 100.0f);
        if (split.frac < 0 || split.frac > 99 || split.whole < 0) {
            char what[48];
            std::snprintf(what, sizeof(what), "invariant broken at input %d", i);
            test::report_case(kSuite, what, false);
            if (++failures == 1U && error_buffer != nullptr && error_size > 0U) {
                std::snprintf(error_buffer, error_size, "invariant broken at input %d", i);
            }
            return false;
        }
        ++passed;
    }
    test::report_case(kSuite, "invariant sweep over -40.00..100.00", true);

    return failures == 0U;
}
