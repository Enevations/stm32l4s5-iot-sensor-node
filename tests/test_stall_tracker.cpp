/** @file test_stall_tracker.cpp
 *  @brief Unit tests for the telemetry queue drop/recovery policy.
 *
 * Runs on host (via tests/host_main.cpp) and on target (STM32_SELF_TEST).
 */

#include <cstdio>
#include <cstddef>
#include <cstdint>

#include "stall_tracker.hpp"

namespace {

std::size_t passed = 0;
std::size_t failures = 0;

void expect_true(bool condition, const char* what, char* error_buffer, std::size_t error_size) {
    if (condition) {
        ++passed;
    }
    else if (++failures == 1U && error_buffer != nullptr && error_size > 0U) {
        std::snprintf(error_buffer, error_size, "%s", what);
    }
}

} // namespace

bool run_stall_tracker_tests(char* error_buffer, std::size_t error_size) {
    if (error_buffer != nullptr && error_size > 0U) {
        error_buffer[0] = '\0';
    }

    // Fresh tracker: nothing dropped, no recovery pending.
    {
        telemetry::StallTracker stall;
        expect_true(stall.consecutive_drops() == 0, "fresh tracker reports zero drops",
                    error_buffer, error_size);
    }

    // Consecutive drops accumulate with a running total.
    {
        telemetry::StallTracker stall;
        const uint32_t first = stall.note_dropped();
        const uint32_t second = stall.note_dropped();
        const uint32_t third = stall.note_dropped();
        expect_true(first == 1 && second == 2 && third == 3,
                    "consecutive drops count 1, 2, 3", error_buffer, error_size);
        expect_true(stall.consecutive_drops() == 3, "total matches after three drops",
                    error_buffer, error_size);
    }

    // A successful send resets the count; the next stall starts fresh.
    {
        telemetry::StallTracker stall;
        stall.note_dropped();
        stall.note_dropped();
        stall.note_enqueued();
        expect_true(stall.consecutive_drops() == 0, "recovery resets the drop count",
                    error_buffer, error_size);
        expect_true(stall.note_dropped() == 1, "next stall starts at one",
                    error_buffer, error_size);
    }

    // Long stall: counter does not wrap or plateau over a realistic horizon.
    {
        telemetry::StallTracker stall;
        for (uint32_t i = 1; i <= 1000U; ++i) {
            const uint32_t total = stall.note_dropped();
            if (total != i) {
                expect_true(false, "counter tracks a 1000-sample stall",
                            error_buffer, error_size);
                return failures == 0U;
            }
        }
        expect_true(stall.consecutive_drops() == 1000, "1000-sample stall tracked",
                    error_buffer, error_size);
    }

    // Interleaved stalls never mix a prior stall into the recovery count.
    {
        telemetry::StallTracker stall;
        stall.note_dropped();
        stall.note_dropped();
        const uint32_t lost_first_stall = stall.consecutive_drops();
        stall.note_enqueued();
        stall.note_dropped();
        expect_true(stall.consecutive_drops() == 1 && lost_first_stall == 2,
                    "stalls stay isolated across recovery", error_buffer, error_size);
    }

    return failures == 0U;
}
