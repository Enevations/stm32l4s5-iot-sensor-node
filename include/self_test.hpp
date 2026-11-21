#pragma once

/** @file self_test.hpp
 *  @brief On-target self-test harness for STM32_SELF_TEST builds.
 */

#include "uart.hpp"

namespace app {

/**
 * @brief Runs every unit-test suite plus the real-kernel queue probe,
 * printing one PASS/FAIL line per test case over the given UART.
 *
 * Defined only when the firmware is configured with -DSTM32_SELF_TEST=ON
 * (the build then also compiles the tests/ suites into the image). A failing
 * suite prints its case after a [FATAL] line; the firmware halts after the
 * last suite when any suite failed. Runs before the scheduler starts, so
 * the real-queue probe exercises the kernel from single-threaded context.
 *
 * @param uart UART used for all progress and failure output.
 */
void run_self_tests(Uart& uart) noexcept;

} // namespace app
