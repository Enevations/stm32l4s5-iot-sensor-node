#pragma once

/** @file tasks.hpp
 *  @brief Entry points of the four FreeRTOS application tasks.
 *
 * Each entry point receives a pointer to the AppContext created in main()
 * and never returns. The bodies live in src/heartbeat.cpp,
 * src/environment_tasks.cpp, and src/tap_task.cpp.
 */

namespace app {

/** @brief Toggles the status LED at a regular interval and reports per-task
 *         stack margins over the UART. */
void heartbeat(void* raw_ctx);

/** @brief Brings every registered sensor up with retry backoff and samples
 *         them once every five seconds into the telemetry queue. */
void sensor_task(void* raw_ctx);

/** @brief Prints queued sensor readings as text lines over the UART. */
void telemetry_task(void* raw_ctx);

/** @brief Reports double taps detected by the LSM6DSL hardware. */
void tap_task(void* raw_ctx);

} // namespace app
