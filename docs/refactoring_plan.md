# Refactoring Plan: main.cpp Decomposition

Track of the planned and completed refactoring of `src/main.cpp`, which had
grown to 831 lines mixing five responsibilities. Update this file whenever a
tier is started or completed so future sessions can pick up where the last
one stopped.

Last updated: 2026-11-21 (branch `feat/improvements`)

## Goal

Reduce `src/main.cpp` to a ~130-line composition root (HAL init → clock →
POST → static `AppContext` → task creation → scheduler start) by moving each
cohesive block into its own `include/` + `src/` module pair, following the
existing `bsp::` (board support) and `drivers::` (device driver) conventions.

## Result (all tiers complete, 2026-11-21)

| File | Lines |
|---|---|
| `src/main.cpp` | 141 |
| `src/environment_tasks.cpp` | 196 |
| `src/tap_task.cpp` | 128 |
| `src/fault_report.cpp` | 179 |
| `src/self_test.cpp` | 114 |
| `src/clock.cpp` | 78 |
| `src/heartbeat.cpp` | 53 |
| `src/board_state.cpp` | 36 |
| `include/app_context.hpp` | 57 |
| `include/tasks.hpp` | 27 |
| `include/clock.hpp` | 46 |
| `include/fault_report.hpp` | 33 |
| `include/board_state.hpp` | 24 |
| `include/self_test.hpp` | 25 |

## Starting point (before Tier 1)

| Block in main.cpp | ~Lines | Destination |
|---|---|---|
| `main()` composition root | 130 | stays |
| `sensor_task` + `init_status_text` | 140 | `src/environment_tasks.cpp` |
| `tap_task` | 120 | `src/tap_task.cpp` |
| Self-test block + reporter | 90 | `src/self_test.cpp` |
| Fail-stop hooks + hard-fault dump | 115 | `src/fault_report.cpp` |
| `system_clock_config` + `i2c_scan_check` | 65 | `src/clock.cpp` |
| heartbeat + telemetry_task + print_channel | 90 | `src/environment_tasks.cpp` |
| Shared state (`AppContext`, `TelemetryMessage`, `BoardState`, `uart1`, `green_led`, globals) | 60 | `include/app_context.hpp`, `src/board.cpp` |

## Contracts that must be preserved in every tier

1. `static AppContext app{}` storage duration in `main()` — the Cortex-M4F
   port resets MSP from the vector table when the first task starts, so any
   object referenced by tasks must not live in main's stack frame (finding #2
   in findings.md).
2. Exact hook names/signatures: `vApplicationTickHook`,
   `vApplicationMallocFailedHook`, `vApplicationStackOverflowHook`,
   `HardFault_Handler`, and `hard_fault_report` with its
   `gnu::used` + `gnu::externally_visible` + `noreturn` attributes (the asm
   branch target must stay alive under `-flto`; finding from the CI gap row
   in findings.md).
3. The fault-report path stays mutex-free (the `SchedulerLock` masking rules
   in `include/rtos.hpp`, finding #4) — moved code must not introduce a lock
   acquisition before UART prints in fault context.
4. Task entries keep the `void (*)(void*)` signature receiving `&app`.
5. CMake globs `src/*.cpp` automatically (`CONFIGURE_DEPENDS`), and host
   tests never compile `main.cpp` — but an *existing* build directory may
   keep a stale source list when a brand-new file appears; re-run
   `cmake -B build/<variant> ...` for that variant after adding files
   (bit us in Tier 1: `build/selftest` needed a reconfigure).
6. The fault-time diagnostics (`g_app_context`, `uart1`) are file-static in
   `main.cpp` today; tiers that move their consumers must introduce the
   `bsp::board` accessors in the same commit so nothing dangles.

## Verification protocol (run after every tier)

1. `cmake --build build/debug` and `build/release` — zero project warnings
   (only the four pre-existing newlib `_close/_read/_write/_lseek` linker
   notices).
2. `cmake --build build/selftest` — self-test variant links.
3. `cmake --build build/host-tests && ./build/host-tests/unit_tests` — all
   suites pass.
4. Flash `dist/<variant>/` (refreshed by the POST_BUILD mirror) and confirm
   boot output unchanged; for self-test images, confirm the per-case
   `[Self-test] PASS` lines and the final PASS summary.

## Tiers

### Tier 1 — self-test harness extraction: DONE (2026-11-21)

- Added `include/self_test.hpp` + `src/self_test.cpp`
  (`app::run_self_tests(Uart&)`, whole body behind `#ifdef STM32_SELF_TEST`).
- Moved: six suite invocations, per-case UART reporter, FATAL handling,
  real-kernel queue probe (including the fill-then-reject fix).
- `main.cpp`: 831 → 734 lines; calls `app::run_self_tests(uart1)` under the
  same ifdef. Non-self-test binaries are byte-identical to before (all moved
  code compiles out), verified via the `dist/debug` mirror keeping its
  timestamp.
- Fixed along the way: `compil ed` typo, two `-Wcomment` warnings
  (`tests/*.cpp` inside doc comments), stale glob reconfigure for
  `build/selftest`.

### Tier 2 — fault-report and clock/POST modules: DONE (2026-11-21)

- Added `include/fault_report.hpp` + `src/fault_report.cpp`: all `extern "C"`
  fail-stop hooks, `HardFault_Handler`/`hard_fault_report` (names and LTO
  attributes preserved verbatim, contract 2), and the kernel-object dumps.
  The hooks reach the UART and diagnostic handles through a registration API
  (`bsp::register_fault_diagnostics(uart, i2c_mutex_handle, queue_handle)`)
  called from `main()` once the objects exist; handles are captured by value
  (stable after kernel-object creation), so `AppContext` is not needed by
  the fault path. Hooks firing before registration stay silent instead of
  touching uninitialized hardware. Contract 3 held: hooks still never lock.
- Added `include/clock.hpp` + `src/clock.cpp`: `bsp::system_clock_config`,
  `bsp::ports_init(const Led&, Uart&)` (`Led` is used through const refs —
  its methods are `const` by design for constexpr instances),
  `bsp::run_i2c_scan_check(I2cBus&, Uart&)`.
- Tier 3 partial, landed early with this tier: `include/board_state.hpp` +
  `src/board_state.cpp` — the `gboard_state` atomic became the
  `bsp::set/clear/is_hardware_fault_active()` API (the `BoardState` enum is now
  private to the implementation). All main.cpp call sites (heartbeat,
  sensor_task, tap_task, clock-fail path) were switched to the API, so the
  raw atomic no longer exists anywhere else.
- Removed from `main.cpp`: the hooks, dumps, clock/POST functions,
  `g_app_context`, `gboard_state`, and the `<atomic>`/`<cstring>` includes.
  `main.cpp`: 734 → 530 lines.

### Tier 3 — application tasks and shared state: DONE (2026-11-21)

- Added `include/app_context.hpp`: `AppContext` (now carries `Uart&` and
  `const Led&` references plus the mirrored `TaskHandle_t task_handles[4]`
  array — written once by main() between task creation and scheduler start,
  read by heartbeat) and `TelemetryMessage`. `static AppContext` storage
  rule preserved (contract 1); the peripheral references bind to file
  statics declared earlier in the same translation unit.
- Added `include/tasks.hpp` and moved the task bodies:
  `src/environment_tasks.cpp` (`app::sensor_task`, `app::telemetry_task`,
  `print_channel`), `src/tap_task.cpp` (`app::tap_task`),
  `src/heartbeat.cpp` (`app::heartbeat`). Task entry signatures stay
  `void (*)(void*)` receiving `&app` (contract 4); tasks reach the UART and
  LED through `ctx->uart` / `ctx->status_led` instead of file statics.
- `init_status_text` became an inline `drivers::init_status_text()` in
  `include/sensor_interface.hpp` (host-testable); both retry loops call it.
- `main.cpp` ends at 141 lines: tick hook, peripheral statics, pre-HAL
  failure path, bring-up calls, task creation, validity checks, scheduler
  start. The `g_task_handles` and `g_app_context` globals are gone (handles
  live in `AppContext`, diagnostics were registered in Tier 2).
- **Latent bug fixed along the way:** `config/FreeRTOSConfig.h` declared
  `extern uint32_t SystemCoreClock;` with no `extern "C"` guard. It worked
  only because every existing file happened to include the HAL (which
  declares it with C linkage) before FreeRTOS.h — the unguarded
  redeclaration inherits C linkage. The new task files included
  `FreeRTOS.h` first and exposed the conflict. The declaration is now
  wrapped in `#ifdef __cplusplus extern "C"`, so include order no longer
  matters.
- Removed an unused `record_failure` helper from `tests/test_lsm6dsl_tap.cpp`
  that the target build flagged (`-Wall -Wextra`) after the per-case
  reporting change.

## Progress log

| Date | Tier | Result |
|---|---|---|
| 2026-11-21 | 1 | `main.cpp` 831 → 734 lines; self-test harness extracted to `src/self_test.cpp` + `include/self_test.hpp`; all three variants build clean, host tests pass, `dist/selftest` refreshed |
| 2026-11-21 | 2 | `main.cpp` 734 → 530 lines; fault-report module (`fault_report`) with registration API, clock/POST module (`clock`), and `board_state` API (Tier 3 partial) extracted; contracts 2 and 3 preserved (hook names/LTO attributes verbatim, fault path lock-free); all three variants build clean, host tests pass |
| 2026-11-21 | 3 | `main.cpp` 530 → 141 lines; task bodies extracted (`heartbeat`, `environment_tasks`, `tap_task`) with `app_context.hpp` + `tasks.hpp`; `init_status_text` moved to `sensor_interface.hpp`; fixed latent `FreeRTOSConfig.h` missing-`extern "C"` bug exposed by the new include order; all three variants build clean, host tests pass |
| 2026-11-21 | follow-up | Folder-per-layer restructure: sources moved to `src/{app,bsp,drivers}/`, headers to `include/{app,bsp,drivers}/`; shared utilities (`split_float.hpp`, `stall_tracker.hpp`) and the composition root (`main.cpp`) stay at the roots; all cross-layer includes are now layer-qualified (`#include "bsp/i2c_bus.hpp"`, `#include "drivers/hts221_sensor.hpp"`, `#include "app/app_context.hpp"`); `tests/CMakeLists.txt` source paths updated; the `GLOB_RECURSE` firmware build picks the subfolders up unchanged; all three variants build clean, host tests pass |
| 2026-11-21 | follow-up | `include/rtos.hpp` moved to `include/rtos/rtos.hpp` to match the `rtos::` namespace like every other layer; all 7 includers updated to `#include "rtos/rtos.hpp"`; builds clean, host tests pass |
