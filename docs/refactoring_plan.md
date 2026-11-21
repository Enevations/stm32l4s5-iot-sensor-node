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

### Tier 2 — fault-report and clock/POST modules: PENDING

- `src/fault_report.cpp` (+ `include/fault_report.hpp`): all `extern "C"`
  fail-stop hooks, `HardFault_Handler`/`hard_fault_report`, kernel-object
  dumps (~115 lines out).
- `src/clock.cpp` (+ `include/clock.hpp`): `system_clock_config`,
  `ports_init`, `i2c_scan_check` (~65 lines out).
- Keep hook names/attributes per contract 2; fault path stays mutex-free per
  contract 3.

### Tier 3 — application tasks and shared state: PENDING

- `include/app_context.hpp`: `AppContext`, `TelemetryMessage`, `BoardState`
  plus a small fault-state API (set/clear/is_faulted over the atomic).
- `src/board.cpp` (+ `include/board.hpp`): `uart1`, `green_led`, the fault
  state, and the `g_app_context` diagnostics pointer become module-owned
  accessors (contract 6).
- `src/environment_tasks.cpp`: `sensor_task`, `telemetry_task`,
  `print_channel`; `init_status_text` becomes a `drivers::` helper in
  `include/sensor_interface.hpp` (host-testable).
- `src/tap_task.cpp`: `tap_task`.
- `src/heartbeat.cpp`: `heartbeat`.
- `include/tasks.hpp`: declares the four task entry points.
- `main.cpp` ends at ~130 lines; tasks keep the `void (*)(void*)` signature
  and `static AppContext` storage rule (contracts 1 and 4).
- Optional cleanup when touching `main.cpp` blocks: replace file-`static`
  helpers with anonymous-namespace style used by the driver modules
  (cosmetic; only where the block moves anyway).

## Progress log

| Date | Tier | Result |
|---|---|---|
| 2026-11-21 | 1 | `main.cpp` 831 → 734 lines; self-test harness extracted to `src/self_test.cpp` + `include/self_test.hpp`; all three variants build clean, host tests pass, `dist/selftest` refreshed |
| 2026-11-21 | 2 | not started |
| 2026-11-21 | 3 | not started |
