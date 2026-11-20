# Findings Log

Record of review findings for this firmware, what was done about each, and
what is still open. Update this file whenever a new finding is identified or
resolved so future sessions can pick up where the last one stopped.

Last updated: 2026-11-20 (branch `feat/improvements`)

## Resolved

| ID | Finding | Fix | Where |
|----|---------|-----|-------|
| #1 | Negative temperatures in (-1, 0) lost their sign in telemetry lines (truncation toward zero); fractions truncated instead of rounded | `SplitFloat`/`split_float` — sign kept separate from whole, hundredths rounded with carry; unit-tested | `include/split_float.hpp`, `tests/test_split_float.cpp`, `src/main.cpp` |
| #2 | `AppContext` lived in `main()`'s stack frame; the Cortex-M4F port resets MSP from the vector table when the first task starts, so SysTick frames overwrote the mutex/queue handles and SensorTask crashed (`HardFault`, INVPC/BUSFAULT) | `static AppContext` inside `main()`; pitfall documented in README and at the declaration | `src/main.cpp` |
| #3 | RTOS wrapper constructors silently ignored allocation failures; a failed object led to null-handle kernel calls | `valid()` accessors on `Task`/`Queue`/`Mutex`, null-handle guards (fail-soft returns), startup validation in `main()` printing `[FATAL] RTOS object creation failed` | `include/rtos.hpp`, `src/main.cpp` |
| #4 | Fail-stop hooks and `hard_fault_report()` ran with interrupts masked but still tried `mutex_.lock(portMAX_DELAY)` on the UART mutex; if a task held it, `xSemaphoreTake` blocked forever (masked PendSV cannot preempt) and all `[FATAL]` diagnostics were silently lost. Handler-mode `xSemaphoreTake` is also illegal | `SchedulerLock::may_lock()` skips locking when the scheduler is not running, when `xPortIsInsideInterrupt()` reports handler mode, or when BASEPRI is raised; fault paths write unlocked | `include/rtos.hpp` |
| #5 | `sensor_task` parked forever when HTS221 `init()` failed; no recovery from transient faults, no failure reason reported | `init()` returns `drivers::InitStatus` (Ok / BusError / WhoAmIMismatch / InvalidCalibration); `sensor_task` retries with exponential backoff (2 s → 4 s → 8 s cap), logs attempt count and reason, and clears `HardwareFault` on recovery (only if it raised the fault itself, so POST/clock faults are not masked) | `include/humidity_temp_snsr.hpp`, `src/humidity_temp_snsr.cpp`, `src/main.cpp`, `tests/test_hts221.cpp` |
| — | CI gap: LTO-only link failure (`hard_fault_report` invisible to `-flto` because it is called from inline asm) reached a flashing session | `.github/workflows/ci.yml` builds debug, release, and self-test variants on every push/PR and uploads flashable artifacts; asm-called symbol carries `used` + `externally_visible` | `.github/workflows/ci.yml`, `src/main.cpp` |
| — | Test infrastructure missing; on-target behavior hard to verify | Host unit tests (split_float, hts221 driver vs `FakeI2cBus`, post_check) + on-target self-test (`-DSTM32_SELF_TEST=ON`, reports over UART) + failure-trigger table in README | `tests/`, `README.md` |
| — | Flashing gap: `build/` is a Docker volume, invisible to the Windows host OpenOCD | CMake POST_BUILD mirrors `.elf`/`.bin`/`.hex` to `dist/<variant>/` in the shared workspace root | `CMakeLists.txt` |
| — | Timer service task allocated (~1.2 KB heap) although no timers are used | `configUSE_TIMERS 0`; safe because `INCLUDE_xTaskGetSchedulerState` is explicit | `config/FreeRTOSConfig.h` |
| — | Merge attempt left conflict markers committed in README.md and a stranded unmerged index on `feat/improvements` | Markers removed (kept current `stm32l4s5-iot-sensor-node` naming); stranded state reset | `README.md` |

## Open

| ID | Finding | Proposed direction |
|----|---------|--------------------|
| #6 | `Queue::send` defaults to `portMAX_DELAY` — a stalled telemetry task freezes the sensor task silently | Bounded send timeout in `sensor_task` + drop-oldest or log-on-full policy |
| #7 | `I2cBus::read_regs` silently truncates `length` to `uint16_t` | Reject `length > UINT16_MAX` |
| — | All faults share one fast-blink LED pattern | Blink-count encoding (N blinks + pause) mapped to fault source |
| — | HTS221 sampled every 1000 ms with 1 Hz ODR + BDU: duplicate/stale samples possible | DRDY-driven sampling (PD15/EXTI15 on this board) or read faster than ODR |
| — | HTS221 slopes recomputed every `read()`; `h1_rh_ > h0_rh_` not validated | Precompute slopes in `init()`; validate ordering |
| — | Startup banner printed from the heartbeat task | Move to `main()` after `ports_init()` |
| — | `gboard_state` declared `inline` in a single translation unit | Use `static` |
| — | MSP grows from 0x200A0000 into unreserved RAM (harmless today; nothing task-referenced lives on main's frame) | Add a dedicated stack region to the linker script if ISR nesting grows |
| — | Task stacks are generous (HWM: HB=332, SN=930, TM=880 free words) | Optional trim to reclaim RAM once stability is trusted |
| — | CI builds firmware variants sequentially in one job to share the toolchain install and FetchContent checkout. Measured: ~1 min faster wall-clock than the previous 3-job parallel matrix (one runner startup + one toolchain install + one dependency download instead of three) | Possible update: publish the devcontainer image to GHCR and run firmware builds in `container:` (toolchain baked in, zero per-run install) if the variant count grows enough to make sequential builds slow; alternative: cache an extracted toolchain tarball and restore it into parallel matrix jobs |

## Lessons learned (keep in mind for future changes)

- **MSP reset pitfall:** any object referenced by a FreeRTOS task must have
  static storage duration; `main()`'s stack frame is destroyed by the port's
  MSP reset. `AppContext` must stay `static` in `main()`.
- **LTO:** symbols called only from inline assembly (e.g. `hard_fault_report`)
  need `[[gnu::used, gnu::externally_visible]]` or Release builds fail to link.
- **Line endings:** README.md is CRLF in both index and working tree; scripts
  that patch it must preserve `\r\n`.
- **Verify instructions before documenting them:** the first flashing/test
  instructions shipped here failed in the dev container; the Dockerfile now
  installs `g++` and every command in the README has been executed at least
  once.
- **Always build all variants** (debug, release, self-test) plus host tests;
  the release/LTO variant has caught a failure no other configuration exposed.
