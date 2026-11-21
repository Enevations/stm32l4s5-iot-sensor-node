# STM32L4S5 Base Firmware

[![CI](https://github.com/Enevations/stm32l4s5-iot-sensor-node/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/Enevations/stm32l4s5-iot-sensor-node/actions/workflows/ci.yml)

This repository is a C++17 and FreeRTOS firmware for the STMicroelectronics
**B-L4S5I-IOT01A** discovery board. It reads the onboard HTS221
humidity/temperature and LPS22HB barometric sensors over I2C2, polls the
onboard LSM6DSL accelerometer's hardware double-tap detector, and prints
sensor telemetry over the ST-LINK virtual COM port, while a heartbeat task
drives LED LD2 and reports per-task stack margins.

## What it does

At startup, `src/main.cpp`:

1. Calls the STM32 HAL initialization.
2. Configures the MSI/MSI-PLL clock tree for an 80 MHz system clock.
3. Enables GPIOB and configures PB14 as the LD2 green LED.
4. Configures PB6 (USART1 TX) and PB7 (USART1 RX) for 115200 baud.
5. Initializes I2C2 (PB10/PB11) and verifies that the expected onboard
   sensors respond (a power-on self-test).
6. Creates four FreeRTOS tasks and starts the scheduler:
   - **Heartbeat** (priority 2): blinks LD2 every second (fast blinks on
     hardware fault) and prints per-task stack high-water marks every ~5 s.
   - **SensorTask** (priority 1): initializes each registered sensor
     independently (identity check, calibration/configuration), retrying with
     exponential backoff and a logged failure reason until it succeeds, then
     samples every ready sensor once every five seconds into a merged reading
     (HTS221: humidity + temperature; LPS22HB: pressure + temperature).
   - **TelemetryTask** (priority 1): prints one line per valid channel of the
     merged reading — `[<uptime> ms] Temp: <t>.<tt> C`,
     `[<uptime> ms] Humidity: <h>.<hh> %`,
     `[<uptime> ms] Pressure: <p>.<pp> hPa`.
   - **TapTask** (priority 1): polls the LSM6DSL tap-source register
     every 20 ms and prints `[<uptime> ms] Double tap detected` whenever the
     chip's hardware double-tap recognizer reports an event.

The FreeRTOS SysTick runs at 1 kHz. The tick hook calls `HAL_IncTick()`. The
firmware also has fail-stop hooks for allocation failure, stack overflow, and
hard faults; see [Fault reporting](#fault-reporting-and-diagnostics).

## Hardware

- Board: STMicroelectronics B-L4S5I-IOT01A
- MCU: STM32L4S5VIT6
- Core: ARM Cortex-M4F
- LD2: PB14 (heartbeat; fast blink pattern signals a hardware fault)
- Onboard sensors: I2C2 bus on PB10 (SCL) / PB11 (SDA), with 2.2 kΩ external
  pull-ups (per board user manual UM2708). Expected addresses:
  - LIS3MDL magnetometer: 0x1E
  - HTS221 humidity/temperature: 0x5F (environmental telemetry source)
  - LSM6DSL accelerometer/gyro: 0x6A (double-tap detection source)
  - LPS22HB barometric pressure: 0x5D
- USART1 TX: PB6
- USART1 RX: PB7
- ST-LINK virtual COM port: 115200 baud, 8-N-1

The I2C2 self-test treats a bus initialization failure or a missing expected
device as a hardware fault; the heartbeat task then blinks LD2 at 250 ms
instead of 1 s. Sensor read failures at runtime are logged and retried on the
next sample cycle.

The 115200 baud serial connection is read through the board's ST-LINK USB
connection. No wiring is required.

## Project layout

```text
.
├── CMakeLists.txt                     # Firmware target and fetched dependencies
├── cmake/arm-none-eabi.cmake          # ARM Cortex-M4F cross-compilation toolchain
├── config/
│   ├── FreeRTOSConfig.h               # FreeRTOS features and interrupt priorities
│   └── stm32l4xx_hal_conf.h           # Enabled HAL modules
├── include/
│   ├── hts221_sensor.hpp              # HTS221 humidity/temperature sensor (implements ISensor)
│   ├── i2c_bus.hpp                    # STM32 I2C2 bus (implements II2cBus)
│   ├── i2c_interface.hpp              # HAL-free II2cBus abstraction
│   ├── led.hpp                        # GPIO LED wrapper
│   ├── lps22hb_sensor.hpp             # LPS22HB barometric sensor (implements ISensor)
│   ├── lsm6dsl_tap.hpp                # LSM6DSL double-tap driver (gesture event source)
│   ├── post_check.hpp                 # Pure I2C power-on self-test probe
│   ├── rtos.hpp                       # FreeRTOS C++ RAII wrappers
│   ├── sensor_interface.hpp           # HAL-free ISensor abstraction + shared sample/InitStatus
│   ├── split_float.hpp                # Constexpr telemetry float formatting (host-testable)
│   ├── stall_tracker.hpp              # Telemetry queue drop/recovery policy (host-testable)
│   └── uart.hpp                       # Thread-safe UART wrapper
├── src/
│   ├── hts221_sensor.cpp              # HTS221 calibration and sample conversion
│   ├── i2c_bus.cpp                    # I2C2 HAL setup and serialized transactions
│   ├── lps22hb_sensor.cpp             # LPS22HB pressure/temperature conversion
│   ├── lsm6dsl_tap.cpp                # LSM6DSL double-tap configuration and event read
│   └── main.cpp                       # Clock, GPIO, tasks, and telemetry application
├── tests/
│   ├── CMakeLists.txt                 # Host + cross-compiled test targets
│   ├── fake_i2c_bus.hpp               # Emulated I2C bus with canned registers and NACKs
│   ├── host_main.cpp                  # Entry point for host-built test binary
│   ├── test_hts221.cpp                # HTS221 driver suite
│   ├── test_lps22hb.cpp               # LPS22HB driver suite
│   ├── test_lsm6dsl_tap.cpp           # LSM6DSL tap driver suite
│   ├── test_post_check.cpp            # POST probe suite
│   ├── test_report.hpp                # Per-case PASS/FAIL reporter shared by suites
│   └── test_split_float.cpp           # Telemetry formatting suite
├── STM32L4S5VITx_FLASH.ld             # 2 MB Flash / 640 KB SRAM memory layout
├── docs/
│   ├── findings.md                     # Review findings log: resolved, open, lessons
│   └── stm32_b_l4s5i_iot01a_setup_guide.md
├── .devcontainer/                     # Reproducible Debian ARM build container
├── .vscode/                           # Build tasks and Cortex-Debug launch config
└── build/debug/                       # Existing debug build outputs
```

## Application architecture

`AppContext` (defined in `src/main.cpp`) owns the shared state: the I2C bus,
the sensor objects, and the telemetry queue. All task-to-task communication
flows through the queue, and the I2C/UART wrappers serialize access with their
own mutexes (`rtos::SchedulerLock`), so no task ever holds two locks at once.

### Sensors

Sensors are one implementation of `drivers::ISensor`
(include/sensor_interface.hpp) per *physical device* — never per measurement
channel, since a device's channels share its identity check, calibration
image, configuration register, and burst read:

- `Hts221Sensor` (0x5F): humidity + temperature
- `Lps22hbSensor` (0x5D): barometric pressure + die temperature

`sensor_task` iterates the registered `ISensor` list: each sensor initializes
independently with its own retry backoff (2 s → 4 s → 8 s cap), so one failed
or absent sensor never blocks the others; a recovered init clears the fault
flag only if this task raised it. Each cycle reads every ready sensor into a
merged `drivers::EnvironmentalSample` — per-channel `valid_*` flags record
which sensor produced which field — and queues it for telemetry.

Adding a sensor means implementing `drivers::ISensor` and adding it to the
list in `sensor_task`; neither the task nor the telemetry code changes. The
LPS22HB driver on this board is the working example (WHO_AM_I 0x0F → 0xB1,
CTRL_REG1 0x10 with 1 Hz ODR + BDU, pressure = 24-bit raw/4096 hPa,
temperature = 16-bit raw/100 °C).

The HTS221 driver reads the factory calibration coefficients (0x30-0x3F) once
during `init()`, precomputes the interpolation slopes, and converts raw ADC
output in `read()`. Register block reads set the auto-increment bit, and
block data update (BDU) is enabled so humidity and temperature are always
read as a consistent pair.

### Tap detection

The LSM6DSL accelerometer recognizes double taps in hardware. `Lsm6dslTap`
(include/lsm6dsl_tap.hpp) configures the detector per ST AN5040 §5.5.5:
accelerometer at 416 Hz/±2 g, tap recognition on all three axes with a
~600 mg threshold (below the app-note example value, so desk taps register),
~538 ms maximum gap between taps. The configuration is *unlatched* (LIR = 0,
the app-note example value): latched mode combined with the task's continuous
`TAP_SRC` polling prevented the second tap of a gesture from ever pairing on
this board, while unlatched works — the `DOUBLE_TAP` flag then self-clears
after the Quiet window (~29 ms), which the 20 ms poll samples inside. The
driver is deliberately *not* an `ISensor` — tap events are a continuous
gesture stream, the case the sensor interface reserves for a separate event
source.

`tap_task` polls the tap-source register every 20 ms and prints
`[<uptime> ms] Double tap detected` over the ST-LINK UART for each gesture
(repeats inside 100 ms are suppressed — the flag outlives one poll). A
recognized single tap prints a rate-limited `Single tap detected (no second
tap within 0.5 s)` tuning line — it proves the chip sees taps when doubles
are not being paired, and consecutive lines show the tap rhythm. Init and
bus-loss recovery use the same exponential retry backoff as the
environmental sensors.

**Important pitfall:** objects referenced by FreeRTOS tasks must not live in
`main()`'s stack frame. The Cortex-M4F port resets the MSP from the vector
table when the first task starts, so SysTick and other exception frames
overwrite main's abandoned frame. `AppContext` is therefore declared
`static` inside `main()`; keep it that way.

## Resource ownership and RAII

**RAII** (Resource Acquisition Is Initialization) is a C++ pattern that ties
the lifetime of a resource to an object's lifetime. The object acquires the
resource in its constructor and releases it in its destructor.

The RAII wrappers in `include/rtos.hpp` follow this pattern:

- `Task` creates a FreeRTOS task in its constructor and deletes it in its
  destructor.
- `Queue` creates a FreeRTOS queue in its constructor and deletes it in its
  destructor.
- `Mutex` creates a FreeRTOS mutex in its constructor and deletes it in its
  destructor.
- `LockGuard` takes a mutex when constructed and releases it when destroyed,
  even if the function exits early or throws an exception.
- `SchedulerLock` behaves like `LockGuard` once the scheduler is running, but
  before the scheduler starts it proceeds without the mutex (only `main()`
  runs then, so there is nothing to serialize against).

Each wrapper exposes `valid()`, which reports whether the kernel object was
actually created. `Mutex::lock()`, `Queue::send()`, and `Queue::receive()`
fail gracefully (return `false`) on a never-created object instead of
dereferencing a null handle. `main()` checks `valid()` for every RTOS object
before starting the scheduler and halts with a `[FATAL]` message if any
creation failed. Heap exhaustion additionally trips the FreeRTOS
malloc-failed hook from inside the kernel allocation, printing its own
`[FATAL]` line before halting.

This reduces the risk of leaked tasks, queues, or mutex ownership and keeps
resource cleanup close to the code that acquires the resource.

## Git workflow notes

Practices that prevent the stale-stash/conflict incident this project hit:

- **Commit early.** Park work in commits on a branch, not in stashes — a stash
  is invisible in `git log`, goes stale silently, and conflicts painfully when
  re-applied days later. A `WIP:` commit beats a stash every time.
- **Silent auto-stash is disabled** (`git.autoStash: false` in
  `.vscode/settings.json`): a pull/sync with a dirty tree stops with an error
  instead of stashing and re-applying behind your back.
- **A pre-commit hook rejects unresolved conflict markers**
  (`githooks/pre-commit`, wired via `core.hooksPath githooks` in the dev
  container's `postCreateCommand`; for other clones run
  `git config core.hooksPath githooks` once). CI performs the same check, so
  marker-laden commits cannot reach `master` either way.
- **End AI or work sessions with `git status` + a build/test run** — never
  leave a tree whose state is unknown to the next session.

## Fault reporting and diagnostics

Failures stop the system loudly instead of freezing silently:

- **Allocation failure** prints `[FATAL] FreeRTOS heap exhausted` and halts.
- **Stack overflow** (FreeRTOS level-2 checking) prints
  `[FATAL] Stack overflow in task: <name>` and halts.
- **Hard fault** prints the faulting task name, the stacked PC/LR, xPSR, PSP,
  and the CFSR/HFSR/BFAR/MMFAR fault registers, then halts.

Decode a reported PC or LR against the ELF to find the faulting code:

```sh
arm-none-eabi-addr2line -e build/debug/stm32l4s5-iot-sensor-node.elf -f -C 0x<addr>
```

The hard fault handler runs on the untouched MSP stack, so the report survives
even faults caused by a task destroying its own stack.

The heartbeat task also prints stack high-water marks every ~5 s:

```text
Stack free words: HB=372 SN=836 TM=858 TP=370
```

Values are minimum-ever free words per task (Heartbeat, SensorTask,
TelemetryTask, TapTask); a value near zero identifies a task at risk of
overflow.
At fault time the firmware additionally hex-dumps the UART/I2C mutex and
telemetry queue structures so memory corruption can be compared across runs.

## Testing

### Architecture: interfaces + fakes

Hardware-dependent code is separated from testable logic through a pure C++
interface: `bsp::II2cBus` (include/i2c_interface.hpp) has no HAL dependency.
`bsp::I2cBus` implements it with the STM32 HAL; tests inject
`test::FakeI2cBus` (tests/fake_i2c_bus.hpp), which emulates a register file
per device address and simulates NACKs by marking devices absent. The HTS221
driver and the POST probe (`bsp::run_post_check`) run identically against
both — on the target and in host unit tests.

### Host unit tests

Runs in the dev container and on any machine with a host C++ toolchain. The
dev container Dockerfile includes `g++`; if your running container predates
that change, install it once:

```sh
apt-get update && apt-get install -y g++
```

Then build and run (from the repository root):

```sh
cmake -B build/host-tests -S tests -G Ninja
cmake --build build/host-tests
./build/host-tests/unit_tests
```

Expected output: one `PASS: <suite>: <case>` (or `FAIL: ...`) line per test
case as it runs, followed by the `PASS: <suite> tests` summary lines for
split_float, hts221, lps22hb, post_check, stall_tracker, and lsm6dsl_tap. You
can also run `ctest --test-dir build/host-tests --output-on-failure`.

### On-target self-test

The same suites are compiled into the firmware and executed during boot,
reporting over the ST-LINK UART:

```sh
cmake -B build/selftest -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
  -DFETCHCONTENT_BASE_DIR="$PWD/.cache/deps" \
  -DSTM32_SELF_TEST=ON

cmake --build build/selftest
```

Expected serial output: one `[Self-test] PASS <suite>: <case>` line per test
case as it runs (so a failing target names the exact case), ending with
`Self-test PASS (split_float, hts221, lps22hb, post_check, stall_tracker,
lsm6dsl_tap, queue probe)` after the banner. A failing case prints
`[Self-test] FAIL <suite>: <case>` and the firmware then prints
`[FATAL] Self-test <suite> FAILED: <case>` and halts.

#### Running the self-test on the target

The self-test firmware runs automatically at boot; you only need to flash it
and watch the serial port:

1. Build it (above), or via VS Code: **Tasks: Run Build Task** →
   **Build STM32 Firmware (Self-Test)**.
2. Flash it exactly as described in [Flashing](#flashing) — the self-test
   build mirrors its image to `dist/selftest/`, so in your Windows PowerShell
   terminal:

   ```powershell
   & "D:/xpack-openocd-0.12.0-7/bin/openocd.exe" `
     -f interface/stlink.cfg `
     -f target/stm32l4x.cfg `
     -c "program dist/selftest/stm32l4s5-iot-sensor-node.elf verify reset exit"
   ```

3. Open the serial terminal as described in
   [Viewing serial output](#viewing-serial-output) (115200 8-N-1) and watch
   for the `Self-test PASS (...)` line after the banner.
4. On failure the failing case prints `[Self-test] FAIL <suite>: <case>` and
   the firmware then prints `[FATAL] Self-test <suite> FAILED: <case>` and
   halts (LED stops blinking). Fix the case, rebuild, and flash again.
5. Afterwards, flash the normal debug or release image to return to the
   regular firmware.

### Test coverage

| Suite | What it verifies |
|-------|------------------|
| `split_float` | Sign preservation in (-1, 0), rounding vs truncation, fraction carry, invariants across a value sweep |
| `hts221` | WHO_AM_I check, calibration-image load, degenerate-calibration guard, linear interpolation at midpoints and endpoints, negative temperatures, humidity clamping, invalid sample on bus loss |
| `lps22hb` | WHO_AM_I check, pressure conversion (hPa), 24-bit sign extension, negative values, per-channel valid flags, invalid sample on bus loss |
| `lsm6dsl_tap` | WHO_AM_I check, AN5040 double-tap configuration registers written at init, double-tap event decode (and non-events), bus error reporting |
| `post_check` | All-devices-present pass, missing-device detection with reported address, probe stop at first failure, empty list |
| `stall_tracker` | Consecutive-drop counting, reset on recovery, fresh stall after recovery, 1000-sample stall tracking |
| queue probe (target only) | A full `rtos::Queue` rejects a zero-tick send and draining makes room again — the bounded-send mechanism the drop policy relies on |

### Testing the failure paths

Which paths can be exercised, and how:

| Path | How to trigger |
|------|----------------|
| Sensor failure modes | Covered by the `hts221` suite via `FakeI2cBus` (wrong WHO_AM_I, NACK, degenerate calibration, bus loss mid-run) |
| Missing device at POST | Covered by the `post_check` suite |
| Clock config failure | Debugger: break at the return of `HAL_RCC_ClockConfig` and force the return register to `HAL_ERROR` |
| HAL_Init failure | Debugger: same technique at `HAL_Init`'s return |
| Stack overflow | Temporarily shrink a task's stack (e.g. `SensorTask` to 64 words); the overflow hook names the task |
| Mutex contention / queue backpressure | Lower the telemetry queue capacity and raise sensor rate |

The fault-handling logic itself (sign handling, fault-state transitions) is
kept in pure, host-testable functions wherever practical so that most
regressions surface in the unit suites instead of on hardware.

### Continuous integration

`.github/workflows/ci.yml` runs on every push and pull request:

- **Host unit tests** job: builds and runs the `tests/` suite with ctest.
- **Firmware** job: installs the ARM toolchain once, then builds the debug,
  release, and self-test variants sequentially — sharing the single toolchain
  install and FetchContent checkout — verifying each `dist/<variant>/`
  flashing artifact exists, and uploading them as one workflow artifact
  (7-day retention).

The release variant matters in particular: it builds with `-flto`, which has
already caught one LTO-only link failure that the debug build did not expose.

## Build dependencies

The project fetches these pinned dependencies during CMake configuration:

- STM32CubeL4 **v1.18.1**: CMSIS, startup code, system initialization, and HAL
- FreeRTOS Kernel **V11.3.3**: ARM CM4F port and Heap 4

The included build commands use the FetchContent cache at `.cache/deps`, which
keeps downloaded dependencies outside `build/`. The cache is not source code
maintained in this repository.

Local host tools:

- CMake 3.22 or newer
- Ninja
- GNU Arm Embedded toolchain:
  `arm-none-eabi-gcc`, `arm-none-eabi-g++`, `arm-none-eabi-objcopy`,
  and `arm-none-eabi-size`
- Git, for FetchContent downloads

The recommended setup is VS Code with the included dev container. This requires
Docker and the VS Code Dev Containers extension.

## Build

### Using the dev container

1. Open the repository in VS Code.
2. Run **Dev Containers: Reopen in Container**.
3. Run **Tasks: Run Build Task** and choose
   **Build STM32 Firmware (Debug)**.

The container uses Debian with `gcc-arm-none-eabi`, CMake, and Ninja.

### From a terminal

From the repository root:

```sh
cmake -B build/debug \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
  -DFETCHCONTENT_BASE_DIR="$PWD/.cache/deps"

cmake --build build/debug
```

For an optimized build:

```sh
cmake -B build/release \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
  -DFETCHCONTENT_BASE_DIR="$PWD/.cache/deps"

cmake --build build/release
```

The VS Code tasks provide the same build commands (debug, release, self-test,
host unit tests). Debug and release build directories are independent.

## Build outputs

A successful build creates:

- `build/debug/stm32l4s5-iot-sensor-node.elf` or `build/release/stm32l4s5-iot-sensor-node.elf`
- `build/debug/stm32l4s5-iot-sensor-node.bin` or `build/release/stm32l4s5-iot-sensor-node.bin`
- `build/debug/stm32l4s5-iot-sensor-node.hex` or `build/release/stm32l4s5-iot-sensor-node.hex`
- `build/<configuration>/stm32l4s5-iot-sensor-node.map`
- `build/<configuration>/compile_commands.json`

The post-build step also reports the ELF Flash and RAM usage. Because `build/`
is a Docker volume (see the container mounts), every build mirrors its
`.elf`/`.bin`/`.hex` images to `dist/<build-type>/` in the repository root for
Windows-host flashing — see [Flashing](#flashing).

## Flashing

The board must be connected to the Windows host, not only to the dev container.

**Important:** the container mounts `build/` as a Docker volume, so files under
it are visible inside the container only — the Windows host cannot read them.
Every build therefore mirrors its images to `dist/<variant>/` in the workspace
root (which *is* shared with Windows) via a CMake post-build step:

- `dist/debug/stm32l4s5-iot-sensor-node.elf`
- `dist/release/stm32l4s5-iot-sensor-node.elf`
- `dist/selftest/stm32l4s5-iot-sensor-node.elf`

Each folder also contains the `.bin` and `.hex` images. Then open a
**PowerShell terminal on the Windows host** in the repository root and run
xPack OpenOCD:

```powershell
& "D:/xpack-openocd-0.12.0-7/bin/openocd.exe" `
  -f interface/stlink.cfg `
  -f target/stm32l4x.cfg `
  -c "program dist/debug/stm32l4s5-iot-sensor-node.elf verify reset exit"
```

Flash `dist/release/...` or `dist/selftest/...` instead for the release or
self-test image. Stop/reset any existing OpenOCD session before running the
command.

If programming fails with `Error: Invalid command argument` or
`couldn't open ...`, check the image path and that `verify`, `reset`, and
`exit` are three separate words — hyphenated or merged forms are rejected.

## Debugging

The included launch configuration starts a Cortex-Debug session using an
**external** OpenOCD GDB server at `host.docker.internal:3333`. It expects that
server to already be running on the Docker host and for the
`gdb-multiarch` debugger to be available in the dev container.

Connect the board to the Windows host, then run this command in a PowerShell
terminal on the host:

```powershell
& "D:/xpack-openocd-0.12.0-7/bin/openocd.exe" `
  -f interface/stlink.cfg `
  -f target/stm32l4x.cfg `
  -c "bindto 0.0.0.0"
```

Keep this OpenOCD process running. Then open the repository in the dev container
and press F5, or select
**Debug STM32L4S5 (Dev Container)**. The launch configuration builds the debug
target and starts execution at `main`.

The Windows/host-specific setup and Docker details are documented in
`docs/stm32_b_l4s5i_iot01a_setup_guide.md`.

## Viewing serial output

Open the board's ST-LINK virtual COM port with a serial terminal:

- Baud: 115200
- Data: 8 bits
- Parity: none
- Stop: 1 bit
- Flow control: none

You should see a startup banner, the I2C self-test result, and then sensor
telemetry once every five seconds (one line per valid channel, plus a
pressure line from the LPS22HB) with stack reports every ~5 s:

```text
[Post Success] All expected devices found

STM32L4S5I-IOT01A Sensor Node
[5000 ms] Temp: 31.71 C
[5000 ms] Humidity: 46.21 %
[5000 ms] Pressure: 1013.25 hPa
[10000 ms] Temp: 31.65 C
[10000 ms] Humidity: 46.11 %
[12040 ms] Double tap detected
Stack free words: HB=372 SN=836 TM=850 TP=370
```

Sensor read failures print `Sensor 0x%02X read failed; retrying next cycle`
and the firmware continues sampling the other sensors. Double-tapping the
board prints `[<uptime> ms] Double tap detected`. Press the board reset
button to restart the application.

## Cleaning

Clean one build configuration:

```sh
cmake --build build/debug --target clean
cmake --build build/release --target clean
```

Delete build directories while retaining downloaded dependencies:

```sh
rm -rf build/debug build/release
```

Delete the FetchContent cache or all generated state:

```sh
rm -rf .cache/deps
rm -rf build .cache/deps
```
