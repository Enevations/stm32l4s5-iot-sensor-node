# STM32L4S5 Base Firmware

This repository is a C++17 and FreeRTOS firmware for the STMicroelectronics
**B-L4S5I-IOT01A** discovery board. It reads the onboard HTS221
humidity/temperature sensor over I2C2 and prints sensor telemetry over the
ST-LINK virtual COM port, while a heartbeat task drives LED LD2 and reports
per-task stack margins.

## What it does

At startup, `src/main.cpp`:

1. Calls the STM32 HAL initialization.
2. Configures the MSI/MSI-PLL clock tree for an 80 MHz system clock.
3. Enables GPIOB and configures PB14 as the LD2 green LED.
4. Configures PB6 (USART1 TX) and PB7 (USART1 RX) for 115200 baud.
5. Initializes I2C2 (PB10/PB11) and verifies that the expected onboard
   sensors respond (a power-on self-test).
6. Creates three FreeRTOS tasks and starts the scheduler:
   - **Heartbeat** (priority 2): blinks LD2 every second (fast blinks on
     hardware fault) and prints per-task stack high-water marks every ~5 s.
   - **SensorTask** (priority 1): initializes the HTS221 (WHO_AM_I check,
     factory calibration, 1 Hz ODR with BDU) and samples it once per second.
   - **TelemetryTask** (priority 1): prints each sample as
     `[<uptime> ms] Temp: <t>.<tt> C | Humidity: <h>.<hh> %`.

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
  - HTS221 humidity/temperature: 0x5F (the sensor this firmware reads)
  - LSM6DSL accelerometer/gyro: 0x6A
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
│   ├── humidity_temp_snsr.hpp         # HTS221 humidity/temperature driver interface
│   ├── i2c_bus.hpp                    # Thread-safe I2C2 bus interface
│   ├── led.hpp                        # GPIO LED wrapper
│   ├── rtos.hpp                       # FreeRTOS C++ RAII wrappers
│   └── uart.hpp                       # Thread-safe UART wrapper
├── src/
│   ├── humidity_temp_snsr.cpp         # HTS221 calibration and sample conversion
│   ├── i2c_bus.cpp                    # I2C2 HAL setup and serialized transactions
│   └── main.cpp                       # Clock, GPIO, tasks, and telemetry application
├── STM32L4S5VITx_FLASH.ld             # 2 MB Flash / 640 KB SRAM memory layout
├── docs/
│   └── stm32_b_l4s5i_iot01a_setup_guide.md
├── .devcontainer/                     # Reproducible Debian ARM build container
├── .vscode/                           # Build tasks and Cortex-Debug launch config
└── build/debug/                       # Existing debug build outputs
```

## Application architecture

`AppContext` (defined in `src/main.cpp`) owns the shared state: the I2C bus
wrapper and the telemetry queue. All task-to-task communication flows through
the queue, and the I2C/UART wrappers serialize access with their own mutexes
(`rtos::SchedulerLock`), so no task ever holds two locks at once.

The HTS221 driver (`drivers::HumidityTempSnsr`) reads the factory calibration
coefficients (0x30-0x3F) once during `init()` and converts raw ADC output with
ST's linear-interpolation formula in `read()`. Register block reads set the
auto-increment bit, and block data update (BDU) is enabled so humidity and
temperature are always read as a consistent pair.

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

This reduces the risk of leaked tasks, queues, or mutex ownership and keeps
resource cleanup close to the code that acquires the resource.

## Fault reporting and diagnostics

Failures stop the system loudly instead of freezing silently:

- **Allocation failure** prints `[FATAL] FreeRTOS heap exhausted` and halts.
- **Stack overflow** (FreeRTOS level-2 checking) prints
  `[FATAL] Stack overflow in task: <name>` and halts.
- **Hard fault** prints the faulting task name, the stacked PC/LR, xPSR, PSP,
  and the CFSR/HFSR/BFAR/MMFAR fault registers, then halts.

Decode a reported PC or LR against the ELF to find the faulting code:

```sh
arm-none-eabi-addr2line -e build/debug/stm32l4s5-base.elf -f -C 0x<addr>
```

The hard fault handler runs on the untouched MSP stack, so the report survives
even faults caused by a task destroying its own stack.

The heartbeat task also prints stack high-water marks every ~5 s:

```text
Stack free words: HB=332 SN=930 TM=880
```

Values are minimum-ever free words per task (Heartbeat, SensorTask,
TelemetryTask); a value near zero identifies a task at risk of overflow.
At fault time the firmware additionally hex-dumps the UART/I2C mutex and
telemetry queue structures so memory corruption can be compared across runs.

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

The VS Code tasks provide the same build commands. Debug and release build
directories are independent.

## Build outputs

A successful build creates:

- `build/debug/stm32l4s5-base.elf` or `build/release/stm32l4s5-base.elf`
- `build/debug/stm32l4s5-base.bin` or `build/release/stm32l4s5-base.bin`
- `build/debug/stm32l4s5-base.hex` or `build/release/stm32l4s5-base.hex`
- `build/<configuration>/stm32l4s5-base.map`
- `build/<configuration>/compile_commands.json`

The post-build step also reports the ELF Flash and RAM usage.

## Flashing

The board must be connected to the Windows host, not only to the dev container.
Build the firmware in the container, then open a **PowerShell terminal on the
Windows host** in the repository root and run xPack OpenOCD:

```powershell
& "D:/xpack-openocd-0.12.0-7/bin/openocd.exe" `
  -f interface/stlink.cfg `
  -f target/stm32l4x.cfg `
  -c "program build/debug/stm32l4s5-base.elf verify reset exit"
```

Stop/reset any existing OpenOCD session before running the command. Use
`build/release/stm32l4s5-base.elf` instead for the release image.

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
telemetry once per second with stack reports every ~5 s:

```text
[Post Success] All expected devices found

STM32L4S5I-IOT01A Sensor Node
[72 ms] Temp: 31.71 C | Humidity: 46.21 %
[1072 ms] Temp: 31.65 C | Humidity: 46.11 %
Stack free words: HB=332 SN=930 TM=880
```

Sensor read failures print `Sensor read failed! Try again.` and the firmware
continues sampling. Press the board reset button to restart the application.

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
