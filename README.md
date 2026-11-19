# STM32L4S5 Base Firmware

This repository is a minimal C++17 and FreeRTOS base firmware for the
STMicroelectronics **B-L4S5I-IOT01A** discovery board. It configures the
STM32L4S5, starts a FreeRTOS task that blinks LD2, and prints a heartbeat over
the ST-LINK virtual COM port.

## What it does

At startup, `src/main.cpp`:

1. Calls the STM32 HAL initialization.
2. Configures the MSI/MSI-PLL clock tree for an 80 MHz system clock.
3. Enables GPIOB and configures PB14 as the LD2 green LED.
4. Configures PB6 (USART1 TX) and PB7 (USART1 RX) for 115200 baud.
5. Creates the `Blinky` FreeRTOS task.
6. Prints a banner and then a blink count every second.

The FreeRTOS SysTick runs at 1 kHz. The tick hook calls `HAL_IncTick()`, and
the project also has hooks for allocation failure and stack overflow detection.

## Hardware

- Board: STMicroelectronics B-L4S5I-IOT01A
- MCU: STM32L4S5VIT6
- Core: ARM Cortex-M4F
- LD2: PB14
- External red failure LED: PA5 (through a current-limiting resistor to ground)
- USART1 TX: PB6
- USART1 RX: PB7
- ST-LINK virtual COM port: 115200 baud, 8-N-1

If I2C2 initialization fails, the firmware stops the green heartbeat and
blinks the external red LED on PA5. A scan that initializes successfully but
finds no responding devices is not treated as a failure.

The 115200 baud serial connection is read through the board's ST-LINK USB
connection. No wiring is required for the default blink and serial output.

## Project layout

```text
.
├── CMakeLists.txt                     # Firmware target and fetched dependencies
├── cmake/arm-none-eabi.cmake          # ARM Cortex-M4F cross-compilation toolchain
├── config/
│   ├── FreeRTOSConfig.h               # FreeRTOS features and interrupt priorities
│   └── stm32l4xx_hal_conf.h           # Enabled HAL modules
├── include/
│   ├── i2c_bus.hpp                    # I2C bus interface scaffold
│   └── rtos.hpp                       # FreeRTOS C++ RAII wrappers
├── src/
│   ├── i2c_bus.cpp                    # I2C example scaffold
│   └── main.cpp                       # Clock, GPIO, UART, and FreeRTOS application
├── STM32L4S5VITx_FLASH.ld             # 2 MB Flash / 640 KB SRAM memory layout
├── docs/
│   └── stm32_b_l4s5i_iot01a_setup_guide.md
├── .devcontainer/                     # Reproducible Debian ARM build container
├── .vscode/                           # Build tasks and Cortex-Debug launch config
└── build/debug/                       # Existing debug build outputs
```

The I2C implementation and example in `include/i2c_bus.hpp` and
`src/i2c_bus.cpp` are development scaffolding. The current application in
`src/main.cpp` does not initialize I2C or run an I2C scan.

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

This reduces the risk of leaked tasks, queues, or mutex ownership and keeps
resource cleanup close to the code that acquires the resource.

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

You should see a startup banner followed by:

```text
Blink #1 (uptime: 1000 ms)
Blink #2 (uptime: 2000 ms)
```

Press the board reset button to restart the application.

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
