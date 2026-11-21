cmake_minimum_required(VERSION 3.22)

project(stm32l4s5-iot-sensor-node LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

# Fetch only the HAL/CMSIS submodules needed to build this firmware. CubeMX and
# CubeIDE are not part of the build; the firmware is configured in source.
FetchContent_Declare(
    stm32cubel4
    GIT_REPOSITORY https://github.com/STMicroelectronics/STM32CubeL4.git
    GIT_TAG        v1.18.1
    GIT_SHALLOW    TRUE
    GIT_SUBMODULES "Drivers/CMSIS/Device/ST/STM32L4xx" "Drivers/STM32L4xx_HAL_Driver" "Drivers/CMSIS/Core"
)
FetchContent_MakeAvailable(stm32cubel4)

# Select FreeRTOS's Cortex-M4F port and Heap 4 allocator; the project settings
# are supplied through the freertos_config interface target.
FetchContent_Declare(
    freertos_kernel
    GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
    GIT_TAG        V11.3.3
    GIT_SHALLOW    TRUE
)

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE config)
target_compile_definitions(freertos_config INTERFACE projCOVERAGE_TEST=0)

set(FREERTOS_PORT GCC_ARM_CM4F CACHE STRING "" FORCE)
set(FREERTOS_HEAP 4 CACHE STRING "" FORCE)

FetchContent_MakeAvailable(freertos_kernel)

# Compile the selected HAL drivers and include ST's CMSIS system clock code and
# STM32L4S5 startup/vector table.
set(CUBE_DRIVERS "${stm32cubel4_SOURCE_DIR}/Drivers")

set(HAL_SOURCES
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c"
)

# 4. Define the firmware executable
file(GLOB_RECURSE APP_SOURCES CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.c"
)

# Optional on-target self-test suite: -DSTM32_SELF_TEST=ON runs unit tests
# during boot and reports the result over the ST-LINK UART.
option(STM32_SELF_TEST "Compile the on-target self-test suite" OFF)

add_executable(${PROJECT_NAME}.elf
    ${APP_SOURCES}
    ${HAL_SOURCES}
    "${CUBE_DRIVERS}/CMSIS/Device/ST/STM32L4xx/Source/Templates/system_stm32l4xx.c"
    "${CUBE_DRIVERS}/CMSIS/Device/ST/STM32L4xx/Source/Templates/gcc/startup_stm32l4s5xx.s"
)

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
    USE_HAL_DRIVER
    STM32L4S5xx
    # Release disables debug-only FreeRTOS thread-awareness metadata.
    $<$<CONFIG:Release>:NDEBUG>
)

target_include_directories(${PROJECT_NAME}.elf PRIVATE
    include
    config
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Inc"
    "${CUBE_DRIVERS}/STM32L4xx_HAL_Driver/Inc/Legacy"
    "${CUBE_DRIVERS}/CMSIS/Device/ST/STM32L4xx/Include"
    "${CUBE_DRIVERS}/CMSIS/Core/Include"
)

if(STM32_SELF_TEST)
    target_sources(${PROJECT_NAME}.elf PRIVATE
        tests/test_split_float.cpp
        tests/test_hts221.cpp
        tests/test_lps22hb.cpp
        tests/test_post_check.cpp
        tests/test_stall_tracker.cpp
        tests/test_lsm6dsl_tap.cpp
    )
    # src/main.cpp installs the UART per-case reporter from tests/test_report.hpp.
    target_include_directories(${PROJECT_NAME}.elf PRIVATE ${CMAKE_SOURCE_DIR}/tests)
    target_compile_definitions(${PROJECT_NAME}.elf PRIVATE STM32_SELF_TEST STM32_TARGET)
endif()

target_compile_options(${PROJECT_NAME}.elf PRIVATE
    -Wall
    -Wextra
    # Keep the C++ runtime small for this bare-metal firmware.
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti -fno-threadsafe-statics>
    $<$<CONFIG:Debug>:-Og -g3>
    $<$<CONFIG:Release>:-Os -flto>
)

target_link_libraries(${PROJECT_NAME}.elf PRIVATE
    freertos_kernel
)

target_link_options(${PROJECT_NAME}.elf PRIVATE
    -T${CMAKE_SOURCE_DIR}/STM32L4S5VITx_FLASH.ld
    # Preserve a map for memory inspection and cross-reference symbols.
    -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map,--cref
    $<$<CONFIG:Release>:-Os -flto>
)

# 5. Post-build: print Flash/RAM usage, generate .bin / .hex images, and mirror
# the final images to dist/<variant>/ in the workspace root.
#
# build/ is a Docker volume in the dev container, so it is visible inside the
# container only; dist/ lives in the workspace root, which is shared with the
# Windows host for flashing. Each configure gets its own subfolder so debug,
# release, and self-test builds do not overwrite each other.
if(STM32_SELF_TEST)
    set(FW_DIST_SUBDIR selftest)
elseif(CMAKE_BUILD_TYPE)
    string(TOLOWER "${CMAKE_BUILD_TYPE}" FW_DIST_SUBDIR)
else()
    set(FW_DIST_SUBDIR default)
endif()

set(FW_DIST_DIR "${CMAKE_SOURCE_DIR}/dist/${FW_DIST_SUBDIR}")

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_COMMAND} -E make_directory ${FW_DIST_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:${PROJECT_NAME}.elf>
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
        ${FW_DIST_DIR}/
)