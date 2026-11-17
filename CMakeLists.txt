cmake_minimum_required(VERSION 3.16)

# Set ARM toolchain if not explicitly specified
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32-Library/cmake/arm-none-eabi-gcc.cmake")
endif()

project(stm32_components C CXX ASM)

# Standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Target Processor Flags (STM32F446RE: Cortex-M4 with single-precision FPU)
set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# Compiler Diagnostic & Embedded Optimization Options
add_compile_options(
    ${MCU_FLAGS}
    -Wall
    -Wextra
    -Wpedantic
    -ffunction-sections
    -fdata-sections
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

# Hardware Definition
add_definitions(-DSTM32F446xx)

# Default to Release (-O2) if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3 -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O2 -g0 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g0 -DNDEBUG")

# Paths
set(LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32-Library")
set(LINKER_SCRIPT "${LIB_DIR}/STM32F446RETX_FLASH.ld")

# ------------------------------------------------------------------------------
# Target: stm32_core (Layer 1 Bare-Metal Engine)
# ------------------------------------------------------------------------------
set(STM32_CORE_SOURCES
    ${LIB_DIR}/Src/syscalls.c
    ${LIB_DIR}/Src/sysmem.c
    ${LIB_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
    ${LIB_DIR}/Startup/startup_stm32f446retx.s

    ${LIB_DIR}/Inc/stm32/common/systick/systick.cpp
    ${LIB_DIR}/Inc/stm32/gpio/digital_input/digital_input.cpp
    ${LIB_DIR}/Inc/stm32/gpio/digital_output/digital_output.cpp
    ${LIB_DIR}/Inc/stm32/gpio/interrupt_input/interrupt_input.cpp
    ${LIB_DIR}/Inc/stm32/exti/exti_manager.cpp
    ${LIB_DIR}/Inc/stm32/exti/exti_irq.cpp
    ${LIB_DIR}/Inc/stm32/timer/interrupt/timer_interrupt.cpp
    ${LIB_DIR}/Inc/stm32/timer/interrupt/timer_irq.cpp
    ${LIB_DIR}/Inc/stm32/uart/interrupt/uart_interrupt.cpp
    ${LIB_DIR}/Inc/stm32/uart/interrupt/uart_irq.cpp
    ${LIB_DIR}/Inc/stm32/spi/interrupt/spi_interrupt.cpp
    ${LIB_DIR}/Inc/stm32/spi/interrupt/spi_irq.cpp
    ${LIB_DIR}/Inc/stm32/i2c/interrupt/i2c_interrupt.cpp
    ${LIB_DIR}/Inc/stm32/i2c/interrupt/i2c_irq.cpp
    ${LIB_DIR}/Inc/stm32/adc/interrupt/adc_interrupt.cpp
    ${LIB_DIR}/Inc/stm32/adc/interrupt/adc_irq.cpp
    ${LIB_DIR}/Inc/stm32/dma/interrupt/dma_interrupt.cpp
    ${LIB_DIR}/Inc/stm32/dma/interrupt/dma_irq.cpp
)

add_library(stm32_core STATIC ${STM32_CORE_SOURCES})

target_include_directories(stm32_core PUBLIC
    ${LIB_DIR}/Inc
    ${LIB_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${LIB_DIR}/Drivers/CMSIS/Include
)

# ------------------------------------------------------------------------------
# Target: stm32_components (Layer 2 External Device Driver Library)
# ------------------------------------------------------------------------------
add_library(stm32_components INTERFACE)

target_include_directories(stm32_components INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(stm32_components INTERFACE
    stm32_core
)

# ------------------------------------------------------------------------------
# Helper Function: Register an Example Executable
# ------------------------------------------------------------------------------
function(add_component_example TARGET_NAME SOURCE_FILE)
    add_executable(${TARGET_NAME}.elf ${SOURCE_FILE})

    target_link_libraries(${TARGET_NAME}.elf PRIVATE
        stm32_components
    )

    target_link_options(${TARGET_NAME}.elf PRIVATE
        ${MCU_FLAGS}
        -T${LINKER_SCRIPT}
        -Wl,-Map=${TARGET_NAME}.map,--cref
        -Wl,--gc-sections
        -specs=nano.specs
        -specs=nosys.specs
    )

    add_custom_command(TARGET ${TARGET_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET_NAME}.elf ${TARGET_NAME}.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex   ${TARGET_NAME}.elf ${TARGET_NAME}.hex
        COMMAND ${CMAKE_SIZE} --format=berkeley ${TARGET_NAME}.elf
        COMMENT "Generated ${TARGET_NAME}.bin / .hex, memory layout:"
    )
endfunction()

# ------------------------------------------------------------------------------
# Registered Examples
# ------------------------------------------------------------------------------
add_component_example(smoke_test examples/smoke_test/main.cpp)
add_component_example(tb6612_motors examples/actuators/tb6612_motors/main.cpp)
add_component_example(hc05_telemetry examples/wireless/hc05_telemetry/main.cpp)
