# STM32-Components: Off-Chip Device Drivers (Tier 2)

A production-grade, zero-dynamic-allocation C++17 device driver library for off-chip external sensors, actuators, and communication modules on STM32 microcontrollers (targeting **STM32F446RE**).

---

## 1. 4-Tier Robotics Architecture

This repository forms **Tier 2** of the standardized 4-tier embedded robotics software stack:

```
┌─────────────────────────────────────────────────────────────┐
│ Tier 4: Product Repositories                                │
│ (STM32-Line-Follower-Robot, STM32-Micromouse)               │
└──────────────────────────┬──────────────────────────────────┘
                           │
       ┌───────────────────┴───────────────────┐
       ▼                                       ▼
┌───────────────────────────────┐   ┌─────────────────────────┐
│ Tier 2: STM32-Components      │   │ Tier 3:                 │
│ (Off-chip device drivers:     │   │ Embedded-Algorithms     │
│  TB6612FNG, QTR-8A, HC-05,    │   │ (Pure C++17 math, PID,  │
│  Encoders, Buzzers)           │   │  kinematics, filtering) │
└──────────────┬────────────────┘   └─────────────────────────┘
               │                               ▲
               ▼                               │ (Unit tested on PC)
┌───────────────────────────────┐              │
│ Tier 1: STM32-Library         ├──────────────┘
│ (Silicon MCU HAL: GPIO, Timer,│
│  ADC, UART, SPI, I2C, DMA)    │
└───────────────────────────────┘
```

- **Tier 1 (`STM32-Library`)**: Bare-metal MCU silicon peripheral HAL (registers, clocks, interrupts, DMA).
- **Tier 2 (`STM32-Components`)**: External component drivers that depend purely on physical MCU abstractions (GPIO, Timer PWM, ADC, UART, SysTick).
- **Tier 3 (`Embedded-Algorithms`)**: Hardware-independent mathematics, filters, state machines, and control loops (100% host-testable without MCU headers).
- **Tier 4 (Products)**: High-level robot firmware coordinating algorithms and component drivers to fulfill mission goals.

---

## 2. Supported Device Driver Catalog

All drivers adhere to embedded best practices:
- **Zero dynamic memory allocation** (0 bytes heap; no `malloc`, `new`, or standard dynamic containers).
- **Compile-time safety & constexpr configuration** (scoped enums, template parameters for timers/UARTs).
- **Non-blocking execution** (SysTick timebases, circular ring buffers, zero CPU stalling).

| Driver Subsystem | Header Path | Key Features | Hardware Interfaces |
| :--- | :--- | :--- | :--- |
| **TB6612FNG Motor Driver** | [`tb6612fng.hpp`](include/components/actuators/tb6612fng/tb6612fng.hpp) | 20 kHz ultrasonic PWM, bare-IC hardware-tied VCC vs MCU GPIO standby, input deadband compensation, overcoming-stiction min duty, dynamic short-brake, differential drive steering mixer. | Timer PWM, GPIO direction pins |
| **Quadrature Encoder** | [`encoder.hpp`](include/components/actuators/encoder/encoder.hpp) | Silicon timer encoder mode (TIM1–TIM5, TIM8), 4x quadrature decoding with 0% CPU overhead, 32-bit overflow handling, velocity (RPM, m/s) and distance calculation. | Hardware Timer encoder input (CH1/CH2) |
| **Active Buzzer** | [`buzzer.hpp`](include/components/actuators/buzzer/buzzer.hpp) | Non-blocking alert patterns and pulse trains (battery alarms, heartbeat) with millisecond precision. | GPIO digital output |
| **Passive Piezo Buzzer** | [`buzzer.hpp`](include/components/actuators/buzzer/buzzer.hpp) | Timer PWM frequency synthesis, pitch notes ([`notes.hpp`](include/components/actuators/buzzer/notes.hpp)), non-blocking melody sequencer. | Timer PWM |
| **QTR Analog Reflectance Array** | [`qtr_analog.hpp`](include/components/sensors/qtr/qtr_analog.hpp) | Supports cheap Amazon clone modules (QTR-8A) via dynamic min/max calibration sweeps; modular custom boards via MOSFET emitter pulsing (AutoDifferential ambient subtraction); weighted centroid estimation with lost-line memory. | ADC channels, optional emitter GPIO |
| **HC-05 Bluetooth SPP** | [`hc05.hpp`](include/components/wireless/hc05/hc05.hpp) | Transparent SPP streaming, line & framed packet parsing (`<CMD:DATA>`), hardware connection detection via STATE pin, dynamic AT command configuration via EN/KEY pin. | UART / USART, optional STATE/KEY GPIOs |
| **Component Base** | [`component_base.hpp`](include/components/common/component_base.hpp) | Universal `components::Status` error codes and `PinConnection` (McuControlled, HardwareTiedHigh, HardwareTiedLow) policy. | Common |

---

## 3. Resolving the Submodule Russian Doll Anti-Pattern

Previously, downstream projects (`STM32-Line-Follower-Robot`, `STM32-Projects`) submoduling both `STM32-Components` and `STM32-Library` encountered conflicting nested submodules and build ambiguity.

`STM32-Components` now provides a dual-mode build system:

1. **Downstream / Parent Project Mode**:
   If `stm32::hal` is already defined by the parent project (e.g. `add_subdirectory(lib/STM32-Library)`), `STM32-Components` automatically consumes that target without rebuilding or duplicating silicon drivers.
2. **Standalone / Example Driver Mode**:
   If building standalone driver examples, `CMakeLists.txt` and `Makefile` automatically detect `../STM32-Library` (sibling repository) or fallback to `lib/STM32-Library` (git submodule).

---

## 4. Downstream Integration Guide

### Integration with Modern CMake

In your parent project's `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_robot C CXX ASM)

# 1. Include Tier 1 (MCU HAL)
add_subdirectory(lib/STM32-Library) # Defines stm32::hal

# 2. Include Tier 2 (Device Drivers)
add_subdirectory(lib/STM32-Components) # Defines stm32::components

# 3. Link your firmware executable against Tier 2
add_executable(my_robot.elf src/main.cpp)

# Automatically brings in component headers, HAL headers, and static library symbols!
target_link_libraries(my_robot.elf PRIVATE
    stm32::components
)
```

### Integration with GNU Make

In your parent project's `Makefile`:

```makefile
# Include Tier 1 and Tier 2 include paths
INCLUDES += \
    -Ilib/STM32-Components/include \
    -Ilib/STM32-Library/Inc \
    -Ilib/STM32-Library/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
    -Ilib/STM32-Library/Drivers/CMSIS/Include
```

---

## 5. Standalone Build & Example Verification

The repository includes five comprehensive example applications demonstrating full hardware capabilities on the **STM32F446RE Nucleo-64** board:

| Example Target | Source Path | Description |
| :--- | :--- | :--- |
| `smoke_test` | [`examples/smoke_test/main.cpp`](examples/smoke_test/main.cpp) | Sanity check verifying SysTick, GPIO, and component base status evaluation. |
| `tb6612_motors` | [`examples/actuators/tb6612_motors/main.cpp`](examples/actuators/tb6612_motors/main.cpp) | 20 kHz ultrasonic PWM motor control, stiction compensation, and braking. |
| `hc05_telemetry` | [`examples/wireless/hc05_telemetry/main.cpp`](examples/wireless/hc05_telemetry/main.cpp) | Transparent SPP Bluetooth telemetry, framed packet parser, STATE pin detection. |
| `qtr_line_follower` | [`examples/sensors/qtr_line_follower/main.cpp`](examples/sensors/qtr_line_follower/main.cpp) | 8-sensor reflectance calibration sweep and centroid calculation with lost-line memory. |
| `closed_loop_robot` | [`examples/control/closed_loop_robot/main.cpp`](examples/control/closed_loop_robot/main.cpp) | Full system integration: dual motors, encoder velocity feedback, QTR sensors, buzzer, and Bluetooth. |

### Build with GNU Make

```bash
# Build default smoke test (auto-resolves ../STM32-Library or lib/STM32-Library)
make all

# Build specific component example
make EXAMPLE=examples/actuators/tb6612_motors all
make EXAMPLE=examples/wireless/hc05_telemetry all
make EXAMPLE=examples/sensors/qtr_line_follower all
make EXAMPLE=examples/control/closed_loop_robot all

# Override STM32-Library path explicitly
make EXAMPLE=examples/smoke_test LIB_DIR=../STM32-Library all
```

### Build with CMake & Ninja

```bash
# Sibling resolution (default)
cmake -B build -GNinja
ninja -C build

# Explicit submodule resolution
cmake -B build -GNinja -DSTM32_LIBRARY_DIR=lib/STM32-Library
ninja -C build
```

---

## 6. Tier 3 Algorithm Migration & Deprecation Notices

To enable pure host-PC unit testing without MCU hardware dependencies, mathematical and control algorithms have moved to **Tier 3 (`Embedded-Algorithms`)**:

- `components/common/pid_controller.hpp` -> Migrate to `algorithms/control/pid_controller.hpp`
- `components/common/math_utils.hpp` -> Migrate to `algorithms/math/math_utils.hpp`

Both headers are retained in `STM32-Components` for backwards compatibility. If legacy projects need to suppress deprecation warnings during migration, define:

```cpp
#define COMPONENTS_DISABLE_DEPRECATION_WARNINGS
```
before including these headers.

---

## 7. Directory Structure

```
STM32-Components/
├── CMakeLists.txt                         # Modern CMake configuration (exports stm32::components)
├── Makefile                               # Standalone GNU Makefile for example builds
├── README.md                              # Repository architecture and integration guide
├── .gitmodules                            # Submodule configuration for lib/STM32-Library
├── lib/
│   └── STM32-Library/                     # Tier 1 MCU HAL submodule
├── include/
│   └── components/
│       ├── common/
│       │   ├── component_base.hpp         # Status enum, PinConnection policies
│       │   ├── math_utils.hpp             # [Deprecated] Hardware-independent math
│       │   └── pid_controller.hpp         # [Deprecated] Discrete PID controller
│       ├── actuators/
│       │   ├── tb6612fng/                 # Toshiba TB6612FNG Dual H-Bridge Driver
│       │   ├── encoder/                   # Hardware Timer Quadrature Encoder Driver
│       │   └── buzzer/                    # Active and Passive Buzzer Drivers
│       ├── sensors/
│       │   └── qtr/                       # QTR-8A / Analog Reflectance Sensor Driver
│       └── wireless/
│           └── hc05/                      # HC-05 Bluetooth SPP Driver
└── examples/
    ├── smoke_test/                        # Baseline hardware sanity test
    ├── actuators/tb6612_motors/           # Dual DC motor control example
    ├── wireless/hc05_telemetry/           # Bluetooth telemetry & packet parsing
    ├── sensors/qtr_line_follower/         # Analog line array calibration & tracking
    └── control/closed_loop_robot/         # Integrated closed-loop robotics system
```
