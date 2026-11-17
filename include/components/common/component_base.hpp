/**
 * @file component_base.hpp
 * @brief Common status codes, return types, and interface contracts for STM32-Components.
 */

#pragma once

#include <cstdint>

namespace components
{

/**
 * @brief Universal status/return code for all component driver operations.
 */
enum class Status : std::uint8_t
{
    Ok = 0,             ///< Operation completed successfully
    Error,              ///< General non-specified error
    Timeout,            ///< Communication or hardware response timeout
    DeviceNotFound,     ///< Device failed WHO_AM_I or ACK verification
    BusError,           ///< Physical bus arbitration/nack error
    InvalidParam,       ///< Parameter value out of bounds
    Busy,               ///< Peripheral or hardware device busy
    NotSupported        ///< Feature not supported by this hardware mode
};

/**
 * @brief Convenience helper to evaluate driver success.
 */
[[nodiscard]] constexpr bool isOk(Status status) noexcept
{
    return status == Status::Ok;
}

/**
 * @brief Pin connection policy for hardware pins that may either be MCU-controlled
 * or hardwired on a PCB (e.g. STBY tied to VCC, CS pulled high/low, AD0 tied to GND/VCC).
 */
enum class PinConnection : std::uint8_t
{
    McuControlled,      ///< Pin is actively wired to and controlled by an STM32 GPIO
    HardwareTiedHigh,   ///< Pin is physically hardwired to VCC on the PCB
    HardwareTiedLow     ///< Pin is physically hardwired to GND on the PCB
};

} // namespace components
