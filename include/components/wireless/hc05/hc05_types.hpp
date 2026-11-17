/**
 * @file hc05_types.hpp
 * @brief Configuration enums, baud rates, and status codes for HC-05 Bluetooth module.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "components/common/component_base.hpp"

namespace components::wireless::hc05
{

/**
 * @brief HC-05 Bluetooth Operating Role.
 */
enum class Role : std::uint8_t
{
    Slave = 0,          ///< Slave mode (waits for connections from phone/PC/master)
    Master = 1,         ///< Master mode (actively initiates SPP connection to slave)
    SlaveLoop = 2       ///< Slave loopback mode (echoes received data)
};

/**
 * @brief Hardware connection status reported by the module's STATE pin.
 */
enum class ConnectionState : std::uint8_t
{
    Disconnected = 0,   ///< Searching, pairing, or standby (STATE pin LOW)
    Connected = 1,      ///< Paired and actively connected over SPP (STATE pin HIGH)
    Unknown = 2         ///< STATE pin not monitored by MCU
};

/**
 * @brief Module operational mode.
 */
enum class OperatingMode : std::uint8_t
{
    Data = 0,           ///< Transparent SPP serial data mode (KEY pin LOW)
    AtCommand = 1       ///< AT command configuration mode (KEY pin HIGH, 38400 baud)
};

/**
 * @brief Stop bits configuration for AT+UART command.
 */
enum class StopBits : std::uint8_t
{
    One = 0,
    Two = 1
};

/**
 * @brief Parity configuration for AT+UART command.
 */
enum class Parity : std::uint8_t
{
    None = 0,
    Odd = 1,
    Even = 2
};

/**
 * @brief Default timing constants for HC-05 communication.
 */
struct Defaults
{
    static constexpr std::uint32_t DataBaudRate     = 9600;     ///< Factory default SPP baud
    static constexpr std::uint32_t AtCommandBaudRate = 38400;    ///< Standard HC-05 AT mode baud
    static constexpr std::uint32_t AtTimeoutMs       = 1000;     ///< Millisecond timeout for AT responses
    static constexpr std::uint32_t LineTimeoutMs     = 200;      ///< Millisecond timeout for line reception
    static constexpr std::uint32_t ModeSwitchDelayMs = 100;      ///< Delay after toggling KEY pin
};

} // namespace components::wireless::hc05
