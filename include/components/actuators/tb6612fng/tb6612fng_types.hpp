/**
 * @file tb6612fng_types.hpp
 * @brief Data types, enums, and configuration structures for TB6612FNG motor driver.
 */

#pragma once

#include <cstdint>
#include "components/common/component_base.hpp"

namespace components::actuators::tb6612fng
{

/**
 * @brief Operating direction state for an individual H-bridge motor channel.
 */
enum class Direction : std::uint8_t
{
    Standby = 0,    ///< High-impedance standby (power stage disabled)
    Coast,          ///< Free-wheeling / high-impedance stop (IN1=LOW, IN2=LOW)
    Forward,        ///< Forward rotation (IN1=HIGH, IN2=LOW)
    Reverse,        ///< Reverse rotation (IN1=LOW, IN2=HIGH)
    Brake           ///< Active dynamic short-brake (IN1=HIGH, IN2=HIGH)
};

/**
 * @brief Behavior when commanded speed is 0.0f.
 */
enum class ZeroSpeedMode : std::uint8_t
{
    Brake = 0,      ///< Actively short motor terminals to GND (firm stop / hold)
    Coast           ///< Leave motor terminals open / High-Z (freewheel to a stop)
};

/**
 * @brief Configuration parameters for a single motor channel.
 */
struct ChannelConfig
{
    bool          inverted      = false;            ///< Invert positive speed direction
    float         deadband      = 0.0f;             ///< Input deadband threshold in [0.0f, 1.0f)
    float         minDuty       = 0.0f;             ///< Minimum overcome-stiction duty cycle in [0.0f, 1.0f)
    ZeroSpeedMode zeroSpeedMode = ZeroSpeedMode::Brake; ///< Action on zero speed command
};

} // namespace components::actuators::tb6612fng
