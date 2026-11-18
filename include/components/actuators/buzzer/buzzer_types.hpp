/**
 * @file buzzer_types.hpp
 * @brief Data structures and enums for Active and Passive buzzers.
 */

#pragma once

#include <cstdint>

namespace components::actuators::buzzer
{

/**
 * @brief Hardware buzzer type.
 */
enum class BuzzerType : std::uint8_t
{
    Active,     ///< Active buzzer with internal oscillator (toggled via GPIO HIGH/LOW)
    Passive     ///< Passive piezoelectric transducer (driven via Timer PWM frequency)
};

/**
 * @brief A single note or tone entry in a melody sequence.
 */
struct Note
{
    std::uint16_t frequency;    ///< Tone frequency in Hz (0 = REST/pause)
    std::uint16_t durationMs;   ///< Note sounding duration in milliseconds
};

/**
 * @brief Simple repeating beep pattern for alerts (e.g. low-battery alarm).
 */
struct BeepPattern
{
    std::uint16_t onMs;         ///< Sound duration
    std::uint16_t offMs;        ///< Silence duration
    std::uint8_t  repeatCount;  ///< Number of beeps (0 = infinite / continuous)
};

} // namespace components::actuators::buzzer
