/**
 * @file notes.hpp
 * @brief Standard musical note frequencies in Hertz (Hz) for passive buzzers.
 */

#pragma once

#include <cstdint>

namespace components::actuators::buzzer::notes
{

constexpr std::uint16_t REST = 0;

// Octave 4
constexpr std::uint16_t NOTE_C4  = 262;
constexpr std::uint16_t NOTE_CS4 = 277;
constexpr std::uint16_t NOTE_D4  = 294;
constexpr std::uint16_t NOTE_DS4 = 311;
constexpr std::uint16_t NOTE_E4  = 330;
constexpr std::uint16_t NOTE_F4  = 349;
constexpr std::uint16_t NOTE_FS4 = 370;
constexpr std::uint16_t NOTE_G4  = 392;
constexpr std::uint16_t NOTE_GS4 = 415;
constexpr std::uint16_t NOTE_A4  = 440;
constexpr std::uint16_t NOTE_AS4 = 466;
constexpr std::uint16_t NOTE_B4  = 494;

// Octave 5
constexpr std::uint16_t NOTE_C5  = 523;
constexpr std::uint16_t NOTE_CS5 = 554;
constexpr std::uint16_t NOTE_D5  = 587;
constexpr std::uint16_t NOTE_DS5 = 617;
constexpr std::uint16_t NOTE_E5  = 659;
constexpr std::uint16_t NOTE_F5  = 698;
constexpr std::uint16_t NOTE_FS5 = 740;
constexpr std::uint16_t NOTE_G5  = 784;
constexpr std::uint16_t NOTE_GS5 = 831;
constexpr std::uint16_t NOTE_A5  = 880;
constexpr std::uint16_t NOTE_AS5 = 932;
constexpr std::uint16_t NOTE_B5  = 988;

// Octave 6
constexpr std::uint16_t NOTE_C6  = 1047;
constexpr std::uint16_t NOTE_D6  = 1175;
constexpr std::uint16_t NOTE_E6  = 1319;
constexpr std::uint16_t NOTE_G6  = 1568;
constexpr std::uint16_t NOTE_A6  = 1760;

// High-frequency alerts
constexpr std::uint16_t BEEP_LOW   = 1000;
constexpr std::uint16_t BEEP_MID   = 2000;
constexpr std::uint16_t BEEP_HIGH  = 3000;
constexpr std::uint16_t BEEP_ALARM = 4000;

} // namespace components::actuators::buzzer::notes
