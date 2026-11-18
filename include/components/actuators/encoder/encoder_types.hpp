/**
 * @file encoder_types.hpp
 * @brief Configuration enums and structures for STM32 hardware timer quadrature encoders.
 */

#pragma once

#include <cstdint>

namespace components::actuators::encoder
{

/**
 * @brief Hardware quadrature decoding resolution mode.
 */
enum class DecodingMode : std::uint8_t
{
    X2_TI1 = 1,     ///< Count edges on Channel 1 only (2x resolution)
    X2_TI2 = 2,     ///< Count edges on Channel 2 only (2x resolution)
    X4_Both = 3     ///< Count edges on both Channel 1 & Channel 2 (4x full resolution, recommended)
};

/**
 * @brief Configuration parameters for a quadrature rotary encoder.
 */
struct EncoderConfig
{
    std::uint32_t countsPerRevolution = 1440;       ///< Total decoded ticks per wheel revolution (e.g. 12 CPR * 30:1 * 4x = 1440)
    float         wheelDiameterMeters = 0.032f;     ///< Physical wheel diameter in meters (default: 32 mm)
    bool          inverted            = false;      ///< Invert positive rotation direction
    std::uint8_t  inputFilter         = 0x03;       ///< Digital filter (0x0 = none, 0x3 = 4 cycles, 0x6 = 8 cycles)
    DecodingMode  decodingMode        = DecodingMode::X4_Both; ///< Quad decoding mode
};

} // namespace components::actuators::encoder
