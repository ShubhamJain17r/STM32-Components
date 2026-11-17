/**
 * @file qtr_types.hpp
 * @brief Configuration enums, calibration data, and result structures for QTR reflectance sensor arrays.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include "components/common/component_base.hpp"

namespace components::sensors::qtr
{

/**
 * @brief Line contrast orientation.
 */
enum class LineColor : std::uint8_t
{
    DarkOnLight = 0,    ///< Standard black tape line on white background (default)
    LightOnDark = 1     ///< White line on black / dark surface
};

/**
 * @brief Emitter LED modulation policy.
 */
enum class EmitterPolicy : std::uint8_t
{
    AlwaysOn = 0,           ///< Emitters are permanently on (standard for Cheap Amazon Clones or PCB VCC tied)
    ManualControl = 1,      ///< Emitters manually controlled via enable/disable calls
    AutoDifferential = 2    ///< Active ambient light rejection (samples LED-OFF then LED-ON and subtracts ambient)
};

/**
 * @brief Per-sensor dynamic calibration bounds.
 *
 * @tparam NumSensors Number of optical channels in the array.
 */
template<std::size_t NumSensors>
struct CalibrationData
{
    std::array<std::uint16_t, NumSensors> minimum;  ///< Baseline reading over white floor (raw ADC count)
    std::array<std::uint16_t, NumSensors> maximum;  ///< Baseline reading over black line (raw ADC count)
    bool initialized = false;

    constexpr CalibrationData() noexcept
    {
        reset();
    }

    constexpr void reset() noexcept
    {
        for (std::size_t i = 0; i < NumSensors; ++i)
        {
            minimum[i] = 0xFFFF; // Start with max possible value to capture minimum
            maximum[i] = 0x0000; // Start with 0 to capture maximum
        }
        initialized = false;
    }
};

/**
 * @brief Complete line estimation result.
 */
struct LineResult
{
    std::int32_t  position;         ///< Estimated centroid position [0 .. (NumSensors - 1) * 1000]
    std::uint32_t totalReflection;  ///< Sum of normalized reflections across all channels
    bool          lineDetected;     ///< True if totalReflection exceeds detection threshold
};

} // namespace components::sensors::qtr
