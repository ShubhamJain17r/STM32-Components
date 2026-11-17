/**
 * @file math_utils.hpp
 * @brief Zero-overhead math utilities for sensor scaling, motor deadband, and calibration.
 */

#pragma once

#include <cstdint>
#include <type_traits>

namespace components::math
{

/**
 * @brief Clamps a value between min and max bounds.
 */
template<typename T>
[[nodiscard]] constexpr T clamp(T value, T minVal, T maxVal) noexcept
{
    return (value < minVal) ? minVal : ((value > maxVal) ? maxVal : value);
}

/**
 * @brief Computes absolute value for integral and floating point types.
 */
template<typename T>
[[nodiscard]] constexpr T abs(T value) noexcept
{
    return (value < static_cast<T>(0)) ? -value : value;
}

/**
 * @brief Linearly maps a value from one numerical range to another.
 */
template<typename T, typename InRange, typename OutRange>
[[nodiscard]] constexpr OutRange mapRange(T value, InRange inMin, InRange inMax, OutRange outMin, OutRange outMax) noexcept
{
    if (inMax == inMin)
    {
        return outMin;
    }
    return outMin + static_cast<OutRange>((value - inMin) * (outMax - outMin) / (inMax - inMin));
}

/**
 * @brief Applies symmetrical deadband compensation around zero.
 * Values strictly inside [-deadband, +deadband] evaluate to 0.0f.
 * Values outside are linearly rescaled so that movement starts smoothly at zero beyond the deadband.
 *
 * @param value Normalized input value in [-1.0f, +1.0f]
 * @param deadband Positive deadband threshold in [0.0f, 1.0f)
 * @return Normalized, deadband-compensated value in [-1.0f, +1.0f]
 */
[[nodiscard]] constexpr float applyDeadband(float value, float deadband) noexcept
{
    if (deadband <= 0.0f)
    {
        return clamp(value, -1.0f, 1.0f);
    }
    if (deadband >= 1.0f)
    {
        return 0.0f;
    }

    if (value > deadband)
    {
        return clamp((value - deadband) / (1.0f - deadband), 0.0f, 1.0f);
    }
    else if (value < -deadband)
    {
        return clamp((value + deadband) / (1.0f - deadband), -1.0f, 0.0f);
    }
    else
    {
        return 0.0f;
    }
}

/**
 * @brief Single-pole infinite impulse response (IIR) exponential moving average filter.
 * y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
 */
class ExponentialFilter
{
public:
    constexpr explicit ExponentialFilter(float alpha = 0.2f, float initialValue = 0.0f) noexcept
        : alpha_(clamp(alpha, 0.0f, 1.0f)), current_(initialValue), initialized_(false)
    {}

    float update(float input) noexcept
    {
        if (!initialized_)
        {
            current_ = input;
            initialized_ = true;
            return current_;
        }
        current_ = (alpha_ * input) + ((1.0f - alpha_) * current_);
        return current_;
    }

    [[nodiscard]] float get() const noexcept { return current_; }
    void setAlpha(float alpha) noexcept { alpha_ = clamp(alpha, 0.0f, 1.0f); }
    void reset(float val = 0.0f) noexcept
    {
        current_ = val;
        initialized_ = false;
    }

private:
    float alpha_;
    float current_;
    bool  initialized_;
};

} // namespace components::math
