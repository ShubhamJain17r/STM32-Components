/**
 * @file pid_controller.hpp
 * @brief Production-grade C++17 discrete PID controller with anti-windup and derivative filtering.
 *
 * @deprecated [Tier 3 Migration Notice]
 * Hardware-independent control algorithms are being migrated to Tier 3 'Embedded-Algorithms'
 * so they can be unit-tested on host PCs without MCU headers.
 * This file is retained in STM32-Components purely for backwards compatibility and will be
 * removed in a future release. Please migrate to Embedded-Algorithms control algorithms.
 *
 * Designed for embedded real-time robotics:
 * - Line-follower steering error correction
 * - Closed-loop DC motor velocity control
 * - Anti-windup integral clamping
 * - Low-pass filtered derivative (eliminates high-frequency sensor noise)
 * - Proportional-on-Measurement / Feedforward support
 * - Zero dynamic allocation (0 bytes heap)
 */

#pragma once

#include <cstdint>
#include "components/common/math_utils.hpp"

#if defined(COMPONENTS_DISABLE_DEPRECATION_WARNINGS)
#define COMPONENTS_PID_DEPRECATED
#else
#define COMPONENTS_PID_DEPRECATED [[deprecated("components::control PID utilities are deprecated; migrate to Tier 3 Embedded-Algorithms")]]
#endif

namespace components::control
{

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

/**
 * @brief Configuration parameters for PID controller.
 */
struct COMPONENTS_PID_DEPRECATED PidConfig
{
    float kp = 0.0f;            ///< Proportional gain
    float ki = 0.0f;            ///< Integral gain
    float kd = 0.0f;            ///< Derivative gain
    float kf = 0.0f;            ///< Feedforward gain (output += kf * setpoint)

    float outMin = -1.0f;       ///< Minimum output clamp bound
    float outMax =  1.0f;       ///< Maximum output clamp bound

    float intMin = -0.5f;       ///< Anti-windup minimum integral clamp
    float intMax =  0.5f;       ///< Anti-windup maximum integral clamp

    float dFilterAlpha = 0.7f;  ///< Derivative low-pass filter factor [0.0 = unfiltered, 1.0 = heavy filter]
};

/**
 * @brief Discrete-time PID controller.
 */
class COMPONENTS_PID_DEPRECATED PidController
{
public:
    constexpr explicit PidController(const PidConfig& config = {}) noexcept
        : config_(config),
          integral_(0.0f),
          lastError_(0.0f),
          filteredDerivative_(0.0f),
          lastMeasurement_(0.0f),
          firstRun_(true)
    {}

    /**
     * @brief Compute PID output given setpoint and process variable.
     *
     * @param setpoint Desired target value
     * @param measurement Current measured value
     * @param dt Time elapsed since last update in seconds
     * @return Clamped controller output in [outMin, outMax]
     */
    float update(float setpoint, float measurement, float dt) noexcept
    {
        if (dt <= 0.0f)
        {
            return 0.0f;
        }

        const float error = setpoint - measurement;
        return calculateInternal(setpoint, measurement, error, dt);
    }

    /**
     * @brief Compute PID output directly from a pre-calculated error term.
     *
     * @param error Tracking error (setpoint - measurement)
     * @param dt Time elapsed since last update in seconds
     * @return Clamped controller output in [outMin, outMax]
     */
    float updateError(float error, float dt) noexcept
    {
        if (dt <= 0.0f)
        {
            return 0.0f;
        }

        return calculateInternal(0.0f, 0.0f, error, dt);
    }

    /**
     * @brief Reset internal integrator and derivative states.
     */
    void reset() noexcept
    {
        integral_ = 0.0f;
        lastError_ = 0.0f;
        filteredDerivative_ = 0.0f;
        lastMeasurement_ = 0.0f;
        firstRun_ = true;
    }

    // Gain accessors & mutators
    void setGains(float kp, float ki, float kd, float kf = 0.0f) noexcept
    {
        config_.kp = kp;
        config_.ki = ki;
        config_.kd = kd;
        config_.kf = kf;
    }

    void setOutputLimits(float minVal, float maxVal) noexcept
    {
        config_.outMin = minVal;
        config_.outMax = maxVal;
    }

    void setIntegralLimits(float minVal, float maxVal) noexcept
    {
        config_.intMin = minVal;
        config_.intMax = maxVal;
    }

    void setDerivativeFilterAlpha(float alpha) noexcept
    {
        config_.dFilterAlpha = math::clamp(alpha, 0.0f, 1.0f);
    }

    [[nodiscard]] float getKp() const noexcept { return config_.kp; }
    [[nodiscard]] float getKi() const noexcept { return config_.ki; }
    [[nodiscard]] float getKd() const noexcept { return config_.kd; }
    [[nodiscard]] float getKf() const noexcept { return config_.kf; }
    [[nodiscard]] float getIntegral() const noexcept { return integral_; }
    [[nodiscard]] const PidConfig& getConfig() const noexcept { return config_; }

private:
    PidConfig config_;
    float     integral_;
    float     lastError_;
    float     filteredDerivative_;
    float     lastMeasurement_;
    bool      firstRun_;

    float calculateInternal(float setpoint, float measurement, float error, float dt) noexcept
    {
        // 1. Proportional term
        const float pTerm = config_.kp * error;

        // 2. Integral term with anti-windup clamping
        integral_ += (config_.ki * error * dt);
        integral_ = math::clamp(integral_, config_.intMin, config_.intMax);

        // 3. Derivative term with low-pass filtering
        float rawDerivative = 0.0f;
        if (!firstRun_)
        {
            rawDerivative = (error - lastError_) / dt;
            filteredDerivative_ = (config_.dFilterAlpha * filteredDerivative_) +
                                  ((1.0f - config_.dFilterAlpha) * rawDerivative);
        }
        else
        {
            filteredDerivative_ = 0.0f;
            firstRun_ = false;
        }

        const float dTerm = config_.kd * filteredDerivative_;

        // 4. Feedforward term
        const float fTerm = config_.kf * setpoint;

        // 5. Total Output with saturation clamping
        const float output = pTerm + integral_ + dTerm + fTerm;

        // Store states for next iteration
        lastError_ = error;
        lastMeasurement_ = measurement;

        return math::clamp(output, config_.outMin, config_.outMax);
    }
};

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

} // namespace components::control
