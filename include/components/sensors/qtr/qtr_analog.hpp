/**
 * @file qtr_analog.hpp
 * @brief Production-grade C++17 driver for Analog Infrared Reflectance Sensor Arrays (QTR-8A / QTR-Analog).
 *
 * Designed to support both:
 * 1. Cheap Amazon / Generic Clone Modules (QTR-8A):
 *    - Per-channel dynamic min/max calibration completely cancels out loose-tolerance phototransistors
 *    - Emitters permanently on (no emitter control pin required)
 * 2. Custom Modular Sensor Boards:
 *    - Configurable channel counts (4, 6, 8, 12, 16 sensors)
 *    - Active ambient light rejection via MOSFET emitter pulsing (AutoDifferential sampling)
 *    - Direct high-speed circular DMA buffer binding for zero-CPU line position estimation
 *    - Weighted centroid calculation with lost-line memory (prevents track departures on sharp curves)
 */

#pragma once

#include "stm32f446xx.h"
#include <cstdint>
#include <cstddef>
#include <array>

#include "stm32/gpio/gpio.hpp"
#include "stm32/adc/adc.hpp"
#include "stm32/common/systick/systick.hpp"

#include "components/common/component_base.hpp"
#include "components/sensors/qtr/qtr_types.hpp"

namespace components::sensors::qtr
{

/**
 * @brief High-precision analog reflectance sensor array driver.
 *
 * @tparam NumSensors Number of optical sensor channels (default: 8 for QTR-8A)
 */
template<std::size_t NumSensors = 8>
class QtrAnalog
{
    static_assert(NumSensors >= 2, "QtrAnalog requires at least 2 sensors for centroid calculation");
    static_assert(NumSensors <= 16, "Maximum 16 channels supported by standard ADC sequences");

public:
    QtrAnalog() = delete;
    ~QtrAnalog() = default;

    QtrAnalog(const QtrAnalog&) = delete;
    QtrAnalog& operator=(const QtrAnalog&) = delete;

    QtrAnalog(QtrAnalog&&) = default;
    QtrAnalog& operator=(QtrAnalog&&) = default;

    /**
     * @brief Constructor for Cheap Amazon Clones or Hardware-Tied Emitter Boards.
     * Emitters are permanently wired to VCC/GND (no MCU GPIO used for emitter control).
     *
     * @param pins Array of physical STM32 pins connected to sensor analog outputs
     * @param lineColor DarkOnLight (black line) or LightOnDark (white line)
     */
    explicit QtrAnalog(const std::array<gpio::Pin, NumSensors>& pins,
                       LineColor lineColor = LineColor::DarkOnLight) noexcept
        : pins_(pins),
          hasEmitterPin_(false),
          emitterPin_{nullptr, 0xFF},
          emitterActiveHigh_(true),
          lineColor_(lineColor),
          emitterPolicy_(EmitterPolicy::AlwaysOn),
          lastPosition_(getCenterPosition()),
          lostLineThreshold_(200U * static_cast<std::uint32_t>(NumSensors > 8 ? (NumSensors / 8) : 1)),
          lastLineDetected_(false)
    {}

    /**
     * @brief Constructor for Custom Modular Sensor Boards with MOSFET Emitter Control.
     * Enables active ambient light subtraction and low-power standby.
     *
     * @param pins Array of physical STM32 pins connected to sensor analog outputs
     * @param emitterPin STM32 GPIO driving the emitter MOSFET gate
     * @param lineColor DarkOnLight (black line) or LightOnDark (white line)
     * @param emitterActiveHigh True if HIGH turns on IR LEDs, False for P-MOSFET active-low
     * @param emitterPolicy Emitter modulation policy (AlwaysOn, ManualControl, AutoDifferential)
     */
    QtrAnalog(const std::array<gpio::Pin, NumSensors>& pins,
              gpio::Pin emitterPin,
              LineColor lineColor = LineColor::DarkOnLight,
              bool emitterActiveHigh = true,
              EmitterPolicy emitterPolicy = EmitterPolicy::AlwaysOn) noexcept
        : pins_(pins),
          hasEmitterPin_(true),
          emitterPin_(emitterPin),
          emitterActiveHigh_(emitterActiveHigh),
          lineColor_(lineColor),
          emitterPolicy_(emitterPolicy),
          lastPosition_(getCenterPosition()),
          lostLineThreshold_(200U * static_cast<std::uint32_t>(NumSensors > 8 ? (NumSensors / 8) : 1)),
          lastLineDetected_(false)
    {}

    /**
     * @brief Initialize all sensor pins as ANALOG inputs and emitter pin as OUTPUT.
     */
    bool init() noexcept
    {
        for (std::size_t i = 0; i < NumSensors; ++i)
        {
            pins_[i].enableClock();
            gpio::helper::setMode(pins_[i], gpio::Mode::ANALOG);
            gpio::helper::setPull(pins_[i], gpio::Pull::NONE);
        }

        if (hasEmitterPin_)
        {
            emitterPin_.enableClock();
            gpio::helper::setMode(emitterPin_, gpio::Mode::OUTPUT);
            gpio::helper::setOutputType(emitterPin_, gpio::OutputType::PUSH_PULL);
            gpio::helper::setOutputSpeed(emitterPin_, gpio::OutputSpeed::LOW);
            gpio::helper::setPull(emitterPin_, gpio::Pull::NONE);
            emittersOn();
        }

        resetCalibration();
        lastPosition_ = getCenterPosition();
        return true;
    }

    // =========================================================================
    // Emitter Control APIs (Custom Modular Board)
    // =========================================================================

    void emittersOn() noexcept
    {
        if (hasEmitterPin_)
        {
            gpio::DigitalOutput emitter{emitterPin_};
            emitter.write(emitterActiveHigh_ ? gpio::PinState::HIGH : gpio::PinState::LOW);
        }
    }

    void emittersOff() noexcept
    {
        if (hasEmitterPin_)
        {
            gpio::DigitalOutput emitter{emitterPin_};
            emitter.write(emitterActiveHigh_ ? gpio::PinState::LOW : gpio::PinState::HIGH);
        }
    }

    void setEmitterPolicy(EmitterPolicy policy) noexcept { emitterPolicy_ = policy; }
    [[nodiscard]] EmitterPolicy getEmitterPolicy() const noexcept { return emitterPolicy_; }
    [[nodiscard]] bool hasEmitterPin() const noexcept { return hasEmitterPin_; }

    // =========================================================================
    // Calibration APIs (Crucial for Cheap Amazon Clones)
    // =========================================================================

    /**
     * @brief Sample current surface and update calibration min/max bounds.
     * Call this in a loop for 2-4 seconds while sweeping the robot across the track!
     *
     * @tparam AdcInst STM32 ADC Instance
     * @param adc Reference to initialized AdcHandler
     */
    template<adc::Instance AdcInst>
    void calibrate(adc::AdcHandler<AdcInst>& adc) noexcept
    {
        std::array<std::uint16_t, NumSensors> raw;
        readRaw(adc, raw);
        updateCalibration(raw);
    }

    /**
     * @brief Update calibration bounds from a pre-acquired raw sample (e.g. DMA buffer).
     */
    void updateCalibration(const std::array<std::uint16_t, NumSensors>& rawSample) noexcept
    {
        for (std::size_t i = 0; i < NumSensors; ++i)
        {
            if (rawSample[i] < calibration_.minimum[i])
            {
                calibration_.minimum[i] = rawSample[i];
            }
            if (rawSample[i] > calibration_.maximum[i])
            {
                calibration_.maximum[i] = rawSample[i];
            }
        }
        calibration_.initialized = true;
    }

    /**
     * @brief Reset calibration data.
     */
    void resetCalibration() noexcept
    {
        calibration_.reset();
    }

    [[nodiscard]] bool isCalibrated() const noexcept { return calibration_.initialized; }
    [[nodiscard]] const CalibrationData<NumSensors>& getCalibration() const noexcept { return calibration_; }
    void setCalibration(const CalibrationData<NumSensors>& cal) noexcept { calibration_ = cal; }

    // =========================================================================
    // Raw & Calibrated Acquisition APIs
    // =========================================================================

    /**
     * @brief Read raw 12-bit ADC counts for all sensors.
     * Supports auto-differential ambient subtraction if enabled on custom board.
     */
    template<adc::Instance AdcInst>
    void readRaw(adc::AdcHandler<AdcInst>& adc, std::array<std::uint16_t, NumSensors>& rawOut) noexcept
    {
        if (hasEmitterPin_ && emitterPolicy_ == EmitterPolicy::AutoDifferential)
        {
            // 1. Read Ambient (Emitters OFF)
            emittersOff();
            systick::delayUs(100); // Phototransistor discharge settling
            std::array<std::uint16_t, NumSensors> ambient;
            for (std::size_t i = 0; i < NumSensors; ++i)
            {
                ambient[i] = adc.readRaw(pins_[i]);
            }

            // 2. Read Active (Emitters ON)
            emittersOn();
            systick::delayUs(100); // Emitter rise settling
            for (std::size_t i = 0; i < NumSensors; ++i)
            {
                const std::uint16_t active = adc.readRaw(pins_[i]);
                // Subtract ambient light baseline
                rawOut[i] = (active > ambient[i]) ? (active - ambient[i]) : 0;
            }
        }
        else
        {
            // Standard synchronous read (Emitters permanently on)
            for (std::size_t i = 0; i < NumSensors; ++i)
            {
                rawOut[i] = adc.readRaw(pins_[i]);
            }
        }
    }

    /**
     * @brief Normalizes raw ADC counts to standard [0 .. 1000] SI reflectance units.
     * Completely zeroes out channel-to-channel variations of cheap clone sensors.
     */
    void processCalibrated(const std::array<std::uint16_t, NumSensors>& raw,
                           std::array<std::uint16_t, NumSensors>& calOut) const noexcept
    {
        for (std::size_t i = 0; i < NumSensors; ++i)
        {
            std::uint16_t calValue = 0;

            if (calibration_.initialized)
            {
                const std::uint16_t minVal = calibration_.minimum[i];
                const std::uint16_t maxVal = calibration_.maximum[i];
                const std::uint16_t denominator = (maxVal > minVal) ? (maxVal - minVal) : 1;

                if (raw[i] <= minVal)
                {
                    calValue = 0;
                }
                else if (raw[i] >= maxVal)
                {
                    calValue = 1000;
                }
                else
                {
                    calValue = static_cast<std::uint16_t>((static_cast<std::uint32_t>(raw[i] - minVal) * 1000U) / denominator);
                }
            }
            else
            {
                // Fallback: 12-bit ADC mapping [0..4095] -> [0..1000]
                calValue = static_cast<std::uint16_t>((static_cast<std::uint32_t>(raw[i]) * 1000U) / 4095U);
            }

            // Invert for Light-on-Dark tracks
            if (lineColor_ == LineColor::LightOnDark)
            {
                calValue = 1000U - calValue;
            }

            calOut[i] = calValue;
        }
    }

    /**
     * @brief Read normalized [0 .. 1000] calibrated values across all channels.
     */
    template<adc::Instance AdcInst>
    void readCalibrated(adc::AdcHandler<AdcInst>& adc, std::array<std::uint16_t, NumSensors>& calOut) noexcept
    {
        std::array<std::uint16_t, NumSensors> raw;
        readRaw(adc, raw);
        processCalibrated(raw, calOut);
    }

    // =========================================================================
    // Line Centroid Calculation & Lost Line Memory
    // =========================================================================

    /**
     * @brief Estimate line position using weighted centroid with lost-line memory.
     *
     * Position formula:
     *   Position = Sum(i * 1000 * cal[i]) / Sum(cal[i])
     *
     * Return range:
     *   [0 .. (NumSensors - 1) * 1000]
     *   For 8 sensors: 0 (far left) to 7000 (far right), with 3500 being dead center.
     *
     * Lost-Line Memory:
     *   When the line is lost (total reflection < threshold), the sensor continues to
     *   return 0 (if lost on the left) or max (if lost on the right) so steering PID
     *   steers back onto the track instead of straightening into a wall.
     *
     * @tparam AdcInst STM32 ADC Instance
     * @param adc Reference to initialized AdcHandler
     * @param calOut Output array filled with normalized [0..1000] readings
     * @return Centroid line position
     */
    template<adc::Instance AdcInst>
    [[nodiscard]] std::int32_t readLine(adc::AdcHandler<AdcInst>& adc,
                                        std::array<std::uint16_t, NumSensors>& calOut) noexcept
    {
        readCalibrated(adc, calOut);
        return computePosition(calOut);
    }

    /**
     * @brief Zero-overhead line estimation directly from a circular DMA ADC buffer.
     * Ideal for custom modular boards running multi-channel ADC continuous DMA!
     *
     * @param dmaBuffer Pointer to array of raw ADC readings continuously updated by DMA
     * @param calOut Output array filled with normalized [0..1000] readings
     * @return Centroid line position
     */
    [[nodiscard]] std::int32_t readLineDma(const std::uint16_t* dmaBuffer,
                                          std::array<std::uint16_t, NumSensors>& calOut) noexcept
    {
        std::array<std::uint16_t, NumSensors> raw;
        for (std::size_t i = 0; i < NumSensors; ++i)
        {
            raw[i] = dmaBuffer[i];
        }
        processCalibrated(raw, calOut);
        return computePosition(calOut);
    }

    /**
     * @brief Simplified readLine overload without requiring a user buffer.
     */
    template<adc::Instance AdcInst>
    [[nodiscard]] std::int32_t readLine(adc::AdcHandler<AdcInst>& adc) noexcept
    {
        std::array<std::uint16_t, NumSensors> dummy;
        return readLine(adc, dummy);
    }

    /**
     * @brief Returns complete LineResult including position, total reflection, and detection flag.
     */
    template<adc::Instance AdcInst>
    [[nodiscard]] LineResult getLineResult(adc::AdcHandler<AdcInst>& adc) noexcept
    {
        std::array<std::uint16_t, NumSensors> cal;
        const std::int32_t pos = readLine(adc, cal);

        std::uint32_t total = 0;
        for (std::size_t i = 0; i < NumSensors; ++i)
        {
            total += cal[i];
        }

        return LineResult{pos, total, lastLineDetected_};
    }

    [[nodiscard]] bool isLineDetected() const noexcept { return lastLineDetected_; }
    [[nodiscard]] std::int32_t getLastPosition() const noexcept { return lastPosition_; }
    [[nodiscard]] constexpr std::int32_t getCenterPosition() const noexcept
    {
        return static_cast<std::int32_t>((NumSensors - 1) * 500);
    }

    void setLostLineThreshold(std::uint32_t threshold) noexcept { lostLineThreshold_ = threshold; }
    [[nodiscard]] std::uint32_t getLostLineThreshold() const noexcept { return lostLineThreshold_; }

    void setLineColor(LineColor color) noexcept { lineColor_ = color; }
    [[nodiscard]] LineColor getLineColor() const noexcept { return lineColor_; }

private:
    std::array<gpio::Pin, NumSensors> pins_;
    bool                              hasEmitterPin_;
    gpio::Pin                         emitterPin_;
    bool                              emitterActiveHigh_;
    LineColor                         lineColor_;
    EmitterPolicy                     emitterPolicy_;
    CalibrationData<NumSensors>       calibration_;
    std::int32_t                      lastPosition_;
    std::uint32_t                     lostLineThreshold_;
    bool                              lastLineDetected_;

    std::int32_t computePosition(const std::array<std::uint16_t, NumSensors>& cal) noexcept
    {
        std::uint32_t weightedSum = 0;
        std::uint32_t sum = 0;

        for (std::size_t i = 0; i < NumSensors; ++i)
        {
            const std::uint32_t val = cal[i];
            // Filter noise floor (ignore tiny readings < 50)
            if (val > 50)
            {
                weightedSum += (static_cast<std::uint32_t>(i) * 1000U) * val;
                sum += val;
            }
        }

        // Check if line is detected
        if (sum < lostLineThreshold_)
        {
            lastLineDetected_ = false;
            // Lost line memory: steer towards the side the line was last seen!
            if (lastPosition_ < getCenterPosition())
            {
                return 0; // Far left
            }
            else
            {
                return static_cast<std::int32_t>((NumSensors - 1) * 1000); // Far right
            }
        }

        lastLineDetected_ = true;
        lastPosition_ = static_cast<std::int32_t>(weightedSum / sum);
        return lastPosition_;
    }
};

} // namespace components::sensors::qtr
