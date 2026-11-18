/**
 * @file encoder.hpp
 * @brief Production-grade C++17 Quadrature Encoder driver using STM32 Hardware Timer Encoder Mode.
 *
 * Utilizes the dedicated STM32 hardware timer encoder interface (TIM_SMCR_SMS_ENCODER3):
 * - Decodes 4x quadrature counts directly in silicon with 0% CPU overhead
 * - Built-in digital noise filtering for noisy DC motor brushes
 * - 32-bit overflow-protected tick accumulation
 * - Velocity (RPM and m/s) and distance calculation
 */

#pragma once

#include "stm32f446xx.h"
#include <cstdint>
#include <cmath>

#include "stm32/common/registers/registers.hpp"
#include "stm32/common/rcc/rcc.hpp"
#include "stm32/common/rcc/rcc_enable.hpp"
#include "stm32/gpio/gpio.hpp"
#include "stm32/timer/timer.hpp"

#include "components/common/component_base.hpp"
#include "components/actuators/encoder/encoder_types.hpp"

namespace components::actuators::encoder
{

/**
 * @brief Hardware Timer Quadrature Encoder Driver.
 *
 * @tparam TimInstance STM32 Timer instance (TIM1, TIM2, TIM3, TIM4, TIM5, TIM8)
 */
template<timer::Instance TimInstance>
class QuadratureEncoder
{
    static_assert(!timer::Traits<TimInstance>::isBasic,
                  "Basic timers (TIM6/TIM7) do not support hardware encoder interface");

public:
    QuadratureEncoder() = delete;
    ~QuadratureEncoder() = default;

    QuadratureEncoder(const QuadratureEncoder&) = delete;
    QuadratureEncoder& operator=(const QuadratureEncoder&) = delete;

    QuadratureEncoder(QuadratureEncoder&&) = default;
    QuadratureEncoder& operator=(QuadratureEncoder&&) = default;

    /**
     * @brief Construct a quadrature encoder on specified timer pins.
     *
     * @param pinA Encoder Channel A pin (connects to Timer CH1)
     * @param pinB Encoder Channel B pin (connects to Timer CH2)
     * @param config Encoder resolution and physical dimensions
     */
    QuadratureEncoder(gpio::Pin pinA, gpio::Pin pinB, const EncoderConfig& config = {}) noexcept
        : pinA_(pinA),
          pinB_(pinB),
          config_(config),
          totalTicks_(0),
          lastHwCount_(0),
          lastUpdateTimeMs_(0),
          velocityRpm_(0.0f),
          velocityMps_(0.0f),
          initialized_(false)
    {}

    /**
     * @brief Initialize hardware GPIO pins and configure Timer in Quadrature Encoder mode.
     *
     * @return true if hardware setup succeeded.
     */
    bool init() noexcept
    {
        // 1. Resolve and configure Alternate Functions for Channel 1 and Channel 2
        const auto af1 = timer::af::alternateFunction<TimInstance, timer::Channel::CH1>(pinA_);
        const auto af2 = timer::af::alternateFunction<TimInstance, timer::Channel::CH2>(pinB_);

        if (af1 == gpio::af::AlternateFunction::INVALID || af2 == gpio::af::AlternateFunction::INVALID)
        {
            return false;
        }

        // Configure Pin A (CH1)
        pinA_.enableClock();
        gpio::af::setAlternateFunction(pinA_, af1);
        gpio::helper::setMode(pinA_, gpio::Mode::ALTERNATE);
        gpio::helper::setOutputSpeed(pinA_, gpio::OutputSpeed::HIGH);
        gpio::helper::setPull(pinA_, gpio::Pull::UP);

        // Configure Pin B (CH2)
        pinB_.enableClock();
        gpio::af::setAlternateFunction(pinB_, af2);
        gpio::helper::setMode(pinB_, gpio::Mode::ALTERNATE);
        gpio::helper::setOutputSpeed(pinB_, gpio::OutputSpeed::HIGH);
        gpio::helper::setPull(pinB_, gpio::Pull::UP);

        // 2. Enable Timer Peripheral Clock
        rcc::enablePeripheralClock(timer::Traits<TimInstance>::peripheral());

        auto* tim = timer::Traits<TimInstance>::peripheral();

        // 3. Disable timer counter before register configuration
        reg::clearBits(tim->CR1, TIM_CR1_CEN);

        // 4. Configure Prescaler to 0 (count every quadrature edge)
        reg::write(tim->PSC, 0U);

        // 5. Configure Auto-Reload to full scale
        constexpr std::uint32_t maxArr = timer::Traits<TimInstance>::is32Bit ? 0xFFFFFFFFU : 0xFFFFU;
        reg::write(tim->ARR, maxArr);

        // 6. Map Channel 1 and 2 to TI1 and TI2 with digital noise filtering
        // CC1S = 01 (mapped on TI1), IC1F = inputFilter
        // CC2S = 01 (mapped on TI2), IC2F = inputFilter
        const std::uint32_t filter = static_cast<std::uint32_t>(config_.inputFilter & 0x0FU);
        const std::uint32_t ccmr1 = (TIM_CCMR1_CC1S_0 | (filter << TIM_CCMR1_IC1F_Pos)) |
                                    (TIM_CCMR1_CC2S_0 | (filter << TIM_CCMR1_IC2F_Pos));
        reg::write(tim->CCMR1, ccmr1);

        // 7. Configure Input Polarity: Non-inverted on both channels
        reg::clearBits(tim->CCER, TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP);

        // 8. Configure Slave Mode Control Register (SMCR) for Encoder Mode
        // SMS[2:0] = 011 for Encoder Mode 3 (X4: counts on both TI1 and TI2 edges)
        reg::modifyBits(tim->SMCR, TIM_SMCR_SMS, static_cast<std::uint32_t>(config_.decodingMode));

        // 9. Reset counter and enable
        reg::write(tim->CNT, 0U);
        reg::setBits(tim->CR1, TIM_CR1_CEN);

        lastHwCount_ = 0;
        totalTicks_ = 0;
        initialized_ = true;

        return true;
    }

    /**
     * @brief Poll accumulated ticks and update velocity estimation.
     * Call this periodically in your control loop (e.g. every 5ms to 20ms).
     *
     * @param nowMs Current monotonic millisecond timestamp (from systick::millis())
     */
    void update(std::uint32_t nowMs) noexcept
    {
        auto* tim = timer::Traits<TimInstance>::peripheral();
        const std::uint32_t currentHw = reg::read(tim->CNT);

        // Delta calculation with seamless 16-bit wrap-around handling
        std::int32_t delta = 0;
        if constexpr (timer::Traits<TimInstance>::is32Bit)
        {
            delta = static_cast<std::int32_t>(currentHw - lastHwCount_);
        }
        else
        {
            delta = static_cast<std::int32_t>(static_cast<std::int16_t>(currentHw - lastHwCount_));
        }

        if (config_.inverted)
        {
            delta = -delta;
        }

        totalTicks_ += delta;
        lastHwCount_ = currentHw;

        // Velocity computation
        if (lastUpdateTimeMs_ > 0 && nowMs > lastUpdateTimeMs_)
        {
            const float dtSec = static_cast<float>(nowMs - lastUpdateTimeMs_) / 1000.0f;
            if (dtSec > 0.0001f && config_.countsPerRevolution > 0)
            {
                const float revs = static_cast<float>(delta) / static_cast<float>(config_.countsPerRevolution);
                velocityRpm_ = (revs / dtSec) * 60.0f;

                constexpr float PI = 3.141592653589793f;
                velocityMps_ = (velocityRpm_ * (PI * config_.wheelDiameterMeters)) / 60.0f;
            }
        }

        lastUpdateTimeMs_ = nowMs;
    }

    /**
     * @brief Returns total accumulated ticks since initialization or last reset.
     */
    [[nodiscard]] std::int64_t getTicks() const noexcept
    {
        return totalTicks_;
    }

    /**
     * @brief Reset tick accumulator to zero.
     */
    void resetTicks() noexcept
    {
        auto* tim = timer::Traits<TimInstance>::peripheral();
        reg::write(tim->CNT, 0U);
        lastHwCount_ = 0;
        totalTicks_ = 0;
        velocityRpm_ = 0.0f;
        velocityMps_ = 0.0f;
    }

    /**
     * @brief Returns total wheel revolutions.
     */
    [[nodiscard]] float getRevolutions() const noexcept
    {
        if (config_.countsPerRevolution == 0) return 0.0f;
        return static_cast<float>(totalTicks_) / static_cast<float>(config_.countsPerRevolution);
    }

    /**
     * @brief Returns total linear distance traveled in meters.
     */
    [[nodiscard]] float getDistanceMeters() const noexcept
    {
        constexpr float PI = 3.141592653589793f;
        return getRevolutions() * (PI * config_.wheelDiameterMeters);
    }

    /**
     * @brief Returns estimated wheel rotational velocity in RPM.
     */
    [[nodiscard]] float getRpm() const noexcept
    {
        return velocityRpm_;
    }

    /**
     * @brief Returns estimated linear velocity in meters per second (m/s).
     */
    [[nodiscard]] float getLinearVelocity() const noexcept
    {
        return velocityMps_;
    }

    /**
     * @brief Returns direct hardware counter value (CNT register).
     */
    [[nodiscard]] std::uint32_t getRawCounter() const noexcept
    {
        return reg::read(timer::Traits<TimInstance>::peripheral()->CNT);
    }

    /**
     * @brief Checks if counter is incrementing (forward rotation).
     */
    [[nodiscard]] bool isForward() const noexcept
    {
        // On STM32 timers in encoder mode, TIM_CR1_DIR bit indicates counting direction:
        // 0 = counting up (forward), 1 = counting down (reverse)
        const bool hwDir = reg::isAnyBitSet(timer::Traits<TimInstance>::peripheral()->CR1, TIM_CR1_DIR);
        return config_.inverted ? hwDir : !hwDir;
    }

    void setInverted(bool inverted) noexcept { config_.inverted = inverted; }
    void setCountsPerRevolution(std::uint32_t cpr) noexcept { config_.countsPerRevolution = cpr; }
    void setWheelDiameterMeters(float diameter) noexcept { config_.wheelDiameterMeters = diameter; }

    [[nodiscard]] const EncoderConfig& getConfig() const noexcept { return config_; }
    [[nodiscard]] bool isInitialized() const noexcept { return initialized_; }

private:
    gpio::Pin     pinA_;
    gpio::Pin     pinB_;
    EncoderConfig config_;
    std::int64_t  totalTicks_;
    std::uint32_t lastHwCount_;
    std::uint32_t lastUpdateTimeMs_;
    float         velocityRpm_;
    float         velocityMps_;
    bool          initialized_;
};

} // namespace components::actuators::encoder
