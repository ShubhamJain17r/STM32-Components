/**
 * @file tb6612fng.hpp
 * @brief High-performance, zero-allocation driver for Toshiba TB6612FNG Dual H-Bridge Motor Driver.
 *
 * Supports:
 * - Direct bare-IC integration on custom PCBs (e.g. STBY tied directly to VCC with 0 wasted GPIO pins)
 * - Breakout module testing on Nucleo / development boards (STBY actively driven by STM32 GPIO)
 * - Single motor channel or coordinated dual-motor differential drive
 * - Forward, reverse, dynamic short-brake, and high-impedance coast
 * - Input deadband compensation and minimum overcoming-stiction duty cycle
 * - Ultrasonic PWM frequencies (e.g. 20 kHz) to eliminate audible motor whine
 */

#pragma once

#include "stm32f446xx.h"
#include <cstdint>

#include "stm32/gpio/gpio.hpp"
#include "stm32/timer/timer.hpp"

#include "components/common/component_base.hpp"
#include "components/common/math_utils.hpp"
#include "components/actuators/tb6612fng/tb6612fng_types.hpp"

namespace components::actuators::tb6612fng
{

/**
 * @brief Standby pin manager supporting both MCU-driven pins and PCB-hardware-tied VCC.
 */
class StandbyManager
{
public:
    constexpr explicit StandbyManager(PinConnection connection = PinConnection::HardwareTiedHigh) noexcept
        : connection_(connection), hasPin_(false), pin_{nullptr, 0xFF}, isStandby_(false)
    {}

    explicit StandbyManager(gpio::Pin stbyPin) noexcept
        : connection_(PinConnection::McuControlled), hasPin_(true), pin_(stbyPin), isStandby_(true)
    {}

    void init() noexcept
    {
        if (hasPin_ && connection_ == PinConnection::McuControlled)
        {
            pin_.enableClock();
            gpio::helper::setMode(pin_, gpio::Mode::OUTPUT);
            gpio::helper::setOutputType(pin_, gpio::OutputType::PUSH_PULL);
            gpio::helper::setOutputSpeed(pin_, gpio::OutputSpeed::LOW);
            gpio::helper::setPull(pin_, gpio::Pull::NONE);
            wake();
        }
    }

    void wake() noexcept
    {
        if (hasPin_ && connection_ == PinConnection::McuControlled)
        {
            gpio::DigitalOutput stby{pin_};
            stby.high();
        }
        isStandby_ = false;
    }

    void standby() noexcept
    {
        if (hasPin_ && connection_ == PinConnection::McuControlled)
        {
            gpio::DigitalOutput stby{pin_};
            stby.low();
        }
        isStandby_ = true;
    }

    [[nodiscard]] bool isStandby() const noexcept { return isStandby_; }
    [[nodiscard]] PinConnection connection() const noexcept { return connection_; }

private:
    PinConnection connection_;
    bool          hasPin_;
    gpio::Pin     pin_;
    bool          isStandby_;
};

/**
 * @brief Represents an individual H-bridge channel (Motor A or Motor B).
 *
 * @tparam TimInstance STM32 Timer instance used for PWM generation.
 */
template<timer::Instance TimInstance>
class MotorChannel
{
public:
    MotorChannel() = delete;
    ~MotorChannel() = default;

    MotorChannel(const MotorChannel&) = delete;
    MotorChannel& operator=(const MotorChannel&) = delete;

    MotorChannel(MotorChannel&&) = default;
    MotorChannel& operator=(MotorChannel&&) = default;

    /**
     * @brief Construct a single motor channel.
     *
     * @param in1Pin Direction control pin 1
     * @param in2Pin Direction control pin 2
     * @param pwm Reference to timer::PwmHandler
     * @param pwmChannel Timer PWM channel (e.g. Channel::CH1)
     * @param config Motor channel tuning configuration
     */
    MotorChannel(gpio::Pin in1Pin,
                 gpio::Pin in2Pin,
                 timer::PwmHandler<TimInstance>& pwm,
                 timer::Channel pwmChannel,
                 ChannelConfig config = {}) noexcept
        : in1Pin_(in1Pin),
          in2Pin_(in2Pin),
          pwm_(pwm),
          pwmChannel_(pwmChannel),
          config_(config),
          currentSpeed_(0.0f),
          currentDirection_(Direction::Standby)
    {}

    /**
     * @brief Initialize GPIO direction pins and optionally map the PWM pin.
     *
     * @param pwmPin Physical GPIO pin for PWM output.
     * @return true if initialization succeeded.
     */
    bool init(gpio::Pin pwmPin) noexcept
    {
        in1Pin_.enableClock();
        gpio::helper::setMode(in1Pin_, gpio::Mode::OUTPUT);
        gpio::helper::setOutputType(in1Pin_, gpio::OutputType::PUSH_PULL);
        gpio::helper::setOutputSpeed(in1Pin_, gpio::OutputSpeed::HIGH);
        gpio::helper::setPull(in1Pin_, gpio::Pull::NONE);

        in2Pin_.enableClock();
        gpio::helper::setMode(in2Pin_, gpio::Mode::OUTPUT);
        gpio::helper::setOutputType(in2Pin_, gpio::OutputType::PUSH_PULL);
        gpio::helper::setOutputSpeed(in2Pin_, gpio::OutputSpeed::HIGH);
        gpio::helper::setPull(in2Pin_, gpio::Pull::NONE);

        bool pwmOk = pwm_.enableChannel(pwmChannel_, pwmPin);
        stop();
        return pwmOk;
    }

    /**
     * @brief Set normalized motor speed in range [-1.0f, +1.0f].
     *
     * Positive values drive forward, negative values drive reverse,
     * and 0.0f triggers zeroSpeedMode (Brake or Coast).
     * Automatically applies deadband compensation and minimum duty threshold.
     */
    void setSpeed(float speed) noexcept
    {
        speed = math::clamp(speed, -1.0f, 1.0f);
        currentSpeed_ = speed;

        if (config_.inverted)
        {
            speed = -speed;
        }

        const float compensated = math::applyDeadband(speed, config_.deadband);

        gpio::DigitalOutput in1{in1Pin_};
        gpio::DigitalOutput in2{in2Pin_};

        if (compensated == 0.0f)
        {
            if (config_.zeroSpeedMode == ZeroSpeedMode::Brake)
            {
                in1.high();
                in2.high();
                currentDirection_ = Direction::Brake;
            }
            else
            {
                in1.low();
                in2.low();
                currentDirection_ = Direction::Coast;
            }
            pwm_.setDutyCycle(pwmChannel_, 0.0f);
        }
        else if (compensated > 0.0f)
        {
            in1.high();
            in2.low();
            currentDirection_ = Direction::Forward;

            const float duty = config_.minDuty + (1.0f - config_.minDuty) * compensated;
            pwm_.setDutyCycle(pwmChannel_, duty * 100.0f);
        }
        else // compensated < 0.0f
        {
            in1.low();
            in2.high();
            currentDirection_ = Direction::Reverse;

            const float duty = config_.minDuty + (1.0f - config_.minDuty) * (-compensated);
            pwm_.setDutyCycle(pwmChannel_, duty * 100.0f);
        }
    }

    /**
     * @brief Drive motor forward at specified speed magnitude in [0.0f, 1.0f].
     */
    void forward(float speed = 1.0f) noexcept
    {
        setSpeed(math::abs(speed));
    }

    /**
     * @brief Drive motor in reverse at specified speed magnitude in [0.0f, 1.0f].
     */
    void reverse(float speed = 1.0f) noexcept
    {
        setSpeed(-math::abs(speed));
    }

    /**
     * @brief Active short-brake (shorts both motor leads to ground).
     */
    void brake() noexcept
    {
        gpio::DigitalOutput in1{in1Pin_};
        gpio::DigitalOutput in2{in2Pin_};
        in1.high();
        in2.high();
        pwm_.setDutyCycle(pwmChannel_, 0.0f);
        currentSpeed_ = 0.0f;
        currentDirection_ = Direction::Brake;
    }

    /**
     * @brief Passive coast (leaves both motor leads floating/High-Z).
     */
    void coast() noexcept
    {
        gpio::DigitalOutput in1{in1Pin_};
        gpio::DigitalOutput in2{in2Pin_};
        in1.low();
        in2.low();
        pwm_.setDutyCycle(pwmChannel_, 0.0f);
        currentSpeed_ = 0.0f;
        currentDirection_ = Direction::Coast;
    }

    /**
     * @brief Alias for stop based on configured zero-speed policy.
     */
    void stop() noexcept
    {
        if (config_.zeroSpeedMode == ZeroSpeedMode::Brake)
        {
            brake();
        }
        else
        {
            coast();
        }
    }

    void setInverted(bool inverted) noexcept { config_.inverted = inverted; }
    void setDeadband(float deadband) noexcept { config_.deadband = math::clamp(deadband, 0.0f, 1.0f); }
    void setMinDuty(float minDuty) noexcept { config_.minDuty = math::clamp(minDuty, 0.0f, 1.0f); }
    void setZeroSpeedMode(ZeroSpeedMode mode) noexcept { config_.zeroSpeedMode = mode; }

    [[nodiscard]] float getSpeed() const noexcept { return currentSpeed_; }
    [[nodiscard]] Direction getDirection() const noexcept { return currentDirection_; }
    [[nodiscard]] const ChannelConfig& getConfig() const noexcept { return config_; }

private:
    gpio::Pin                       in1Pin_;
    gpio::Pin                       in2Pin_;
    timer::PwmHandler<TimInstance>& pwm_;
    timer::Channel                  pwmChannel_;
    ChannelConfig                   config_;
    float                           currentSpeed_;
    Direction                       currentDirection_;
};

/**
 * @brief Dual-channel TB6612FNG controller.
 *
 * Supports dual motors driven by the same or different hardware timers,
 * with either an MCU-driven STBY pin (breakout module) or hardware-tied STBY (bare IC on PCB).
 *
 * @tparam TimA Timer instance for Motor A PWM
 * @tparam TimB Timer instance for Motor B PWM (defaults to TimA if both share one timer)
 */
template<timer::Instance TimA, timer::Instance TimB = TimA>
class Tb6612Fng
{
public:
    /**
     * @brief Breakout Module Constructor: with MCU-controlled STBY pin.
     */
    Tb6612Fng(gpio::Pin inA1, gpio::Pin inA2, timer::PwmHandler<TimA>& pwmA, timer::Channel chA,
              gpio::Pin inB1, gpio::Pin inB2, timer::PwmHandler<TimB>& pwmB, timer::Channel chB,
              gpio::Pin stbyPin,
              ChannelConfig configA = {}, ChannelConfig configB = {}) noexcept
        : motorA_(inA1, inA2, pwmA, chA, configA),
          motorB_(inB1, inB2, pwmB, chB, configB),
          standby_(stbyPin)
    {}

    /**
     * @brief Bare-IC / PCB Constructor: STBY is hardwired to VCC on the PCB (0 MCU pins needed).
     */
    Tb6612Fng(gpio::Pin inA1, gpio::Pin inA2, timer::PwmHandler<TimA>& pwmA, timer::Channel chA,
              gpio::Pin inB1, gpio::Pin inB2, timer::PwmHandler<TimB>& pwmB, timer::Channel chB,
              PinConnection stbyConnection = PinConnection::HardwareTiedHigh,
              ChannelConfig configA = {}, ChannelConfig configB = {}) noexcept
        : motorA_(inA1, inA2, pwmA, chA, configA),
          motorB_(inB1, inB2, pwmB, chB, configB),
          standby_(stbyConnection)
    {}

    /**
     * @brief Initialize all direction GPIOs, PWM channels, and standby pin.
     *
     * @param pwmPinA Physical STM32 pin for Motor A PWM
     * @param pwmPinB Physical STM32 pin for Motor B PWM
     * @return true if both PWM channels initialized successfully.
     */
    bool init(gpio::Pin pwmPinA, gpio::Pin pwmPinB) noexcept
    {
        standby_.init();
        bool okA = motorA_.init(pwmPinA);
        bool okB = motorB_.init(pwmPinB);
        return okA && okB;
    }

    /**
     * @brief Set individual speeds for Motor A and Motor B in range [-1.0f, +1.0f].
     */
    void setSpeeds(float speedA, float speedB) noexcept
    {
        if (standby_.isStandby())
        {
            standby_.wake();
        }
        motorA_.setSpeed(speedA);
        motorB_.setSpeed(speedB);
    }

    /**
     * @brief Coordinated differential drive steering mixer.
     *
     * @param throttle Forward/reverse throttle in [-1.0f, +1.0f]
     * @param steering Left/right steering in [-1.0f, +1.0f] (positive turns right)
     */
    void drive(float throttle, float steering) noexcept
    {
        const float speedA = math::clamp(throttle + steering, -1.0f, 1.0f);
        const float speedB = math::clamp(throttle - steering, -1.0f, 1.0f);
        setSpeeds(speedA, speedB);
    }

    /**
     * @brief Actively short-brake both motors.
     */
    void brake() noexcept
    {
        motorA_.brake();
        motorB_.brake();
    }

    /**
     * @brief Freewheel coast both motors.
     */
    void coast() noexcept
    {
        motorA_.coast();
        motorB_.coast();
    }

    /**
     * @brief Enter power-saving standby mode.
     * If STBY is MCU-controlled, pulls STBY low. Also stops both motor outputs.
     */
    void standby() noexcept
    {
        motorA_.coast();
        motorB_.coast();
        standby_.standby();
    }

    /**
     * @brief Exit standby mode.
     */
    void wake() noexcept
    {
        standby_.wake();
    }

    [[nodiscard]] bool isStandby() const noexcept { return standby_.isStandby(); }

    // Direct access to individual motor channels
    [[nodiscard]] MotorChannel<TimA>& a() noexcept { return motorA_; }
    [[nodiscard]] MotorChannel<TimB>& b() noexcept { return motorB_; }
    [[nodiscard]] const MotorChannel<TimA>& a() const noexcept { return motorA_; }
    [[nodiscard]] const MotorChannel<TimB>& b() const noexcept { return motorB_; }

private:
    MotorChannel<TimA> motorA_;
    MotorChannel<TimB> motorB_;
    StandbyManager     standby_;
};

} // namespace components::actuators::tb6612fng
