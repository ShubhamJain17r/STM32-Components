/**
 * @file buzzer.hpp
 * @brief Production-grade C++17 driver for Active and Passive buzzers.
 *
 * Supports:
 * - Active Buzzers: GPIO-driven with non-blocking alert patterns and pulse trains (battery alarms)
 * - Passive Buzzers: PWM frequency tone generation, note sequencing, and melodies
 * - Non-blocking SysTick-based state machine updates (zero CPU stalling)
 */

#pragma once

#include "stm32f446xx.h"
#include <cstdint>
#include <cstddef>

#include "stm32/gpio/gpio.hpp"
#include "stm32/timer/timer.hpp"
#include "stm32/common/systick/systick.hpp"

#include "components/common/component_base.hpp"
#include "components/actuators/buzzer/buzzer_types.hpp"
#include "components/actuators/buzzer/notes.hpp"

namespace components::actuators::buzzer
{

/**
 * @brief Active Buzzer Controller (driven via GPIO logic high/low).
 */
class ActiveBuzzer
{
public:
    explicit ActiveBuzzer(gpio::Pin pin, bool activeHigh = true) noexcept
        : pin_(pin),
          activeHigh_(activeHigh),
          isPlaying_(false),
          stateOn_(false),
          pattern_{0, 0, 0},
          beepsRemaining_(0),
          lastTransitionTimeMs_(0)
    {}

    void init() noexcept
    {
        pin_.enableClock();
        gpio::helper::setMode(pin_, gpio::Mode::OUTPUT);
        gpio::helper::setOutputType(pin_, gpio::OutputType::PUSH_PULL);
        gpio::helper::setOutputSpeed(pin_, gpio::OutputSpeed::LOW);
        gpio::helper::setPull(pin_, gpio::Pull::NONE);
        off();
    }

    void on() noexcept
    {
        gpio::DigitalOutput out{pin_};
        out.write(activeHigh_ ? gpio::PinState::HIGH : gpio::PinState::LOW);
        stateOn_ = true;
    }

    void off() noexcept
    {
        gpio::DigitalOutput out{pin_};
        out.write(activeHigh_ ? gpio::PinState::LOW : gpio::PinState::HIGH);
        stateOn_ = false;
        isPlaying_ = false;
    }

    /**
     * @brief Sound a single blocking beep.
     */
    void beepBlocking(std::uint16_t durationMs) noexcept
    {
        on();
        systick::delayMs(durationMs);
        off();
    }

    /**
     * @brief Trigger a non-blocking repeating beep pattern.
     *
     * @param onMs Sound duration in milliseconds
     * @param offMs Silence duration in milliseconds
     * @param count Number of beeps (0 = repeat indefinitely until stopped)
     */
    void playPattern(std::uint16_t onMs, std::uint16_t offMs, std::uint8_t count = 1) noexcept
    {
        pattern_ = BeepPattern{onMs, offMs, count};
        beepsRemaining_ = count;
        lastTransitionTimeMs_ = systick::millis();
        on();
        isPlaying_ = true;
    }

    /**
     * @brief Periodic state machine update for non-blocking beep patterns.
     * Call this in your main loop.
     *
     * @param nowMs Monotonic millisecond timestamp from systick::millis()
     */
    void update(std::uint32_t nowMs) noexcept
    {
        if (!isPlaying_)
        {
            return;
        }

        const std::uint32_t elapsed = nowMs - lastTransitionTimeMs_;

        if (stateOn_)
        {
            if (elapsed >= pattern_.onMs)
            {
                // Transition to OFF state
                gpio::DigitalOutput out{pin_};
                out.write(activeHigh_ ? gpio::PinState::LOW : gpio::PinState::HIGH);
                stateOn_ = false;
                lastTransitionTimeMs_ = nowMs;

                if (pattern_.repeatCount > 0)
                {
                    if (--beepsRemaining_ == 0)
                    {
                        isPlaying_ = false;
                    }
                }
            }
        }
        else
        {
            if (elapsed >= pattern_.offMs)
            {
                // Transition to ON state
                on();
                lastTransitionTimeMs_ = nowMs;
            }
        }
    }

    [[nodiscard]] bool isPlaying() const noexcept { return isPlaying_; }

private:
    gpio::Pin     pin_;
    bool          activeHigh_;
    bool          isPlaying_;
    bool          stateOn_;
    BeepPattern   pattern_;
    std::uint8_t  beepsRemaining_;
    std::uint32_t lastTransitionTimeMs_;
};

/**
 * @brief Passive Piezoelectric Buzzer Controller (driven via Timer PWM frequency).
 *
 * @tparam TimInstance STM32 Timer instance used for frequency synthesis
 */
template<timer::Instance TimInstance>
class PassiveBuzzer
{
public:
    PassiveBuzzer(timer::PwmHandler<TimInstance>& pwm, timer::Channel ch, gpio::Pin pin) noexcept
        : pwm_(pwm),
          channel_(ch),
          pin_(pin),
          melody_(nullptr),
          melodyLength_(0),
          currentNoteIndex_(0),
          isPlayingMelody_(false),
          isNoteSounding_(false),
          lastNoteStartTimeMs_(0)
    {}

    bool init() noexcept
    {
        bool ok = pwm_.enableChannel(channel_, pin_);
        stop();
        return ok;
    }

    /**
     * @brief Play a continuous frequency tone.
     *
     * @param frequencyHz Frequency in Hz
     */
    void tone(std::uint16_t frequencyHz) noexcept
    {
        if (frequencyHz == 0)
        {
            stop();
            return;
        }

        pwm_.setFrequency(frequencyHz);
        pwm_.setDutyCycle(channel_, 50.0f); // 50% square wave for maximum resonance
        pwm_.start();
    }

    /**
     * @brief Silence the passive buzzer.
     */
    void stop() noexcept
    {
        pwm_.setDutyCycle(channel_, 0.0f);
        isPlayingMelody_ = false;
        isNoteSounding_ = false;
    }

    /**
     * @brief Play a single tone blocking for durationMs.
     */
    void toneBlocking(std::uint16_t frequencyHz, std::uint16_t durationMs) noexcept
    {
        tone(frequencyHz);
        systick::delayMs(durationMs);
        stop();
    }

    /**
     * @brief Start non-blocking melody playback.
     *
     * @param melody Pointer to array of Note structs
     * @param length Number of notes in the melody
     */
    void playMelody(const Note* melody, std::size_t length) noexcept
    {
        if (!melody || length == 0) return;

        melody_ = melody;
        melodyLength_ = length;
        currentNoteIndex_ = 0;
        isPlayingMelody_ = true;
        isNoteSounding_ = false;
        lastNoteStartTimeMs_ = systick::millis();

        playCurrentNote();
    }

    /**
     * @brief Periodic melody sequencer update. Call in main loop.
     */
    void update(std::uint32_t nowMs) noexcept
    {
        if (!isPlayingMelody_ || !melody_)
        {
            return;
        }

        const auto& note = melody_[currentNoteIndex_];
        const std::uint32_t elapsed = nowMs - lastNoteStartTimeMs_;

        if (isNoteSounding_)
        {
            // Note articulation: 90% sounding duration, 10% pause between notes
            const std::uint32_t soundDuration = (static_cast<std::uint32_t>(note.durationMs) * 9U) / 10U;
            if (elapsed >= soundDuration)
            {
                pwm_.setDutyCycle(channel_, 0.0f);
                isNoteSounding_ = false;
            }
        }
        else
        {
            if (elapsed >= note.durationMs)
            {
                // Advance to next note
                currentNoteIndex_++;
                if (currentNoteIndex_ >= melodyLength_)
                {
                    stop();
                }
                else
                {
                    lastNoteStartTimeMs_ = nowMs;
                    playCurrentNote();
                }
            }
        }
    }

    [[nodiscard]] bool isPlaying() const noexcept { return isPlayingMelody_; }

private:
    timer::PwmHandler<TimInstance>& pwm_;
    timer::Channel                  channel_;
    gpio::Pin                       pin_;
    const Note*                     melody_;
    std::size_t                     melodyLength_;
    std::size_t                     currentNoteIndex_;
    bool                            isPlayingMelody_;
    bool                            isNoteSounding_;
    std::uint32_t                   lastNoteStartTimeMs_;

    void playCurrentNote() noexcept
    {
        const auto& note = melody_[currentNoteIndex_];
        if (note.frequency > 0)
        {
            pwm_.setFrequency(note.frequency);
            pwm_.setDutyCycle(channel_, 50.0f);
            pwm_.start();
            isNoteSounding_ = true;
        }
        else
        {
            pwm_.setDutyCycle(channel_, 0.0f);
            isNoteSounding_ = false;
        }
    }
};

} // namespace components::actuators::buzzer
