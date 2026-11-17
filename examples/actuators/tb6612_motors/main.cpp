/**
 * @file main.cpp
 * @brief TB6612FNG Dual DC Motor Driver Example for STM32F446RE Nucleo.
 *
 * Demonstrates:
 * 1. 20 kHz ultrasonic PWM generation using TIM2 (eliminates audible motor whine).
 * 2. Breakout module mode (with STBY pin on PC4).
 * 3. Bare-IC on custom PCB mode (STBY hardwired to VCC with 0 wasted GPIOs).
 * 4. Deadband compensation, acceleration ramping, differential steering, and dynamic short-braking.
 */

#include "stm32/common/systick/systick.hpp"
#include "stm32/gpio/gpio.hpp"
#include "stm32/timer/timer.hpp"

#include "components/actuators/tb6612fng/tb6612fng.hpp"

using namespace components::actuators::tb6612fng;

// Motor Pin Definitions on STM32F446RE Nucleo
// Motor A:
constexpr gpio::Pin PIN_PWMA = gpio::PA0; // TIM2_CH1 (AF1)
constexpr gpio::Pin PIN_AIN1 = gpio::PC0;
constexpr gpio::Pin PIN_AIN2 = gpio::PC1;

// Motor B:
constexpr gpio::Pin PIN_PWMB = gpio::PA1; // TIM2_CH2 (AF1)
constexpr gpio::Pin PIN_BIN1 = gpio::PC2;
constexpr gpio::Pin PIN_BIN2 = gpio::PC3;

// Standby Pin (Used when testing breakout module with STBY pin connected)
constexpr gpio::Pin PIN_STBY = gpio::PC4;

int main()
{
    // 1. Initialize SysTick timebase for millisecond delays
    systick::init(1000);

    // 2. Initialize onboard LED (LD2 on Nucleo-F446RE = PA5) for status heartbeat
    gpio::DigitalOutput statusLed{gpio::PA5};

    // 3. Initialize 20 kHz PWM generator on TIM2 (Ultrasonic frequency)
    timer::PwmHandler<timer::Instance::tim2> motorPwm{20'000};
    motorPwm.start();

    // 4. Configure motor channels with deadband and stiction compensation
    ChannelConfig leftConfig;
    leftConfig.inverted = false;
    leftConfig.deadband = 0.05f;   // Ignore tiny jitter near 0
    leftConfig.minDuty  = 0.12f;   // Overcome gearbox static friction at low speeds

    ChannelConfig rightConfig;
    rightConfig.inverted = false;
    rightConfig.deadband = 0.05f;
    rightConfig.minDuty  = 0.12f;

    // -------------------------------------------------------------------------
    // CASE A: BREAKOUT MODULE (e.g. SparkFun/Pololu module tested with Nucleo)
    // The STBY pin is actively controlled by STM32 GPIO (PIN_STBY).
    // -------------------------------------------------------------------------
    Tb6612Fng<timer::Instance::tim2> motorsBreakout(
        PIN_AIN1, PIN_AIN2, motorPwm, timer::Channel::CH1,
        PIN_BIN1, PIN_BIN2, motorPwm, timer::Channel::CH2,
        PIN_STBY,
        leftConfig, rightConfig
    );

    // Initialize motor driver pins and PWM channels
    motorsBreakout.init(PIN_PWMA, PIN_PWMB);

    // -------------------------------------------------------------------------
    // CASE B: BARE IC ON CUSTOM PCB (STBY hardwired to 3.3V on the PCB trace)
    // Zero STM32 GPIO pins wasted on STBY!
    // -------------------------------------------------------------------------
    [[maybe_unused]] Tb6612Fng<timer::Instance::tim2> motorsCustomPcb(
        PIN_AIN1, PIN_AIN2, motorPwm, timer::Channel::CH1,
        PIN_BIN1, PIN_BIN2, motorPwm, timer::Channel::CH2,
        components::PinConnection::HardwareTiedHigh,
        leftConfig, rightConfig
    );

    // Demonstration Sequence
    while (true)
    {
        // --- State 1: Ramp forward acceleration ---
        statusLed.high();
        for (float speed = 0.0f; speed <= 1.0f; speed += 0.05f)
        {
            motorsBreakout.setSpeeds(speed, speed);
            systick::delayMs(50);
        }

        systick::delayMs(500);

        // --- State 2: Active Dynamic Short-Brake ---
        statusLed.low();
        motorsBreakout.brake();
        systick::delayMs(500);

        // --- State 3: Differential Steering / Tank Turn (Right Turn) ---
        statusLed.high();
        motorsBreakout.drive(0.6f, 0.4f); // Throttle = 0.6, Steering = 0.4
        systick::delayMs(1000);

        // --- State 4: Reverse at 50% speed ---
        statusLed.low();
        motorsBreakout.setSpeeds(-0.5f, -0.5f);
        systick::delayMs(1000);

        // --- State 5: Passive Coast (Freewheel) & Standby ---
        motorsBreakout.coast();
        motorsBreakout.standby();
        systick::delayMs(1000);

        // Wake from standby for next loop cycle
        motorsBreakout.wake();
    }

    return 0;
}
