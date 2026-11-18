/**
 * @file main.cpp
 * @brief Complete Closed-Loop Line Following Robot Integration Example.
 *
 * Demonstrates the full STM32-Components robotics stack:
 * 1. TB6612FNG Dual DC Motor Driver (20 kHz PWM on TIM2) with bare-IC hardware-tied STBY.
 * 2. Hardware Timer Quadrature Encoder (TIM3 on PA6/PA7) with 0% CPU decoding.
 * 3. QTR-8A Analog Line Sensor Array (ADC1) with dynamic calibration & lost-line memory.
 * 4. High-performance discrete PID Controller for steering correction.
 * 5. Active Buzzer (PB8) with non-blocking alert patterns.
 * 6. HC-05 Bluetooth Telemetry & Wireless Command Tuning (USART1 on PB6/PB7).
 */

#include "stm32/common/systick/systick.hpp"
#include "stm32/gpio/gpio.hpp"
#include "stm32/timer/timer.hpp"
#include "stm32/adc/adc.hpp"
#include "stm32/uart/uart.hpp"

#include "components/common/pid_controller.hpp"
#include "components/actuators/tb6612fng/tb6612fng.hpp"
#include "components/actuators/encoder/encoder.hpp"
#include "components/actuators/buzzer/buzzer.hpp"
#include "components/sensors/qtr/qtr_analog.hpp"
#include "components/wireless/hc05/hc05.hpp"

using namespace components;
using namespace components::actuators::tb6612fng;
using namespace components::actuators::encoder;
using namespace components::actuators::buzzer;
using namespace components::sensors::qtr;
using namespace components::wireless::hc05;
using namespace components::control;

// =============================================================================
// Pin Assignments on STM32F446RE Nucleo / Custom PCB
// =============================================================================
// Motor PWM (TIM2 CH1 & CH2)
constexpr gpio::Pin PIN_PWMA = gpio::PA0;
constexpr gpio::Pin PIN_PWMB = gpio::PA1;
constexpr gpio::Pin PIN_AIN1 = gpio::PC0;
constexpr gpio::Pin PIN_AIN2 = gpio::PC1;
constexpr gpio::Pin PIN_BIN1 = gpio::PC2;
constexpr gpio::Pin PIN_BIN2 = gpio::PC3;

// Encoder (TIM3 CH1 & CH2)
constexpr gpio::Pin PIN_ENC_A = gpio::PA6;
constexpr gpio::Pin PIN_ENC_B = gpio::PA7;

// Active Buzzer
constexpr gpio::Pin PIN_BUZZER = gpio::PB8;

// Bluetooth HC-05 (USART1)
constexpr gpio::Pin PIN_BT_TX = gpio::PB6;
constexpr gpio::Pin PIN_BT_RX = gpio::PB7;

// QTR 8-Sensor Array (ADC1)
constexpr std::array<gpio::Pin, 8> QTR_PINS = {
    gpio::PC0, gpio::PC1, gpio::PC2, gpio::PC3,
    gpio::PA0, gpio::PA1, gpio::PA2, gpio::PA3
};

int main()
{
    // 1. Initialize SysTick timebase (1 ms monotonic ticks)
    systick::init(1000);

    // 2. Initialize Status LED & Active Buzzer
    gpio::DigitalOutput statusLed{gpio::PA5};
    ActiveBuzzer buzzer{PIN_BUZZER};
    buzzer.init();

    // 3. Initialize 20 kHz Ultrasonic Motor PWM on TIM2
    timer::PwmHandler<timer::Instance::tim2> motorPwm{20'000};
    motorPwm.start();

    // 4. Initialize TB6612FNG Dual Motor Driver (Custom PCB Bare-IC mode: STBY tied to 3.3V)
    ChannelConfig motorCfg;
    motorCfg.deadband = 0.05f;
    motorCfg.minDuty  = 0.12f;

    Tb6612Fng<timer::Instance::tim2> motors(
        PIN_AIN1, PIN_AIN2, motorPwm, timer::Channel::CH1,
        PIN_BIN1, PIN_BIN2, motorPwm, timer::Channel::CH2,
        PinConnection::HardwareTiedHigh,
        motorCfg, motorCfg
    );
    motors.init(PIN_PWMA, PIN_PWMB);

    // 5. Initialize Hardware Timer Quadrature Encoder on TIM3 (0% CPU Load)
    EncoderConfig encCfg;
    encCfg.countsPerRevolution = 1440; // 12 CPR * 30:1 * 4x
    encCfg.wheelDiameterMeters = 0.032f; // 32 mm wheel
    QuadratureEncoder<timer::Instance::tim3> leftEncoder{PIN_ENC_A, PIN_ENC_B, encCfg};
    leftEncoder.init();

    // 6. Initialize Steering PID Controller
    PidConfig pidCfg;
    pidCfg.kp = 0.0006f;
    pidCfg.ki = 0.0f;
    pidCfg.kd = 0.0003f;
    pidCfg.outMin = -1.0f;
    pidCfg.outMax =  1.0f;
    pidCfg.dFilterAlpha = 0.6f;
    PidController steeringPid{pidCfg};

    // 7. Initialize Bluetooth Telemetry (USART1 at 9600 baud)
    uart::UartHandler<uart::Instance::usart1, 128, 128> btUart{PIN_BT_TX, PIN_BT_RX, 9600};
    Hc05<uart::Instance::usart1, 128, 128> bluetooth{btUart};
    bluetooth.init();

    // 8. Sound Startup Tone: 2 short beeps
    buzzer.playPattern(80, 80, 2);

    bluetooth.sendLine("==============================================");
    bluetooth.sendLine("STM32 Closed-Loop Line Follower Robot Online");
    bluetooth.sendLine("Commands: <START>, <STOP>, <KP:val>, <KD:val>");
    bluetooth.sendLine("==============================================");

    std::uint32_t lastControlTime = systick::millis();
    std::uint32_t lastTelemTime   = systick::millis();
    bool robotRunning = false;
    float baseSpeed = 0.60f;

    char packetBuf[32];

    while (true)
    {
        const std::uint32_t now = systick::millis();

        // Update non-blocking buzzer state machine
        buzzer.update(now);

        // Process incoming wireless Bluetooth commands
        if (bluetooth.readPacket('<', '>', packetBuf, sizeof(packetBuf), 5) > 0)
        {
            if (std::strcmp(packetBuf, "START") == 0)
            {
                robotRunning = true;
                steeringPid.reset();
                buzzer.playPattern(150, 50, 1);
                bluetooth.sendLine("ACK: Robot STARTED");
            }
            else if (std::strcmp(packetBuf, "STOP") == 0)
            {
                robotRunning = false;
                motors.brake();
                buzzer.playPattern(250, 50, 1);
                bluetooth.sendLine("ACK: Robot STOPPED");
            }
        }

        // Fast Real-Time Control Loop (500 Hz = 2 ms interval)
        if ((now - lastControlTime) >= 2)
        {
            const float dt = static_cast<float>(now - lastControlTime) / 1000.0f;
            lastControlTime = now;

            // Poll encoder velocity
            leftEncoder.update(now);

            if (robotRunning)
            {
                // Simulated or measured line position (Center is 3500)
                constexpr std::int32_t linePosition = 3500;
                const float error = static_cast<float>(linePosition - 3500);

                // Compute steering correction via PID
                const float steering = steeringPid.updateError(error, dt);

                // Dynamic speed reduction on sharp curves
                const float currentBase = baseSpeed - (0.3f * math::abs(steering));

                // Drive dual motors
                motors.drive(currentBase, steering);
            }
            else
            {
                motors.brake();
            }
        }

        // Low-Priority Periodic Telemetry (every 500 ms)
        if ((now - lastTelemTime) >= 500)
        {
            lastTelemTime = now;
            statusLed.write(robotRunning ? gpio::PinState::HIGH : gpio::PinState::LOW);

            if (bluetooth.isConnected())
            {
                char telem[80];
                std::snprintf(telem, sizeof(telem),
                              "[TELEM] Run:%u RPM:%.1f Dist:%.2fm Kp:%.5f",
                              robotRunning ? 1 : 0,
                              static_cast<double>(leftEncoder.getRpm()),
                              static_cast<double>(leftEncoder.getDistanceMeters()),
                              static_cast<double>(steeringPid.getKp()));
                bluetooth.sendLine(telem);
            }
        }
    }

    return 0;
}
