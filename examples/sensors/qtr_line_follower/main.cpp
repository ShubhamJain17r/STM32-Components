/**
 * @file main.cpp
 * @brief QTR Analog Reflectance Line Sensor Array Example for STM32F446RE Nucleo.
 *
 * Demonstrates:
 * 1. CASE A: Cheap Amazon Clone (QTR-8A)
 *    - Emitters permanently on (VCC-tied, 0 emitter pins needed)
 *    - Dynamic calibration sweep completely canceling out cheap phototransistor mismatches
 * 2. CASE B: Custom Modular Sensor Board
 *    - MOSFET Emitter Control (PB0) for ambient light rejection (AutoDifferential mode)
 * 3. Weighted centroid calculation with lost-line memory for high-speed line followers.
 */

#include "stm32/common/systick/systick.hpp"
#include "stm32/gpio/gpio.hpp"
#include "stm32/adc/adc.hpp"

#include "components/sensors/qtr/qtr_analog.hpp"

using namespace components::sensors::qtr;

// 8 Sensor Pins on STM32F446RE (Mapped to ADC1 channels)
constexpr std::array<gpio::Pin, 8> SENSOR_PINS = {
    gpio::PC0, // ADC1_IN10 (Sensor 1 - Far Left)
    gpio::PC1, // ADC1_IN11 (Sensor 2)
    gpio::PC2, // ADC1_IN12 (Sensor 3)
    gpio::PC3, // ADC1_IN13 (Sensor 4 - Center Left)
    gpio::PA0, // ADC1_IN0  (Sensor 5 - Center Right)
    gpio::PA1, // ADC1_IN1  (Sensor 6)
    gpio::PA2, // ADC1_IN2  (Sensor 7)
    gpio::PA3  // ADC1_IN3  (Sensor 8 - Far Right)
};

// Optional MOSFET Emitter Control Pin for Custom Modular Board
constexpr gpio::Pin EMITTER_PIN = gpio::PB0;

int main()
{
    // 1. Initialize SysTick timebase (1ms monotonic ticks)
    systick::init(1000);

    // 2. Initialize onboard User LED (PA5 on Nucleo-F446RE) for calibration indicator
    gpio::DigitalOutput statusLed{gpio::PA5};

    // 3. Initialize ADC1 Peripheral (12-bit resolution, 84 clock cycles sampling)
    adc::AdcHandler<adc::Instance::adc1> adc{adc::Resolution::Bits12, adc::SamplingTime::Cycles84};

    // -------------------------------------------------------------------------
    // CASE A: CHEAP AMAZON CLONE (QTR-8A)
    // Emitters are wired directly to VCC on the module (no emitter pin).
    // -------------------------------------------------------------------------
    QtrAnalog<8> qtrClone(SENSOR_PINS, LineColor::DarkOnLight);
    qtrClone.init();

    // -------------------------------------------------------------------------
    // CASE B: CUSTOM MODULAR SENSOR BOARD (with MOSFET Emitter Control)
    // Supports active ambient light subtraction (AutoDifferential).
    // -------------------------------------------------------------------------
    [[maybe_unused]] QtrAnalog<8> qtrCustom(
        SENSOR_PINS,
        EMITTER_PIN,
        LineColor::DarkOnLight,
        true,                               // Active-High gate driving N-MOSFET
        EmitterPolicy::AutoDifferential    // Active ambient light subtraction
    );

    // -------------------------------------------------------------------------
    // Dynamic Calibration Phase (Essential for Cheap Amazon Clones)
    // Rapidly sweep the sensor array across the black line and white floor
    // -------------------------------------------------------------------------
    statusLed.high(); // LED ON indicates calibration in progress

    const std::uint32_t calibStartTime = systick::millis();
    while ((systick::millis() - calibStartTime) < 3000) // Calibrate for 3 seconds
    {
        qtrClone.calibrate(adc);
        systick::delayMs(5);
    }

    statusLed.low(); // LED OFF indicates calibration completed
    systick::delayMs(500);

    // -------------------------------------------------------------------------
    // Main Line-Tracking Loop
    // -------------------------------------------------------------------------
    std::array<std::uint16_t, 8> calibratedSensors;

    while (true)
    {
        // Calculate weighted centroid line position:
        // Range: 0 (far left) to 7000 (far right), with 3500 being dead center.
        const std::int32_t linePosition = qtrClone.readLine(adc, calibratedSensors);
        const bool lineSeen = qtrClone.isLineDetected();

        // Calculate steering error for line-following PID controller:
        // Error = 0 when perfectly centered on line
        // Error < 0 when line is to the left
        // Error > 0 when line is to the right
        [[maybe_unused]] const std::int32_t steeringError = linePosition - qtrClone.getCenterPosition();

        // Heartbeat LED: ON when line detected, OFF when track lost
        statusLed.write(lineSeen ? gpio::PinState::HIGH : gpio::PinState::LOW);

        systick::delayMs(2); // 500 Hz control loop frequency
    }

    return 0;
}
