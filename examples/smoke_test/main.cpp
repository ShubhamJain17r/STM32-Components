/**
 * @file main.cpp
 * @brief Smoke test application verifying STM32-Library linking and Layer 2 components base.
 *
 * Runs on Nucleo-F446RE:
 * - Initializes SysTick monotonic timebase
 * - Toggles Onboard User LED (PA5) at 2 Hz
 * - Exercises components::Status and components::math utilities
 */

#include "stm32/common/systick/systick.hpp"
#include "stm32/gpio/gpio.hpp"

#include "components/common/component_base.hpp"
#include "components/common/math_utils.hpp"

int main()
{
    // Initialize monotonic SysTick timebase at 1000 Hz (1ms per tick)
    systick::init(1000);

    // Initialize Onboard User LED (LD2 on Nucleo-F446RE = PA5)
    gpio::DigitalOutput led{gpio::PA5};

    // Verify component base status check
    constexpr components::Status initStatus = components::Status::Ok;
    static_assert(components::isOk(initStatus), "Status evaluation must be constexpr-safe");

    // Verify component math utilities
    constexpr float clamped = components::math::clamp(1.5f, -1.0f, 1.0f);
    static_assert(clamped == 1.0f, "Math clamp must be constexpr-safe");

    std::uint32_t lastToggle = systick::millis();
    bool ledState = false;

    while (true)
    {
        const std::uint32_t now = systick::millis();
        if ((now - lastToggle) >= 250) // 250 ms toggle = 2 Hz blink
        {
            lastToggle = now;
            ledState = !ledState;
            led.write(ledState ? gpio::PinState::HIGH : gpio::PinState::LOW);
        }
    }

    return 0;
}
