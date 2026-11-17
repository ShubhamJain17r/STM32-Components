/**
 * @file main.cpp
 * @brief HC-05 Bluetooth Breakout Module Example for STM32F446RE Nucleo.
 *
 * Wiring to HC-05 Breakout Module (ZS-040):
 * - VCC   -> 5V or 3.3V (Module has onboard 3.3V LDO)
 * - GND   -> GND
 * - TXD   -> PB7 (STM32 USART1_RX)
 * - RXD   -> PB6 (STM32 USART1_TX)
 * - STATE -> PC7 (Connection detection input: HIGH = paired/connected)
 * - EN/KEY-> PC8 (AT mode control output: HIGH = AT mode, LOW = Data mode)
 *
 * Demonstrates:
 * 1. Non-blocking transparent SPP data telemetry.
 * 2. Hardware connection state detection via the module's STATE pin.
 * 3. Command packet parsing (<CMD:VALUE> and line-based commands).
 * 4. Periodic telemetry streaming to paired smartphone/PC terminal.
 */

#include "stm32/common/systick/systick.hpp"
#include "stm32/gpio/gpio.hpp"
#include "stm32/uart/uart.hpp"

#include "components/wireless/hc05/hc05.hpp"

using namespace components::wireless::hc05;

// Pin Definitions on STM32F446RE Nucleo
constexpr gpio::Pin PIN_BT_TX    = gpio::PB6; // STM32 TX -> HC-05 RXD
constexpr gpio::Pin PIN_BT_RX    = gpio::PB7; // STM32 RX <- HC-05 TXD
constexpr gpio::Pin PIN_BT_STATE = gpio::PC7; // Connection state input
constexpr gpio::Pin PIN_BT_KEY   = gpio::PC8; // AT mode control output (KEY/EN)

int main()
{
    // 1. Initialize SysTick monotonic timebase (1ms ticks)
    systick::init(1000);

    // 2. Initialize onboard User LED (PA5 on Nucleo-F446RE)
    gpio::DigitalOutput led{gpio::PA5};

    // 3. Initialize USART1 peripheral at 9600 baud for HC-05 transparent SPP mode
    uart::UartHandler<uart::Instance::usart1, 128, 128> btUart{PIN_BT_TX, PIN_BT_RX, 9600};

    // 4. Instantiate HC-05 Driver with STATE and KEY pin monitoring
    Hc05<uart::Instance::usart1, 128, 128> bluetooth{btUart, PIN_BT_KEY, PIN_BT_STATE};
    bluetooth.init();

    // 5. Send initial banner over Bluetooth
    bluetooth.sendLine("========================================");
    bluetooth.sendLine("STM32F446RE HC-05 Telemetry Online");
    bluetooth.sendLine("Commands: PING, LED:ON, LED:OFF, <HELP>");
    bluetooth.sendLine("========================================");

    char lineBuffer[64];
    char packetBuffer[32];
    std::uint32_t lastTelemetryTime = systick::millis();
    std::uint32_t lastBlinkTime = systick::millis();
    std::uint32_t telemetryCounter = 0;

    while (true)
    {
        const std::uint32_t now = systick::millis();
        const bool connected = bluetooth.isConnected();

        // ---------------------------------------------------------------------
        // Visual Status Indication
        // ---------------------------------------------------------------------
        if (connected)
        {
            // Solid ON when Bluetooth paired/connected
            led.high();
        }
        else
        {
            // Rapid blink (5 Hz) when searching/waiting for pairing
            if ((now - lastBlinkTime) >= 100)
            {
                lastBlinkTime = now;
                led.write(gpio::DigitalInput{gpio::PA5}.read() == gpio::PinState::HIGH
                              ? gpio::PinState::LOW
                              : gpio::PinState::HIGH);
            }
        }

        // ---------------------------------------------------------------------
        // Receive and Process Framed Packets: <COMMAND:DATA>
        // ---------------------------------------------------------------------
        if (bluetooth.readPacket('<', '>', packetBuffer, sizeof(packetBuffer), 10) > 0)
        {
            if (std::strcmp(packetBuffer, "HELP") == 0)
            {
                bluetooth.sendLine("Available commands: <PING>, <STATUS>, <LED:ON>, <LED:OFF>");
            }
            else if (std::strcmp(packetBuffer, "PING") == 0)
            {
                bluetooth.sendLine("<PONG>");
            }
            else if (std::strcmp(packetBuffer, "STATUS") == 0)
            {
                char statusMsg[48];
                std::snprintf(statusMsg, sizeof(statusMsg), "<STATUS:CONNECTED,TIME:%lu>",
                              static_cast<unsigned long>(now));
                bluetooth.sendLine(statusMsg);
            }
        }

        // ---------------------------------------------------------------------
        // Receive and Process Line-based Commands: "PING\n", "LED:ON\n"
        // ---------------------------------------------------------------------
        if (bluetooth.available() > 0)
        {
            if (bluetooth.readLine(lineBuffer, sizeof(lineBuffer), 20) > 0)
            {
                if (std::strcmp(lineBuffer, "PING") == 0)
                {
                    bluetooth.sendLine("PONG");
                }
                else if (std::strcmp(lineBuffer, "LED:ON") == 0)
                {
                    led.high();
                    bluetooth.sendLine("ACK: LED Turned ON");
                }
                else if (std::strcmp(lineBuffer, "LED:OFF") == 0)
                {
                    led.low();
                    bluetooth.sendLine("ACK: LED Turned OFF");
                }
            }
        }

        // ---------------------------------------------------------------------
        // Periodic Telemetry Streaming (every 1000 ms when connected)
        // ---------------------------------------------------------------------
        if (connected && (now - lastTelemetryTime) >= 1000)
        {
            lastTelemetryTime = now;
            telemetryCounter++;

            char telemetryMsg[64];
            std::snprintf(telemetryMsg, sizeof(telemetryMsg),
                          "[TELEM #%lu] Uptime: %lu ms, Link: OK",
                          static_cast<unsigned long>(telemetryCounter),
                          static_cast<unsigned long>(now));
            bluetooth.sendLine(telemetryMsg);
        }
    }

    return 0;
}
