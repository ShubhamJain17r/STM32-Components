/**
 * @file hc05.hpp
 * @brief Production-grade C++17 driver for HC-05 Bluetooth 2.0+EDR SPP Breakout Module.
 *
 * Designed exclusively for HC-05 breakout modules (e.g. ZS-040 with VCC, GND, TXD, RXD, STATE, EN/KEY).
 *
 * Features:
 * - Transparent SPP serial data streaming with non-blocking reception
 * - Robust line and framed packet parsing (<START:PAYLOAD:END>)
 * - Direct connection monitoring via the module's STATE pin
 * - Dynamic AT command configuration via the EN/KEY pin (Baud rate, Name, PIN, Role)
 * - Zero dynamic memory allocation (0 bytes heap)
 * - SysTick timeout guarded against bus or wireless stalls
 */

#pragma once

#include "stm32f446xx.h"
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstring>

#include "stm32/gpio/gpio.hpp"
#include "stm32/uart/uart.hpp"
#include "stm32/common/systick/systick.hpp"

#include "components/common/component_base.hpp"
#include "components/wireless/hc05/hc05_types.hpp"

namespace components::wireless::hc05
{

/**
 * @brief HC-05 Bluetooth Module Driver.
 *
 * @tparam UartInst STM32 UART Instance (e.g. uart::Instance::uart1)
 * @tparam TxBuf Internal UART transmit ring buffer size
 * @tparam RxBuf Internal UART receive ring buffer size
 */
template<uart::Instance UartInst, std::size_t TxBuf = 128, std::size_t RxBuf = 128>
class Hc05
{
public:
    Hc05() = delete;
    ~Hc05() = default;

    Hc05(const Hc05&) = delete;
    Hc05& operator=(const Hc05&) = delete;

    Hc05(Hc05&&) = default;
    Hc05& operator=(Hc05&&) = default;

    /**
     * @brief Construct for 4-pin connection (VCC, GND, TX, RX).
     * KEY and STATE pins are unmonitored.
     */
    explicit Hc05(uart::UartHandler<UartInst, TxBuf, RxBuf>& uart) noexcept
        : uart_(uart),
          hasKeyPin_(false),
          keyPin_{nullptr, 0xFF},
          hasStatePin_(false),
          statePin_{nullptr, 0xFF},
          mode_(OperatingMode::Data)
    {}

    /**
     * @brief Construct with STATE connection monitoring pin (5-pin connection).
     */
    Hc05(uart::UartHandler<UartInst, TxBuf, RxBuf>& uart,
         gpio::Pin statePin) noexcept
        : uart_(uart),
          hasKeyPin_(false),
          keyPin_{nullptr, 0xFF},
          hasStatePin_(true),
          statePin_(statePin),
          mode_(OperatingMode::Data)
    {}

    /**
     * @brief Construct with both KEY and STATE pins for full control (6-pin connection).
     *
     * @param uart Reference to initialized UART peripheral
     * @param keyPin STM32 pin connected to HC-05 KEY/EN pin
     * @param statePin STM32 pin connected to HC-05 STATE pin
     */
    Hc05(uart::UartHandler<UartInst, TxBuf, RxBuf>& uart,
         gpio::Pin keyPin,
         gpio::Pin statePin) noexcept
        : uart_(uart),
          hasKeyPin_(true),
          keyPin_(keyPin),
          hasStatePin_(true),
          statePin_(statePin),
          mode_(OperatingMode::Data)
    {}

    /**
     * @brief Initialize hardware pins and prepare module.
     */
    bool init() noexcept
    {
        if (hasKeyPin_)
        {
            keyPin_.enableClock();
            gpio::helper::setMode(keyPin_, gpio::Mode::OUTPUT);
            gpio::helper::setOutputType(keyPin_, gpio::OutputType::PUSH_PULL);
            gpio::helper::setOutputSpeed(keyPin_, gpio::OutputSpeed::LOW);
            gpio::helper::setPull(keyPin_, gpio::Pull::NONE);
            exitAtMode();
        }

        if (hasStatePin_)
        {
            statePin_.enableClock();
            gpio::helper::setMode(statePin_, gpio::Mode::INPUT);
            gpio::helper::setPull(statePin_, gpio::Pull::DOWN);
        }

        flush();
        return true;
    }

    // =========================================================================
    // Connection & Status APIs
    // =========================================================================

    /**
     * @brief Check whether an active SPP connection is established.
     * If the STATE pin is wired, reads physical hardware state directly.
     * If not wired, returns true (assumes connected/ready).
     */
    [[nodiscard]] bool isConnected() const noexcept
    {
        if (!hasStatePin_)
        {
            return true;
        }
        return gpio::DigitalInput{statePin_}.read() == gpio::PinState::HIGH;
    }

    /**
     * @brief Returns current connection state enum.
     */
    [[nodiscard]] ConnectionState getConnectionState() const noexcept
    {
        if (!hasStatePin_)
        {
            return ConnectionState::Unknown;
        }
        return isConnected() ? ConnectionState::Connected : ConnectionState::Disconnected;
    }

    [[nodiscard]] bool hasStatePin() const noexcept { return hasStatePin_; }
    [[nodiscard]] bool hasKeyPin()   const noexcept { return hasKeyPin_; }
    [[nodiscard]] OperatingMode getOperatingMode() const noexcept { return mode_; }

    // =========================================================================
    // Transparent SPP Data Streaming APIs
    // =========================================================================

    /**
     * @brief Send null-terminated ASCII string.
     */
    void send(const char* str) noexcept
    {
        uart_.write(str);
    }

    /**
     * @brief Send raw binary byte buffer.
     */
    void send(const std::uint8_t* data, std::size_t length) noexcept
    {
        uart_.write(data, length);
    }

    /**
     * @brief Send a single byte.
     */
    void sendByte(std::uint8_t byte) noexcept
    {
        uart_.writeByte(byte);
    }

    /**
     * @brief Send string followed by CRLF ("\r\n").
     */
    void sendLine(const char* str) noexcept
    {
        uart_.write(str);
        uart_.write("\r\n");
    }

    /**
     * @brief Asynchronous high-speed DMA string transmission.
     */
    void sendDma(const char* str, stm32::Callback onComplete = nullptr) noexcept
    {
        uart_.writeDma(str, onComplete);
    }

    /**
     * @brief Asynchronous high-speed DMA buffer transmission.
     */
    void sendDma(const std::uint8_t* data, std::size_t length, stm32::Callback onComplete = nullptr) noexcept
    {
        uart_.writeDma(data, length, onComplete);
    }

    /**
     * @brief Returns number of unread bytes in the receive ring buffer.
     */
    [[nodiscard]] std::size_t available() const noexcept
    {
        return uart_.available();
    }

    /**
     * @brief Read a single character from the RX buffer (blocks if empty).
     */
    char read() noexcept
    {
        return uart_.read();
    }

    /**
     * @brief Read a single byte from the RX buffer (blocks if empty).
     */
    std::uint8_t readByte() noexcept
    {
        return uart_.readByte();
    }

    /**
     * @brief Reads available bytes up to maxLen into destination buffer.
     * @return Number of bytes read.
     */
    std::size_t readBytes(std::uint8_t* buffer, std::size_t maxLen) noexcept
    {
        std::size_t count = 0;
        while (count < maxLen && uart_.available() > 0)
        {
            buffer[count++] = uart_.readByte();
        }
        return count;
    }

    /**
     * @brief Discard all unread bytes in the RX buffer.
     */
    void flush() noexcept
    {
        while (uart_.available() > 0)
        {
            (void)uart_.readByte();
        }
    }

    /**
     * @brief Non-blocking, timeout-protected line reader.
     *
     * Reads until '\n' or '\r\n' is encountered, or maxLen - 1 bytes are reached,
     * or the timeout expires. Strips carriage returns and adds null-terminator.
     *
     * @param buffer Destination character buffer
     * @param maxLen Buffer capacity including null terminator
     * @param timeoutMs Timeout in milliseconds
     * @return Number of characters read into buffer (excluding null terminator), or 0 on timeout.
     */
    std::size_t readLine(char* buffer, std::size_t maxLen, std::uint32_t timeoutMs = Defaults::LineTimeoutMs) noexcept
    {
        if (buffer == nullptr || maxLen <= 1)
        {
            return 0;
        }

        systick::Timeout timeout{timeoutMs};
        std::size_t idx = 0;

        while (!timeout.expired() && idx < (maxLen - 1))
        {
            if (uart_.available() > 0)
            {
                const char c = uart_.read();
                if (c == '\n')
                {
                    break;
                }
                if (c != '\r')
                {
                    buffer[idx++] = c;
                }
            }
        }

        buffer[idx] = '\0';
        return idx;
    }

    /**
     * @brief Non-blocking packet framing parser.
     *
     * Extracts payload delimited by startChar and endChar (e.g. "<MOTOR:100>").
     *
     * @param startChar Opening frame byte (e.g. '<')
     * @param endChar Closing frame byte (e.g. '>')
     * @param payload Destination buffer for extracted contents
     * @param maxLen Destination capacity
     * @param timeoutMs Timeout in milliseconds
     * @return Number of payload characters read, or 0 if framed packet not found.
     */
    std::size_t readPacket(char startChar, char endChar, char* payload, std::size_t maxLen,
                           std::uint32_t timeoutMs = Defaults::LineTimeoutMs) noexcept
    {
        if (payload == nullptr || maxLen <= 1)
        {
            return 0;
        }

        systick::Timeout timeout{timeoutMs};

        // 1. Hunt for start delimiter
        bool foundStart = false;
        while (!timeout.expired())
        {
            if (uart_.available() > 0)
            {
                if (uart_.read() == startChar)
                {
                    foundStart = true;
                    break;
                }
            }
        }

        if (!foundStart)
        {
            return 0;
        }

        // 2. Collect payload until end delimiter
        std::size_t idx = 0;
        while (!timeout.expired() && idx < (maxLen - 1))
        {
            if (uart_.available() > 0)
            {
                const char c = uart_.read();
                if (c == endChar)
                {
                    payload[idx] = '\0';
                    return idx;
                }
                payload[idx++] = c;
            }
        }

        payload[0] = '\0';
        return 0;
    }

    // =========================================================================
    // AT Command Configuration Mode (requires KEY/EN pin connected)
    // =========================================================================

    /**
     * @brief Switch module into AT command configuration mode.
     * Drives KEY/EN pin HIGH and waits for settling time.
     */
    bool enterAtMode() noexcept
    {
        if (!hasKeyPin_)
        {
            return false;
        }

        gpio::DigitalOutput key{keyPin_};
        key.high();
        systick::delayMs(Defaults::ModeSwitchDelayMs);
        mode_ = OperatingMode::AtCommand;
        return true;
    }

    /**
     * @brief Switch module back into transparent SPP data communication mode.
     * Drives KEY/EN pin LOW and waits for settling time.
     */
    bool exitAtMode() noexcept
    {
        if (!hasKeyPin_)
        {
            return false;
        }

        gpio::DigitalOutput key{keyPin_};
        key.low();
        systick::delayMs(Defaults::ModeSwitchDelayMs);
        mode_ = OperatingMode::Data;
        return true;
    }

    /**
     * @brief Send raw AT command and await response containing "OK".
     *
     * @param cmd AT command string (with or without "\r\n")
     * @param responseBuffer Optional buffer to receive full module response
     * @param maxRespLen Capacity of responseBuffer
     * @param timeoutMs Timeout waiting for response
     * @return true if response contains "OK".
     */
    bool sendAtCommand(const char* cmd, char* responseBuffer = nullptr, std::size_t maxRespLen = 0,
                       std::uint32_t timeoutMs = Defaults::AtTimeoutMs) noexcept
    {
        flush();

        uart_.write(cmd);
        // Ensure CRLF is sent if not included
        const std::size_t len = std::strlen(cmd);
        if (len < 2 || cmd[len - 2] != '\r' || cmd[len - 1] != '\n')
        {
            uart_.write("\r\n");
        }

        systick::Timeout timeout{timeoutMs};
        std::size_t respIdx = 0;
        bool okDetected = false;

        char window[4] = {0, 0, 0, 0};

        while (!timeout.expired())
        {
            if (uart_.available() > 0)
            {
                const char c = uart_.read();

                if (responseBuffer != nullptr && respIdx < (maxRespLen - 1))
                {
                    responseBuffer[respIdx++] = c;
                }

                // Rolling window to detect "OK"
                window[0] = window[1];
                window[1] = window[2];
                window[2] = c;

                if (window[1] == 'O' && window[2] == 'K')
                {
                    okDetected = true;
                    // Read a few extra bytes to consume trailing CRLF
                    systick::delayMs(20);
                    while (uart_.available() > 0 && responseBuffer != nullptr && respIdx < (maxRespLen - 1))
                    {
                        responseBuffer[respIdx++] = uart_.read();
                    }
                    break;
                }
            }
        }

        if (responseBuffer != nullptr && maxRespLen > 0)
        {
            responseBuffer[respIdx] = '\0';
        }

        return okDetected;
    }

    /**
     * @brief Ping test: sends "AT" and expects "OK".
     */
    bool ping(std::uint32_t timeoutMs = 500) noexcept
    {
        return sendAtCommand("AT", nullptr, 0, timeoutMs);
    }

    /**
     * @brief Set Bluetooth broadcast device name (AT+NAME=<name>).
     */
    bool setName(const char* name) noexcept
    {
        char cmd[48];
        std::snprintf(cmd, sizeof(cmd), "AT+NAME=%s", name);
        return sendAtCommand(cmd);
    }

    /**
     * @brief Set Bluetooth pairing PIN code (AT+PSWD=<pin>).
     */
    bool setPin(const char* pin) noexcept
    {
        char cmd[32];
        std::snprintf(cmd, sizeof(cmd), "AT+PSWD=%s", pin);
        return sendAtCommand(cmd);
    }

    /**
     * @brief Set Bluetooth operating role (AT+ROLE=0 for Slave, 1 for Master).
     */
    bool setRole(Role role) noexcept
    {
        char cmd[24];
        std::snprintf(cmd, sizeof(cmd), "AT+ROLE=%u", static_cast<unsigned>(role));
        return sendAtCommand(cmd);
    }

    /**
     * @brief Configure communication baud rate, stop bits, and parity (AT+UART=<baud>,<stop>,<parity>).
     */
    bool setBaudRate(std::uint32_t baud, StopBits stop = StopBits::One, Parity parity = Parity::None) noexcept
    {
        char cmd[40];
        std::snprintf(cmd, sizeof(cmd), "AT+UART=%lu,%u,%u",
                      static_cast<unsigned long>(baud),
                      static_cast<unsigned>(stop),
                      static_cast<unsigned>(parity));
        return sendAtCommand(cmd);
    }

    /**
     * @brief Query firmware version (AT+VERSION?).
     */
    bool getVersion(char* buffer, std::size_t maxLen) noexcept
    {
        return sendAtCommand("AT+VERSION?", buffer, maxLen);
    }

    /**
     * @brief Software reset the module (AT+RESET).
     */
    bool reset() noexcept
    {
        return sendAtCommand("AT+RESET");
    }

private:
    uart::UartHandler<UartInst, TxBuf, RxBuf>& uart_;
    bool                                       hasKeyPin_;
    gpio::Pin                                  keyPin_;
    bool                                       hasStatePin_;
    gpio::Pin                                  statePin_;
    OperatingMode                              mode_;
};

} // namespace components::wireless::hc05
