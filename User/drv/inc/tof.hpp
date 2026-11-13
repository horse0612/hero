#pragma once

#include <cstdint>
#include <span>
#include "uart.hpp"

namespace JLUni::TOF_N
{
    enum class unit_e : uint8_t { cm = 0x01, mm = 0x06 };
    enum class command_e : uint8_t
    {
        version = 0x01, reset = 0x02, rate = 0x03, unit = 0x05,
        baud = 0x06, output = 0x07, restore = 0x10, save = 0x11
    };
    enum class update_result_e : uint8_t { NONE, MEASUREMENT, RESPONSE };
    struct measurement_s { uint16_t distance_raw; uint16_t strength; float temperature_c; };
    // Little-endian payload. Version: patch in bits 0..7, minor 8..15, major 16..23.
    struct response_s { command_e command; uint32_t value; };

    class TFminiPlus
    {
    public:
        explicit TFminiPlus(LibXR::UART& uart) : uart_(uart) {}
        // Zero: invalid prefix. Otherwise next required size, or full frame size.
        // An empty prefix needs one byte. No buffering or UART access.
        static uint8_t ExpectFrameSize(std::span<const uint8_t> prefix);
        static update_result_e UpdateFrame(std::span<const uint8_t> frame,
            measurement_s& measurement, response_s* response = nullptr);

        // Caller owns the semaphore; one serial sender per UART. Return value
        // reports UART write completion, not device acknowledgement.
        LibXR::ErrorCode RequestFirmwareVersion(LibXR::Semaphore& sem) const;
        LibXR::ErrorCode Reset(LibXR::Semaphore& sem) const;
        // Manual recommends 1000/n Hz; accepts all integers in 1..1000.
        LibXR::ErrorCode SetOutputRate(uint16_t hz, LibXR::Semaphore& sem) const;
        LibXR::ErrorCode SetOutputUnit(unit_e unit, LibXR::Semaphore& sem) const;
        // Does not change the MCU UART configuration.
        LibXR::ErrorCode SetBaudRate(uint32_t baud, LibXR::Semaphore& sem) const;
        LibXR::ErrorCode SetOutputEnabled(bool enabled, LibXR::Semaphore& sem) const;
        LibXR::ErrorCode RestoreFactorySettings(LibXR::Semaphore& sem) const;
        // Never called implicitly by another command.
        LibXR::ErrorCode SaveSettings(LibXR::Semaphore& sem) const;
    private:
        LibXR::ErrorCode Send(command_e command, LibXR::Semaphore& sem, uint32_t value = 0, uint8_t bytes = 0) const;
        LibXR::UART& uart_;
    };
}
