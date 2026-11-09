#include "tof.hpp"
#include <array>

namespace JLUni::TOF_N
{
    namespace
    {
        constexpr uint8_t measurement_head = 0x59, command_head = 0x5A;
        constexpr uint8_t measurement_length = 9, status_length = 5;
        constexpr uint8_t version_length = 7, rate_length = 6, baud_length = 8;

        constexpr uint16_t Word(const uint8_t* data)
        {
            return static_cast<uint16_t>(static_cast<uint32_t>(data[0])
                | (static_cast<uint32_t>(data[1]) << 8U));
        }
    }

    uint8_t TFminiPlus::ExpectFrameSize(std::span<const uint8_t> prefix)
    {
        if (prefix.empty()) return 1;
        if (prefix[0] != measurement_head && prefix[0] != command_head) return 0;
        if (prefix.size() < 2) return 2;
        if (prefix[0] == measurement_head) return prefix[1] == measurement_head ? measurement_length : 0;
        if (prefix[1] < status_length || prefix[1] > baud_length) return 0;
        if (prefix.size() < 3) return 3;
        uint8_t expected = 0;
        switch (static_cast<command_e>(prefix[2]))
        {
        case command_e::version:
            expected = version_length;
            break;
        case command_e::rate:
            expected = rate_length;
            break;
        case command_e::baud:
            expected = baud_length;
            break;
        case command_e::reset:
        case command_e::unit:
        case command_e::output:
        case command_e::restore:
        case command_e::save:
            expected = status_length;
            break;
        default:
            break;
        }
        return prefix[1] == expected ? expected : 0;
    }

    update_result_e TFminiPlus::UpdateFrame(std::span<const uint8_t> frame,
        measurement_s& measurement, response_s* response)
    {
        if (frame.size() < 5 || ExpectFrameSize(frame) != frame.size()) return update_result_e::NONE;
        uint8_t sum = 0;
        for (size_t i = 0; i + 1 < frame.size(); ++i) sum += frame[i];
        if (sum != frame.back()) return update_result_e::NONE;
        if (frame[0] == measurement_head)
        {
            measurement =
            {
                .distance_raw = Word(frame.data() + 2), .strength = Word(frame.data() + 4),
                .temperature_c = static_cast<float>(Word(frame.data() + 6)) / 8.0F - 256.0F
            };
            return update_result_e::MEASUREMENT;
        }
        if (response != nullptr)
        {
            uint32_t value = 0;
            for (size_t i = 3; i + 1 < frame.size(); ++i)
                value |= static_cast<uint32_t>(frame[i]) << (8U * (i - 3));
            *response = {.command = static_cast<command_e>(frame[2]), .value = value};
        }
        return update_result_e::RESPONSE;
    }

    LibXR::ErrorCode TFminiPlus::Send(command_e command, LibXR::Semaphore& sem,
        const uint32_t value, const uint8_t bytes) const
    {
        std::array<uint8_t, 8> frame{command_head, static_cast<uint8_t>(bytes + 4), static_cast<uint8_t>(command)};
        for (uint8_t i = 0; i < bytes; ++i) frame[3 + i] = static_cast<uint8_t>(value >> (8U * i));
        for (uint8_t i = 0; i < bytes + 3; ++i) frame[bytes + 3] += frame[i];
        LibXR::WriteOperation operation(sem);
        return uart_.Write({frame.data(), frame[1]}, operation);
    }

    LibXR::ErrorCode TFminiPlus::RequestFirmwareVersion(LibXR::Semaphore& sem) const
    {
        return Send(command_e::version, sem);
    }
    LibXR::ErrorCode TFminiPlus::Reset(LibXR::Semaphore& sem) const
    {
        return Send(command_e::reset, sem);
    }

    LibXR::ErrorCode TFminiPlus::SetOutputRate(const uint16_t hz, LibXR::Semaphore& sem) const
    {
        if (hz < 1 || hz > 1000) return LibXR::ErrorCode::ARG_ERR;
        return Send(command_e::rate, sem, hz, 2);
    }

    LibXR::ErrorCode TFminiPlus::SetOutputUnit(unit_e unit, LibXR::Semaphore& sem) const
    {
        if (unit != unit_e::cm && unit != unit_e::mm) return LibXR::ErrorCode::ARG_ERR;
        return Send(command_e::unit, sem, static_cast<uint8_t>(unit), 1);
    }

    LibXR::ErrorCode TFminiPlus::SetBaudRate(const uint32_t baud, LibXR::Semaphore& sem) const
    {
        if (baud < 9600 || baud > 921600) return LibXR::ErrorCode::ARG_ERR;
        return Send(command_e::baud, sem, baud, 4);
    }

    LibXR::ErrorCode TFminiPlus::SetOutputEnabled(const bool enabled, LibXR::Semaphore& sem) const
    {
        return Send(command_e::output, sem, enabled, 1);
    }

    LibXR::ErrorCode TFminiPlus::RestoreFactorySettings(LibXR::Semaphore& sem) const
    {
        return Send(command_e::restore, sem);
    }
    LibXR::ErrorCode TFminiPlus::SaveSettings(LibXR::Semaphore& sem) const
    {
        return Send(command_e::save, sem);
    }
}
