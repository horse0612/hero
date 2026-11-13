#include "remote_ctrl.hpp"

#include <bit>

namespace JLUni::Remote_Ctrl_N
{
    void RemoteCtrl::Parse(const std::span<const uint8_t, 18> frame)
    {
        const auto byte = [&](const uint8_t i) { return static_cast<uint32_t>(frame[i]); };
        const std::array<uint32_t, 4> channels
        {
            (byte(0) | (byte(1) << 8U)) & 0x7FFU,
            ((byte(1) >> 3U) | (byte(2) << 5U)) & 0x7FFU,
            ((byte(2) >> 6U) | (byte(3) << 2U) | (byte(4) << 10U)) & 0x7FFU,
            ((byte(4) >> 1U) | (byte(5) << 7U)) & 0x7FFU
        };
        for (uint8_t i = 0; i < 4; ++i)
        {
            const int raw = static_cast<int>(channels[i]) - 1024;
            auto& lever = state_.lever[i];
            lever = lever * 0.8F + (1.F - 0.8F) * static_cast<float>(raw) / 660.F;
            if (lever > -0.01F && lever < 0.01F) lever = 0.F;
        }
        state_.toggle[0] = static_cast<toggle_e>((byte(5) >> 4U) & 3U);
        state_.toggle[1] = static_cast<toggle_e>((byte(5) >> 6U) & 3U);
        const auto word = [&](const uint8_t offset)
        {
            return static_cast<uint16_t>(byte(offset) | (byte(offset + 1) << 8U));
        };
        state_.x = std::bit_cast<int16_t>(word(6));
        state_.y = std::bit_cast<int16_t>(word(8));
        state_.z = std::bit_cast<int16_t>(word(10));
        state_.press_l = frame[12];
        state_.press_r = frame[13];
        state_.key_value = word(14);
        state_.wheel = word(16) & 0x7FFU;

        const auto previous = GetPolicy();
        if (previous == policy_e::disabled && state_.wheel < 1280U) return;
        uint8_t next = static_cast<uint8_t>(previous) & 0x30U;  // mustn't store too early
        switch (state_.wheel >> 8U)
        {
        case 1:
        case 2:
            next = 0x10U;
            break;
        case 5:
        case 6:
            next = 0x20U;
            break;
        default: break;
        }
        next |= (static_cast<uint8_t>(state_.toggle[1]) << 2U) | static_cast<uint8_t>(state_.toggle[0]);
        policy_.store(static_cast<policy_e>(next), std::memory_order_relaxed);
    }
}
