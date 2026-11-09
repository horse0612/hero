#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <span>
#include "uart.hpp"

namespace JLUni::Remote_Ctrl_N
{
    enum class toggle_e : uint8_t { up = 1, low = 2, mid = 3 };

    enum class policy_e : uint8_t
    {
        disabled = 0x00,
        step_test = 0x15, sine_test = 0x25,
        slope_test = 0x17, reserved_4 = 0x27,
        reserved_5 = 0x16, reserved_6 = 0x26,
        reserved_7 = 0x1D, reserved_8 = 0x2D,
        chassis = 0x1F, keyboard = 0x2F,
        custom = 0x1E, host = 0x2E,
        right_near = 0x19, right_far = 0x29,
        left_near = 0x1B, left_far = 0x2B,
        error = 0x0A
    };

    struct rc_state_s
    {
        std::array<float, 4> lever{}; // Right horizontal/vertical, left horizontal/vertical.
        std::array<toggle_e, 2> toggle{}; // Right, left.
        uint16_t wheel = 0;
        int16_t x = 0, y = 0, z = 0;
        uint8_t press_l = 0, press_r = 0;
        uint16_t key_value = 0;
    };

    class RemoteCtrl
    {
    public:
        using Frame = std::array<uint8_t, 18>;
        explicit RemoteCtrl(LibXR::UART& uart) : uart_(uart) {}

        LibXR::ErrorCode Read(Frame& frame, LibXR::ReadOperation& operation) const
        {
            return uart_.Read({frame.data(), frame.size()}, operation);
        }

        // Receive-thread only
        void Parse(std::span<const uint8_t, 18> frame);
        const rc_state_s& State() const { return state_; }
        policy_e GetPolicy() const { return policy_.load(std::memory_order_relaxed); }

    private:
        LibXR::UART& uart_;
        rc_state_s state_{};
        std::atomic<policy_e> policy_{policy_e::disabled};
    };
}
