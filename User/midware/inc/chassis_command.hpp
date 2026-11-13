#pragma once

#include <cstdint>

namespace JLUni::Chassis_N
{
    enum class motion_mode_e : uint8_t
    {
        flat,
        lift,
        climb,
        retract,
        steady
    };

    struct chassis_cmd_s
    {
        // Body-frame command: x forward, y left, angular velocity about body z.
        float vx = 0.0F, vy = 0.0F, wz = 0.0F;
        motion_mode_e mode = motion_mode_e::flat;
    };

}
