#pragma once
#include <array>
#include "math_utils.hpp"

namespace JLUni::ImuCalibration
{
    inline constexpr std::array<float, 3> accel_bias_raw{{0.F, 0.F, 0.F}};
    inline constexpr std::array<float, 9> accel_correction_matrix
    {{
        24.F * Math::gravity / 32768.F, 0.F, 0.F,
        0.F, 24.F * Math::gravity / 32768.F, 0.F,
        0.F, 0.F, 24.F * Math::gravity / 32768.F
    }};
}
