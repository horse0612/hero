#pragma once
#include "bmi088.hpp"
#include "ws2812.hpp"

namespace JLUni
{
    struct imu_calib_args_s
    {
        BMI088* imu;
        WS2812* led;
    };

    void ImuCalibThread(imu_calib_args_s* args);
} // namespace JLUni
