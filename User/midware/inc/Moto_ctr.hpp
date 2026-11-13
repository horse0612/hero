#pragma once
#include <cstdint>
#include "JLU_HERO/Math/inc/PID.hpp"
#include "CAN_DJI.hpp"
#include "CAN_DM.hpp"
#include "CAN_LK.hpp"

constexpr std::int32_t MOTOR_M3508_DGR2CODE(const float degree)
{
    return static_cast<std::int32_t>(degree * 436.9263f);
}

constexpr float MOTOR_M3508_CODE2DGR(const std::int32_t code)
{
    return static_cast<float>(code / 436.9263f);
}

constexpr std::int32_t MOTOR_M2006_DGR2CODE(const float degree)
{
    return static_cast<std::int32_t>(degree * 819.1f * 2.5f);
}

constexpr float MOTOR_M2006_CODE2DGR(const std::int32_t code)
{
    return static_cast<float>(code / 819.1f / 2.5f);
}

constexpr std::int32_t MOTOR_M6020_DGR2CODE(const float degree)
{
    return static_cast<std::int32_t>(degree * 22.7528f);
}

constexpr float MOTOR_M6020_CODE2DGR(const std::int32_t code)
{
    return static_cast<float>(code / 22.7528f);
}

constexpr std::int32_t MOTOR_MF9025_DGR2CODE(const float degree)
{
    return static_cast<std::int32_t>(degree * 182.04167f);
}

constexpr float MOTOR_MF9025_CODE2DGR(const std::int32_t code)
{
    return static_cast<float>(code / 182.04167f);
}

extern "C"
{

void Task_CANMotors_Callback(void);
void OS_MotorCallback(void const *argument);
}
