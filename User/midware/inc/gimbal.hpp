#pragma once

#include <cstdint>

#include "advanced_pid.hpp"
#include "Moto_ctr.hpp"

struct Gimbal_t
{
    DMmotor dm8518_yaw{};
    DJImotor dji6020_pitch{};
    struct
    {
        float initAngle{};
        float angle{};
        float Motor_IMU_Angle{};
        float Motor_IMU_TargetAngle{};
        float lastAngle{};
        float totalAngle{};
        float gyro{};
        float targetAngle{};
        float lastTargetAngle{};
        CascadePID imuPID{};
        PID singleanglePID{};
    } yaw_up;
    
    struct
    {
        float initAngle{};
        float angle{};
        float Motor_IMU_Angle{};
        float Motor_IMU_TargetAngle{};
        float lastAngle{};
        float gyro{};
        float targetAngle{};
        float pitchMax{};
        float pitchMin{};
        CascadePID imuPID{};
        PID singleanglePID{};
    } pitch;

    std::int16_t turn_flag{};
    bool visionEnable{};
    bool rockerCtrl{};
};

extern "C"
{
extern Gimbal gimbal;
extern float pitch_forward;

void Gimbal_Init(void);
void Gimbal_InitPID(void);
void Gimbal_Target_Limit(void);
void Gimbal_UpdataAngle(void);
void Gimbal_RockerCtrl(void);
void Gimbal_VisionCtrl(void);
void Task_Gimbal_Callback(void);
void OS_GimbalCallback(void const *argument);
}
