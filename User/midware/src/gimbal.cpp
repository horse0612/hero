#include "Gimbal.hpp"
#include "User_CAN.hpp"

#include <cmath>

#include "ins_task.hpp"
#include "remote_ctrl.hpp"
#include "Vision.hpp"
#include "Shooter.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "detect.hpp"

extern "C"
{
#include "arm_math.h"
#include "main.h"
#include "app_main.h"
}

namespace
{

inline float Clamp(const float value, const float minimum, const float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}
} 

extern "C"
{
Gimbal_t gimbal{};
float pi_result = kPi / 180.0f;
bool vision_flag = false;
}

extern "C"
{

void Gimbal_InitPID(void)
{
    PID_Init(&gimbal.pitch.singleanglePID, -4.5f, -0.01f, -35.0f, 2.0f, 10.0f);
    PID_Init(&gimbal.yaw_up.imuPID.inner, 120.0f, 0.0f, 1.0f, 7000.0f, 25000.0f);
    DEPID_Init(&gimbal.yaw_up.imuPID.deOuter, -170.0f, 0.0f, 8.0f, 4500.0f, 25000.0f, 0.45f);
}

void Gimbal_Init(void)
{
    
    gimbal.pitch.pitchMax = 35.0f;
    gimbal.pitch.pitchMin = -5.0f;
    gimbal.pitch.initAngle = 0.0f;
    gimbal.pitch.targetAngle = 0.0f;
    gimbal.yaw_up.initAngle = 0.0f;
    gimbal.yaw_up.targetAngle = 0.0f;
    gimbal.visionEnable = false;
    gimbal.rockerCtrl = false;
    Gimbal_InitPID();
}

void Gimbal_UpdataAngle(void)
{
    gimbal.pitch.gyro = User_Imu_Get_Y_Gyro();
    gimbal.pitch.angle = -User_Imu_GetPitch();
    gimbal.yaw_up.gyro = User_Imu_Get_Z_Gyro();
    gimbal.yaw_up.angle = User_Imu_GetYaw();
}

void Gimbal_VisionCtrl(void)
{
    gimbal.yaw_up.targetAngle = vision.yaw;
    gimbal.pitch.targetAngle = Clamp(vision.pitch, static_cast<float>(gimbal.pitch.pitchMin), static_cast<float>(gimbal.pitch.pitchMax));
}

void Gimbal_RockerCtrl(void)
{
    gimbal.yaw_up.targetAngle += (rcInfo.ch1) * 0.35f / 660.0f;
    gimbal.pitch.targetAngle += (rcInfo.ch2) * 0.35f / 660.0f;
    gimbal.pitch.targetAngle = Clamp(gimbal.pitch.targetAngle, static_cast<float>(gimbal.pitch.pitchMin), static_cast<float>(gimbal.pitch.pitchMax));
}

void Gimbal_Target_Limit(void)
{
    const float yawError = gimbal.yaw_up.targetAngle - gimbal.yaw_up.angle;
    if (yawError > 180.0f)
    {
        gimbal.yaw_up.targetAngle -= 360.0f;
    }
    else if (yawError < -180.0f)
    {
        gimbal.yaw_up.targetAngle += 360.0f;
    }
}

void Task_Gimbal_Callback(void)
{
    if (rcInfo.wheel > 500)
    {
        gimbal.rockerCtrl = true;
        gimbal.visionEnable = true;
    }
    else if (rcInfo.wheel < -600)
    {
        gimbal.rockerCtrl = false;
        gimbal.visionEnable = false;
    }

    if (gimbal.visionEnable && vision_receive.looking_in != 0U)
    {
        vision_flag = true;
        Gimbal_VisionCtrl();
    }
    else
    {
        vision_flag = false;
        Gimbal_RockerCtrl();
    }

    Gimbal_UpdataAngle();
    Gimbal_Target_Limit();

    EX_DEPID_CascadeCalc(&gimbal.yaw_up.imuPID, gimbal.yaw_up.targetAngle, gimbal.yaw_up.angle, gimbal.yaw_up.gyro);
    PID_SingleCalc(&gimbal.pitch.singleanglePID, gimbal.pitch.targetAngle, gimbal.pitch.angle);
}

void OS_GimbalCallback(void const *argument)
{
    (void)argument;
    while (App_IsInitialized() == 0U)
    {
         LibXR::Thread::Sleep(1);
    }
    for (;;)
    {
        CAN_RefreshGimbalFeedback();
        Task_Gimbal_Callback();
        LibXR::Thread::Sleep(1);
    }
}

} 