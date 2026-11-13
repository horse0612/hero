#include "Moto_ctr.hpp"
#include "Gimbal.hpp"
#include "Shooter.hpp"
#include "User_CAN.hpp"
#include "Judge.hpp"
#include "libxr.hpp"
#include "CAN_DJI.hpp"
#include "CAN_DM.hpp"
#include "CAN_LK.hpp"
#include "app_main.h"
#include "detect.hpp"
extern "C"
{
#include "cmsis_os.h"
}

extern "C"
{

void Task_CANMotors_Callback(void)
{

    switch (gimbal.dm8518_yaw.state)
    {
        case 0:
            DMstrat(&hfdcan1, 0x04);
        break;
        case 1:
            MIT_CtrlMotor(&hfdcan1, 0x04, 0.0f, gimbal.dm8518_yaw.dePID.output, 0.0f, 2.0f, 0);
        break;
        default:
            DMerror(&hfdcan1, 0x04);
        break;
    }

    switch (shooter.dm4310_trig.state)
    {
        case 0:
            DMstrat(&hfdcan1, 0x02);
        break;
        case 1:
            #if TRIG_MOTOR_MODE == 0
            //lgp的侧切拨盘参数
            MIT_CtrlMotor(&hfdcan1, 0x02, 0.0f, shooter.dm4310_trig.dePID.output, 0.0f, 2.0f, DM4310_Trig_StaticTorque);
            #elif TRIG_MOTOR_MODE == 1
            //中供拨盘26.10.30
             MIT_CtrlMotor(&hfdcan1, 0x02, 0.0f, shooter.dm4310_trig.dePID.output, 0.0f, 4.0f, DM4310_Trig_StaticTorque);
            #endif
        break;
        default:
            DMerror(&hfdcan1, 0x02);
        break;
    }
}

void OS_MotorCallback(void const *argument)
{
    (void)argument;

    while (App_IsInitialized() == 0)
    {
        LibXR::Thread::Sleep(1);
    }


    for (;;)
    {
        Task_CANMotors_Callback();
        LibXR::Thread::Sleep(1);
    }
}
}
