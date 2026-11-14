#include "shooter.hpp"

#include <cmath>
#include <cstdint>

#include "remote_ctrl.hpp"
#include "gimbal.hpp"
#include "vision.hpp"
#include "User_CAN.hpp"
#include "libxr.hpp"

#include "advanced_pid.hpp"

extern "C"
{
#include "main.h"
#include "app_main.h"
#include "fdcan.h"
}

JLUni::Shooter_N shooter{};

namespace JLUni::Shooter_N
{
    void Shooter_InitPID(void)
    {
        for (auto &motor : shooter.fricMotor)
        {
            PID_Init(&motor.singlePID, 40.0f, 0.0f, 5.0f, 10000.0f, 16000.0f);
        }
        #if TRIG_MOTOR_MODE == 0
        DEPID_Init(&shooter.dm4310_trig.dePID,100.0f, 50.0f, 0.6f, 15.0f, 20.0f, 0.5f);
        #elif TRIG_MOTOR_MODE == 1
        DEPID_Init(&shooter.dm4310_trig.dePID,23.0f, 0.001f, 1.0f, 2.0f, 15.0f, 0.3f);
        #endif
    }

    void Shooter_Init(void)
    {
        shooter.rockerCtrl = false;
        Slope_Init(&shooter.fricSlope, 10.0f, 0.0f);
        Slope_SetTarget(&shooter.fricSlope, 0.0f);
        shooter.fricOpenFlag = false;
        shooter.workState = IDLE;
        shooter.judgeCnt = 0U;
        vision_delay_time = 0;
        last_shoot_mode = IDLE;
        Shooter_InitPID();
    }


    #if TRIG_MOTOR_MODE == 0
    float TrigMotor_JudgeAngleRecently(void)
    {
        float correction = std::remainder( Trig_Start_Angle - shooter.dm4310_trig.pos, kTriggerStep);
        return correction;
        
    }
    #elif TRIG_MOTOR_MODE == 1
    float TrigMotor_JudgeAngleRecently(void)
    {
        int best_start = 0;
        float best = std::fabs(std::remainder(shooter.dm4310_trig.pos * Trig_Moto2Gear - kTriggerStarts[0], (2.0f * kPi)));
        for (int i = 1; i < 8; ++i)
        {
            float error = std::fabs(std::remainder(shooter.dm4310_trig.pos * Trig_Moto2Gear - kTriggerStarts[i], (2.0f * kPi)));
            if (error < best)
            {
                best = error;
                best_start = i;
            }
        }
        float correction = std::remainder(kTriggerStarts[best_start] - shooter.dm4310_trig.pos * Trig_Moto2Gear, (2.0f * kPi))* Trig_Gear2Moto;
        //拨盘角度误差换算为电机轴角度误差
        return correction ;
    }
    #endif

    void TrigMotor_SetCalibration(void)
    {
        float value = shooter.dm4310_trig.totalangle + TrigMotor_JudgeAngleRecently();
        shooter.dm4310_trig.total_target_pos = value;
        if (fabs(shooter.dm4310_trig.total_target_pos - shooter.dm4310_trig.totalangle) < 0.1f)
        {
            shooter.dm4310_trig.total_target_pos = 0;
            shooter.dm4310_trig.totalangle = 0;
            trig_calibration_flag = true;
        }

    }

    void SetFrictionTargets(const float speed, const std::int16_t offset, DJImotor (&motors)[6])
    {
        motors[0].targetSpeed = (-speed);
        motors[1].targetSpeed = (speed);
        motors[2].targetSpeed = (-speed);
        motors[3].targetSpeed = (-(speed - (offset)));
        motors[4].targetSpeed = (  speed - (offset));
        motors[5].targetSpeed = (-(speed - (offset)));
    }
    
    void Shooter_RockerCtrl(void)
    {
        if (rcInfo.right == 1)
        {
            shooter.fricOpenFlag = true;
            Slope_SetTarget(&shooter.fricSlope, static_cast<float>(shooter.fricSpd1));
        }
        else
        {
            shooter.fricOpenFlag = false;
            Slope_SetTarget(&shooter.fricSlope, 0.0f);
        }

        Slope_NextVal(&shooter.fricSlope);
        SetFrictionTargets(Slope_GetVal(&shooter.fricSlope), shooter.fric_offset1, shooter.fricMotor);
        if (rcInfo.left == 1 && !trig_flag_ready && shooter.fricOpenFlag)
        {
            trig_flag_ready = true;
            shooter.workState = TRIGGER;
        }
        else if (rcInfo.left != 1)
        {
            trig_flag_ready = false;
        }
    }

    void Task_Shooter_Callback(void)
    {
        if (rcInfo.wheel > 600)
        {
            shooter.rockerCtrl = true;
        }
        else if (rcInfo.wheel < -600)
        {
            shooter.rockerCtrl = false;
        }

        if (shooter.rockerCtrl)
        {
            Shooter_RockerCtrl();
        }
        else
        {
            Slope_NextVal(&shooter.fricSlope);
            float speed = Slope_GetVal(&shooter.fricSlope);
            if (shooter.fricOpenFlag)
            {
                SetFrictionTargets(speed, shooter.fric_offset1, shooter.fricMotor);
            }
            else
            {
                SetFrictionTargets(speed, 0, shooter.fricMotor);
            }
        }

        if (fabs(shooter.dm4310_trig.vel) <= 1.5f * Trig_Gear2Moto && fabs(shooter.dm4310_trig.total_target_pos - shooter.dm4310_trig.totalangle) >= kTriggerStep && last_shoot_mode!= TRIGGER_REVERSE)
        {
            shooter.block.judgeCnt++;
            if (shooter.block.judgeCnt >= 150)
            {
                shooter.block.judgeCnt = 0;
                shooter.workState = TRIGGER_REVERSE;
            }
        }
        else 
        {
            shooter.block.judgeCnt = 0;
        }

        if (gimbal.visionEnable)
        {
            ++vision_delay_time;
            if (vision_delay_time > 20 && vision_receive.fire != 0.0f  && shooter.fricOpenFlag)
            {
                shooter.workState = TRIGGER;
            }
        }
        else
        {
            vision_delay_time = 0;
        }

        switch (shooter.workState)
        {
        case IDLE:
            last_shoot_mode = IDLE;
            break;
        case TRIGGER:
            if (shooter.fricOpenFlag)
            {
                #if   TRIG_MOTOR_MODE == 0
                shooter.dm4310_trig.total_target_pos += kTriggerStep;
                #elif TRIG_MOTOR_MODE == 1
                shooter.dm4310_trig.total_target_pos -= kTriggerStep * Trig_Gear2Moto;
                #endif
                last_shoot_mode = TRIGGER;
                shooter.workState = IDLE;
            }
            break;
        case TRIGGER_REVERSE:
            {
                #if   TRIG_MOTOR_MODE == 0
                shooter.dm4310_trig.total_target_pos -= 2.0f * kTriggerStep;
                #elif TRIG_MOTOR_MODE == 1
                shooter.dm4310_trig.total_target_pos += 2.0f * kTriggerStep * Trig_Gear2Moto;
                #endif
                last_shoot_mode   = TRIGGER_REVERSE;
                shooter.workState = IDLE;
            }
            break;
        default:
            break;
        }
        
        #if   TRIG_MOTOR_MODE == 0
        PIDRegulation(&shooter.dm4310_trig.dePID, shooter.dm4310_trig.total_target_pos, shooter.dm4310_trig.totalangle, -shooter.dm4310_trig.vel);
        #elif TRIG_MOTOR_MODE == 1
        PIDRegulation(&shooter.dm4310_trig.dePID, shooter.dm4310_trig.total_target_pos, shooter.dm4310_trig.totalangle, -shooter.dm4310_trig.vel);
        #endif
        for (auto &motor : shooter.fricMotor)
        {
            PID_SingleCalc(&motor.singlePID, motor.targetSpeed, motor.speed);
        }
    }

}
