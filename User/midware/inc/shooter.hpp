#pragma once

#include <array>
#include <cstdint>

#include "advanced_pid.hpp"
#include "motor_dm.hpp"
#include "motor_dji.hpp"
#include "remote_ctrl.hpp"
#include "slope.hpp"
#include "can.hpp"
#include "Moto_ctr.hpp"

#ifndef TRIG_MOTOR_MODE
#define TRIG_MOTOR_MODE 1 //0:侧切拨盘 1：中供拨盘 改变的是校准方式,以及有没有传动比
#endif

namespace JLUni::Shooter_N
{   
    inline static constexpr float kPi = LibXR::PI;
    enum class WorkState : std::uint8_t//枚举三种送弹状态
    {
        IDLE = 0,
        TRIGGER,
        TRIGGER_REVERSE
    };

    struct ShooterInput//云台从外界获取的输入，关于遥控器和视觉
    {
        Remote_Ctrl_N::rc_state_s remote{};//遥控器输入，对应rcInfo
        bool rockerCtrl{};//是否由遥控器控制
        bool vision_enable  = false;//视觉功能是否使能，以前的vision_enble
        bool vision_receive = false;//视觉是否开火
    };

    struct DM4310_Trig//拨盘电机的运行时变量，功能和原先的DMmotor dm4310_trig相近
    {
        inline static constexpr float Trig_Start_Angle = 1.26282001f;//-0.09f
        inline static constexpr float Trig_Gear2Moto = 88.0f/40.0f;
        inline static constexpr float Trig_Moto2Gear = 40.0f/88.0f;
        #if     (TRIG_MOTOR_MODE == 0)//选择不同的拨盘, 0:侧切拨盘 1：中供拨盘
        inline static constexpr float kTriggerStep = kPi / 3.0f;
        #elif   (TRIG_MOTOR_MODE == 1)
        inline static constexpr float kTriggerStep = kPi / 4.0f;
        inline static constexpr float kTriggerOffset = - 4.04f;//测出来校准后的其中一个槽位
        //后面有新的结构，应该是用输出力矩的突变来校准，这个暂时不需要
        inline static constexpr float kTriggerStarts[8] = 
        {
            -kPi * 2.0f / 4.0f - kTriggerOffset,
            -kPi * 1.0f / 4.0f - kTriggerOffset,
            -kTriggerOffset,
            kPi * 1.0f / 4.0f - kTriggerOffset,
            kPi * 2.0f / 4.0f - kTriggerOffset,
            kPi * 3.0f / 4.0f - kTriggerOffset,
            kPi * 4.0f / 4.0f - kTriggerOffset,
            kPi * 5.0f / 4.0f - kTriggerOffset
        };
        #endif

        WorkState workState{WorkState::IDLE};//当前送弹状态
        WorkState last_shoot_mode{WorkState::IDLE};//上次送弹状态
        MotorDM_N::feedback_s dm4310_trig{};//拨盘电机反馈
        std::uint16_t judgeCnt{};//堵转计数
        bool trig_flag_ready{};//是否已有送弹动作
        bool trig_calibration_flag{};//拨盘是否已校准好
    };

    struct FricMotor//摩擦轮电机的运行时变量，功能和原先的DJImotor fricMotor[6]相近
    {
        inline static constexpr std::int16_t fricSpd1 = 3500;//摩擦轮的目标转速
        inline static constexpr std::int16_t fric_offset1 = 650;//摩擦轮之间的速度补偿值

        std::array<MotorDJI_N::fb_3508_6020_s, 6> fricMotor{};//六摩擦轮反馈
        slope_s fricSlope{};//摩擦轮斜坡
        bool fricOpenFlag{};//摩擦轮是否开启
    };

    struct Shooter_s
    {
        ShooterInput input{};//外部输入，包括之前的rcInfo，vision_enble, vision_receive
        DM4310_Trig dm4310_trig{};//拨盘电机参数
        FricMotor fricMotor{};//摩擦轮电机参数
        int vision_delay_time = 0;
        
    };

    class Shooter
    {
        public:

            void Shooter_Init(void);
            void Task_Shooter_Callback(void);

        private:
        
            void TrigMotor_SetCalibration(void);
            void Shooter_InitPID(void);
            void Shooter_RockerCtrl(void);

            Shooter_s shooter;
    };
}
