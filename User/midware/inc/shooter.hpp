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
#define TRIG_MOTOR_MODE 1 //0:侧切拨盘 1：中供拨盘 改变的是校准方式
#endif

namespace JLUni::Shooter_N
{   
    constexpr float kPi = LibXR::PI;
    enum class WorkState : std::uint8_t//枚举三种送弹状态
    {
        IDLE = 0,
        TRIGGER,
        TRIGGER_REVERSE
    };

    struct ShooterInput//云台从外界获取的输入，遥控器和视觉
    {
        Remote_Ctrl_N::rc_state_s remote{};//遥控器输入，对应rcInfo
        bool vision_enable  = false;//视觉功能是否使能，以前的vision_enble
        bool vision_receive = false;//视觉开火信号
    };

    struct ShooterFeedback//摩擦轮和拨盘的反馈数据
    {
        std::array<MotorDJI_N::fb_3508_6020_s, 6> fricMotor{};//六摩擦轮反馈，对应原先的DJImotor fricMotor[6]
        MotorDM_N::feedback_s dm4310_trig{};//拨盘电机反馈，对应原先的DMmotor dm4310_trig{};
        bool trigger_valid = false;
    };

    struct ShooterConfig//摩擦轮和拨盘的固定配置参数，原先分散在.cpp和.hpp
    {
        float Trig_Start_Angle = 1.26282001f;//-0.09f
        #if     (TRIG_MOTOR_MODE == 0)//选择不同的拨盘, 0:侧切拨盘 1：中供拨盘
        constexpr float kTriggerStep = kPi / 3.0f;
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

        inline static constexpr float Trig_Gear2Moto = 88.0f/40.0f;
        inline static constexpr float Trig_Moto2Gear = 40.0f/88.0f;

    };

    struct Shooter_s
    {
        WorkState workState{WorkState::IDLE};//当前送弹状态
        WorkState last_shoot_mode{WorkState::IDLE};//上次送弹状态
        ShooterInput input{};//外部输入，包括之前的rcInfo，vision_enble, vision_receive
        ShooterFeedback shooter_feedback{};//电机反馈
        std::int16_t fricSpd1{};//摩擦轮目标速度
        std::int16_t fric_offset1{};//摩擦轮速度补偿
        std::int16_t fricSpdOffset{};//摩擦轮动态速度偏移
        ShooterConfig config{};//参数配置
        std::array<int16_t, 6> friction_targets{};//运行时变量
        int16_t trigger_target_raw = 0;
        bool rockerCtrl{};//是否由遥控器控制
        bool fricOpenFlag{};//摩擦轮是否开启
        bool trigger_ready{};//拨盘是否准备好
        std::uint16_t judgeCnt{};//堵转计数
        slope_s fricSlope{};//摩擦轮斜坡
    };

    class Shooter
    {
    public:
        Shooter(LibXR::FDCAN& can, const MotorDM_N::scale_fac_s& trigger_scale, const ShooterConfig& config);

        void Shooter_Init(void);
        void Shooter_InitPID(void);
        void Shooter_RockerCtrl(void);
        void Task_Shooter_Callback(void);
        void TrigMotor_SetCalibration(void);

        WorkState GetWorkState() 
        {
            return work_state_;
        }

    private:

        static void BuildFrictionFrame(const std::array<int16_t, 6>& commands, std::array<uint8_t, 8>& frame);

        LibXR::FDCAN& can_;
        const ShooterConfig config_;

        WorkState work_state_ = WorkState::IDLE;

        ShooterInput input_{};
        ShooterFeedback feedback_{};

        slope_s friction_slope_{};

        std::array<int16_t, 6> friction_targets_{};
        int16_t trigger_target_raw_ = 0;

        uint32_t jam_count_ = 0;
        bool friction_enabled_ = false;
        bool trigger_ready_ = false;

        Algo::pid_diff_ahead_s trigger_pid_;
    };
}
