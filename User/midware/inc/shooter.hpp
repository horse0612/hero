/*



struct Shooter
{
    std::uint8_t workState{};
    bool rockerCtrl{};
    bool vision_enble{};
    DJImotor fricMotor[6]{};
    DMmotor dm4310_trig{};
    bool fricOpenFlag{};
    std::int16_t fricSpd1{};
    std::int16_t fric_offset1{};
    std::int16_t fricSpdOffset{};
    Slope fricSlope{};

    struct
    {
        std::uint16_t judgeCnt{};
    } block;
};

#define Trig_Gear2Moto 88.0f/40.0f
#define Trig_Moto2Gear 40.0f/88.0f

namespace
{
constexpr float kPi = LibXR::PI;
}

extern "C"
{
extern Shooter shooter;
void Shooter_Init(void);
void Shooter_InitPID(void);
void Shooter_RockerCtrl(void);
void Task_Shooter_Callback(void);
void TrigMotor_SetCalibration(void);
void OS_ShooterCallback(void const *argument);
}

*/


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

#define Trig_Start_Angle 1.26282001f//-0.09f
#ifndef TRIG_MOTOR_MODE
#define TRIG_MOTOR_MODE 1 //0:侧切拨盘 1：中供拨盘 改变的是校准方式
#endif

namespace
{
constexpr float kPi = LibXR::PI;
}

namespace JLUni::Shooter_N
{   
    enum class WorkState : std::uint8_t//枚举三种送弹状态
    {
        IDLE = 0,
        TRIGGER,
        TRIGGER_REVERSE
    };

    struct ShooterInput//云台从外界获取的输入，遥控器和视觉
    {
        Remote_Ctrl_N::rc_state_s remote{};//遥控器输入，包含了以前的rockerCtrl、rcInfo、`vision_receive`
        bool vision_enable = false;
        bool vision_fire = false;
    };

    struct ShooterFeedback//摩擦轮和拨盘的反馈数据
    {
        std::array<MotorDJI_N::fb_3508_6020_s, 6> fricMotor{};//六个摩擦轮反馈，对应原先的DJImotor fricMotor[6]
        MotorDM_N::feedback_s trigger{};//拨盘反馈
        bool trigger_valid = false;
    };

    struct ShooterConfig//摩擦轮和拨盘的配置参数，原先分散在.cpp和.hpp
    {
        float fricSpd1 = 3500.0F;
        float fric_offset1 = 650.0F;
        float friction_step = 10.0F;

        float trigger_step = 3.1415926F / 4.0F;

        float Trig_Gear2Moto = 88.0f/40.0f;
        float Trig_Moto2Gear = 40.0f/88.0f;

        uint32_t jam_count_limit = 150U;
//下面是原先的配置参数
    #if (TRIG_MOTOR_MODE == 0)//选择不同的拨盘, 0:侧切拨盘 1：中供拨盘
        constexpr float kTriggerStep = kPi / 3.0f;
    #elif (TRIG_MOTOR_MODE == 1)
        constexpr float kTriggerStep = kPi / 4.0f;
        constexpr float kTriggerOffset = - 4.04f;//测出来校准后的其中一个槽位
        //后面有新的结构，应该是用输出力矩的突变来校准，这个暂时不需要
        constexpr float kTriggerStarts[8] = 
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
    int last_shoot_mode = IDLE;
    bool trig_flag_ready = false;
    int vision_delay_time = 0;
    bool trig_calibration_flag = false;
    shooter.rockerCtrl = false;
        //shooter.fricSpd1 = 3500;
        shooter.fric_offset1 = 650;
        Slope_Init(&shooter.fricSlope, 10.0f, 0.0f);
        Slope_SetTarget(&shooter.fricSlope, 0.0f);
        shooter.fricOpenFlag = false;
        shooter.workState = IDLE;
        shooter.block.judgeCnt = 0U;
        vision_delay_time = 0;
        last_shoot_mode = IDLE;
    };
    struct Shooter
    {
        //std::uint8_t workState{};
        //bool rockerCtrl{};
       // bool vision_enble{};
        //DJImotor fricMotor[6]{};
        DMmotor dm4310_trig{};
        bool fricOpenFlag{};
        std::int16_t fricSpd1{};
        std::int16_t fric_offset1{};
        std::int16_t fricSpdOffset{};
        Slope fricSlope{};

        struct
        {
            std::uint16_t judgeCnt{};
        } block;
    };
/////////
    struct Shooter
    {
        WorkState workState{WorkState::IDLE};//当前送弹状态
        ShooterInput input{};//外部输入，包括rockerCtrl，vision_enble
        ShooterFeedback shooter_feedback{};//电机反馈
        ShooterConfig config{};//参数配置
        std::array<int16_t, 6> friction_targets{};//运行时变量
        int16_t trigger_target_raw = 0;
        uint32_t jam_count = 0;
        bool friction_enabled = false;
        bool trigger_ready = false;

    }

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
        void Shooter_InitPID(void);

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
