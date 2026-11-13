/*
#pragma once

#include <cstdint>
#include "JLU_HERO\Math\inc\Slope.hpp"
#include "JLU_HERO\UserMiddlewares\inc\Moto_ctr.hpp"
#define Trig_Start_Angle 1.26282001f//-0.09f
#ifndef TRIG_MOTOR_MODE
#define TRIG_MOTOR_MODE 1 //0:侧切拨盘 1：中供拨盘 改变的是校准方式
#endif

enum ShooterWorkState : std::uint8_t
{
    IDLE = 0,
    LOAD,
    TRIGGER,
    TRIGGER_REVERSE
};

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
        Remote_Ctrl_N::rc_state_s remote{};
        bool vision_enable = false;
        bool vision_fire = false;
    };

    struct ShooterFeedback//摩擦轮和拨盘的反馈数据
    {
        std::array<MotorDJI_N::fb_3508_6020_s, 6> friction{};//摩擦轮反馈
        MotorDM_N::feedback_s trigger{};//拨盘反馈
        bool trigger_valid = false;
    };

    struct ShooterConfig//摩擦轮和拨盘的配置参数，原先
    {
        float friction_speed = 3500.0F;
        float friction_offset = 650.0F;
        float friction_step = 10.0F;

        float trigger_step = 3.1415926F / 4.0F;

        float Trig_Gear2Moto = 88.0f/40.0f;
        float Trig_Moto2Gear = 40.0f/88.0f;

        uint32_t jam_count_limit = 150U;
    };
    struct Shooter
    {
        ShooterWorkState workState{ShooterWorkState::IDLE};

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
