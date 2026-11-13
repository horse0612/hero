#pragma once
#include <cstdio>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>
#include "shooter.hpp"
#include "gimbal.hpp"
#include "ins_task.hpp"
#include "libxr.hpp"
extern "C"
{
#include "main.h"
#include "app_main.h"
}

inline std::uint8_t VISION_FRAME_HEADER_TX = 0x5A;
inline std::uint8_t VISION_FRAME_FINISH_TX = 0x6A;

inline std::uint8_t VISION_FRAME_HEADER_RX = 0xA5;
inline std::uint8_t VISION_FRAME_FINISH_RX = 0xA6;

#pragma pack(push, 1)
struct VisionReceive
{
    std::uint8_t header{};
    float yaw{};
    float pitch{};
    std::uint8_t fire{};
    std::uint8_t looking_in : 1;
    std::uint8_t finish{};
    std::uint16_t checksum{};
};

struct Vision_Transmit
{
    std::uint8_t header{};
    float roll{};
    float pitch{};
    float yaw{};
    std::uint8_t finish{};
    std::uint16_t checksum{};
};

struct Vision_Gimbal
{
    float yaw{};
    float pitch{};
    std::uint8_t fire{};
    std::uint8_t found{};
};
#pragma pack(pop)

extern "C"
{
extern VisionReceive vision_receive;
extern Vision_Transmit vision_transmit;
extern Vision_Gimbal vision;
void Vision_Init(void);
void Vision_ParseData(void);
void Vision_DataReceive(std::uint8_t *read_from_usart, std::uint32_t length);
void Vision_DataUpdate(void);
void Vision_LLM_Update(void);
void Vision_DataTransmit(void);
void Task_VS_Callback(void);
void OS_VisionCallback(void const *argument);
}