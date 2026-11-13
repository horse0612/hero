#include "vision.hpp"
#include "detect.hpp"
extern "C"
{
#include "Vision_api.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
}

extern "C"
{
extern USBD_HandleTypeDef hUsbDeviceHS;
}

namespace
{
std::uint16_t VisionCrc16(const std::uint8_t *data, const std::size_t length)
{
    std::uint16_t crc = 0xFFFF;
    for (std::size_t index = 0; index < length; ++index)
    {
        crc ^= static_cast<std::uint16_t>(data[index]) << 8;
        for (std::uint8_t bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 0x8000) != 0 ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021) : static_cast<std::uint16_t>(crc << 1);
        }
    }
    return crc;
}

void AppendVisionCrc(Vision_Transmit *frame)
{
    frame->checksum = VisionCrc16(reinterpret_cast<const std::uint8_t *>(frame), sizeof(Vision_Transmit) - sizeof(frame->checksum));
}
} 

extern "C"
{
VisionReceive vision_receive{};
Vision_Transmit vision_transmit{};
Vision_Gimbal vision{};
uint8_t VisionTransBuf[20];
uint8_t VisionRxBuf[20];
}

extern "C"
{

void Vision_Init(void)
{
    vision_transmit.header = VISION_FRAME_HEADER_TX;
    vision_transmit.finish = VISION_FRAME_FINISH_TX;
}

void Vision_ParseData(void)
{
    vision.pitch = vision_receive.pitch;
    vision.yaw = -vision_receive.yaw;
    vision.fire = vision_receive.fire ;
    vision.found = vision_receive.looking_in;
}

void Vision_DataReceive(std::uint8_t *read_from_usart, std::uint32_t length)
{
    if(read_from_usart == nullptr)
    {
        return;
    }
    std::uint32_t remaining = length;

    if(read_from_usart[0] == VISION_FRAME_HEADER_RX )
    {
        std::memcpy(&vision_receive, read_from_usart, sizeof(vision_receive));
        Vision_ParseData();
        return;
    }//一个完整，正常的包应该第一帧就是帧头，否则就应该怀疑丢包或者粘包，需要轮询查找帧头与帧尾进行拼接，但这是不合理的通信方式，只能作为临时应急方案

    while (remaining > 0)
    {
        if(*read_from_usart != VISION_FRAME_HEADER_RX)
        {
        ++read_from_usart;
        --remaining;
        }
        else
        {
            break;
        }
    }
    if(remaining == 0)
    {
        return;
    }

}

void Vision_DataUpdate(void)
{
    vision_transmit.roll = User_Imu_GetPitch();
    vision_transmit.yaw = -User_Imu_GetYaw();
    vision_transmit.pitch = -User_Imu_GetRoll();
    AppendVisionCrc(&vision_transmit);
}

void Vision_DataTransmit(void)
{
    Vision_DataUpdate();
    std::memcpy(VisionTransBuf, &vision_transmit, sizeof(vision_transmit));
    (void)CDC_Transmit_HS(VisionTransBuf, static_cast<std::uint16_t>(sizeof(vision_transmit)));
}

void Task_VS_Callback(void)
{
    Vision_DataTransmit();
}



} 
