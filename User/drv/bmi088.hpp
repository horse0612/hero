#pragma once
#include "gpio.hpp"
#include "imu_cfg.hpp"
#include "libxr.hpp"
#include "spi.hpp"
#include <array>
#include <cstdint>

namespace JLUni
{
#if IMU_CALIBRATION_MODE != 0
    struct RawImuSample
    {
        uint64_t timestamp_us{};
        uint32_t sequence{};
        std::array<int16_t, 3> gyro{}, accel{};
        int16_t temperature_raw{};
    };

#else
    struct ImuFrame
    {
        std::array<float, 3> gyro_rad_s{}, accel_m_s2{};
    };

#endif
    class BMI088
    {
    public:
        BMI088(LibXR::SPI&, LibXR::GPIO&, LibXR::GPIO&, LibXR::GPIO&,
               LibXR::GPIO&
#if IMU_CALIBRATION_MODE == 0
               , const std::array<float, 3>& gyro_bias
#endif
               );

#if IMU_CALIBRATION_MODE == 0
        LibXR::Topic& GetSampleTopic() { return sample_topic_; }
#else
        bool ReadRaw(RawImuSample& sample)
        {
            return raw_queue_.Pop(sample) == LibXR::ErrorCode::OK;
        }
#endif

    private:
        enum class Transfer : uint8_t { NONE, ACCEL, GYRO };

        void InitAccel() const;
        void InitGyro() const;
        bool ReadAccelRegister(uint8_t, uint8_t&) const;
        bool ReadGyroRegister(uint8_t, uint8_t&) const;
        bool WriteAccelRegister(uint8_t, uint8_t) const;
        bool WriteGyroRegister(uint8_t, uint8_t) const;
        bool TryTransfer(bool, LibXR::MicrosecondTimestamp);
        void PublishCompletedTransfer(bool);
        void Deselect(Transfer) const;
        static void OnDataReady(bool, BMI088*);
        static void OnTransferComplete(bool, BMI088*, LibXR::ErrorCode);
        LibXR::SPI& spi_;
        LibXR::GPIO& accel_cs_;
        LibXR::GPIO& gyro_cs_;
#if IMU_CALIBRATION_MODE == 0
        const std::array<float, 3> gyro_bias_;
        LibXR::Topic sample_topic_{LibXR::Topic::CreateTopic<ImuFrame>("/imu/bmi088/sample")};
#else
        LibXR::SPSCQueue<RawImuSample> raw_queue_{64U};
#endif
        LibXR::WriteOperation::Callback transfer_callback_;
        std::array<uint8_t, 19> rx_frame_{};
        std::array<int16_t, 3> gyro_raw_{};
        bool gyro_frame_ready_ = false;
#if IMU_CALIBRATION_MODE != 0
        uint32_t sequence_ = 0, active_sequence_ = 0;
        bool active_temperature_read_ = false;
        uint8_t data_ready_count_ = 0;
        int16_t temperature_raw_ = INT16_MIN;
#endif
        uint16_t transfer_errors_ = 0;
        Transfer active_transfer_ = Transfer::NONE;
        uint8_t tx_command_ = 0;
        LibXR::MicrosecondTimestamp active_timestamp_{};
    };
} // namespace JLUni
