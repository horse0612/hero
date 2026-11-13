#include "imu_cfg.hpp"
#if IMU_CALIBRATION_MODE != 0
#include "imu_calib_thread.hpp"
#if IMU_CALIBRATION_MODE == 1
#include "flash_map.hpp"
#endif

#include <array>
#include <cstdint>
#if IMU_CALIBRATION_MODE == 2
#include "main.h"
#include <cmath>
#include <cstring>
#endif

namespace JLUni
{
#if IMU_CALIBRATION_MODE == 2
    namespace
    {
        constexpr unsigned rtt_terminal_size = 512U;
        constexpr unsigned rtt_sample_size = 4096U;
        char rtt_terminal[rtt_terminal_size] __attribute__((section(".ram_d2")));
        char rtt_samples[rtt_sample_size] __attribute__((section(".ram_d2")));
        struct RttBuffer
        {
            const char* name;
            char* buffer;
            uint32_t size;
            volatile uint32_t write, read;
            uint32_t flags;
        };

        struct RttControl
        {
            char id[16];
            int32_t up_count, down_count;
            RttBuffer up[2];
        };

        extern "C" {
        RttControl _SEGGER_RTT __attribute__((section(".ram_d2"),aligned(32)));
        }

        void RttInit()
        {
            std::memset(&_SEGGER_RTT, 0, sizeof(_SEGGER_RTT));
            _SEGGER_RTT.up_count = 2;
            _SEGGER_RTT.up[1] = {"IMU_RAW", rtt_samples, rtt_sample_size, 0, 0, 0};
            _SEGGER_RTT.up[0] = {"Terminal", rtt_terminal, rtt_terminal_size, 0, 0, 0};
            __DMB();
            std::memcpy(_SEGGER_RTT.id, "SEGGER RTT", 11);
            __DMB();
        }

        uint32_t RttWrite(unsigned channel, const void* data, unsigned size)
        {
            if (!data || channel >= static_cast<unsigned>(_SEGGER_RTT.up_count))return 0;
            auto& b = _SEGGER_RTT.up[channel];
            unsigned write = b.write;
            const unsigned read = b.read;
            if (size > (read + b.size - write - 1U) % b.size)return 0;
            for (unsigned i = 0; i < size; ++i)
            {
                b.buffer[write] = static_cast<const char*>(data)[i];
                write = (write + 1U) % b.size;
            }
            __DMB();
            b.write = write;
            return size;
        }
    } // namespace
#endif
    namespace Calibration
    {
        inline constexpr uint64_t wait_us = 5000000, collect_us = 120000000;
        inline constexpr uint32_t window_size = 2048, poses = 16;
        inline constexpr float gyro_scale = 2000.F * 3.14159265358979323846F / (180.F * 32768.F);

        enum class State : uint8_t { Wait, Unstable, Collect, Next, Complete, Fault };

#if IMU_CALIBRATION_MODE == 1 || defined(IMU_HOST_TEST)
        struct gyro_calib_s
        {
            State state = State::Wait;
            bool started = false;
            uint64_t start = 0;
            uint32_t count = 0;
            std::array<int64_t, 3> sum{};
            std::array<float, 3> bias{};

            void Update(const RawImuSample& s)
            {
                if (state == State::Complete || state == State::Fault)
                    return;
                if (!started)
                {
                    started = true;
                    start = s.timestamp_us;
                }
                if (s.timestamp_us < start)
                {
                    state = State::Fault;
                    return;
                }
                auto elapsed = s.timestamp_us - start;
                if (elapsed < wait_us)
                    return;
                if (elapsed < wait_us + collect_us)
                {
                    state = State::Collect;
                    for (unsigned i = 0; i < 3; ++i)
                        sum[i] += s.gyro[i];
                    ++count;
                    return;
                }
                if (!count)
                {
                    state = State::Fault;
                    return;
                }
                for (unsigned i = 0; i < 3; ++i)
                    bias[i] = float(double(sum[i]) / count) * gyro_scale;
                state = State::Complete;
            }
        };

#endif
#if IMU_CALIBRATION_MODE == 2 || defined(IMU_HOST_TEST)
        struct Accel
        {
            State state = State::Wait;
            bool started = false;
            uint64_t start = 0;
            uint32_t count = 0;
            uint8_t completed = 0;
            std::array<int64_t, 3> sum{};
            std::array<uint64_t, 3> square{};

            void Reject()
            {
                count = 0;
                sum = {};
                square = {};
                state = State::Unstable;
            }

            static bool Stable(const RawImuSample& s)
            {
                constexpr double gravity_raw = 32768. / 24., gyro_limit = 0.06 / gyro_scale;
                double a = 0, g = 0;
                for (unsigned i = 0; i < 3; ++i)
                {
                    a += double(s.accel[i]) * s.accel[i];
                    g += double(s.gyro[i]) * s.gyro[i];
                }
                return a > std::pow(0.89 * gravity_raw, 2) &&
                    a < std::pow(1.11 * gravity_raw, 2) && g < gyro_limit * gyro_limit;
            }

            // Returns true when a full candidate window is ready; caller commits only
            // after logging it.
            bool Update(const RawImuSample& s, bool gap)
            {
                if (state == State::Complete || state == State::Fault)
                    return false;
                if (!started)
                {
                    started = true;
                    start = s.timestamp_us;
                }
                if (s.timestamp_us < start)
                {
                    state = State::Fault;
                    return false;
                }
                if (s.timestamp_us - start < wait_us)
                    return false;
                bool stable = Stable(s);
                if (state == State::Next)
                {
                    if (!stable)
                        Reject();
                    return false;
                }
                if (gap || !stable)
                {
                    Reject();
                    return false;
                }
                state = State::Collect;
                for (unsigned i = 0; i < 3; ++i)
                {
                    int64_t v = s.accel[i];
                    sum[i] += v;
                    square[i] += static_cast<uint64_t>(v * v);
                }
                if (++count < window_size)
                    return false;

                for (unsigned i = 0; i < 3; ++i)
                {
                    const uint64_t n = count;
                    const uint64_t sum_square = static_cast<uint64_t>(sum[i] * sum[i]);
                    if (n * square[i] - sum_square > 400U * n * n)
                    {
                        Reject();
                        return false;
                    }
                }
                return true;
            }

            void Commit()
            {
                ++completed;
                count = 0;
                sum = {};
                square = {};
                state = completed == poses ? State::Complete : State::Next;
            }
        };
#pragma pack(push, 1)
        struct Packet
        {
            uint32_t magic = 0x314D5549;
            uint8_t version = 1, state = 0, window = 0, flags = 0;
            uint32_t sequence = 0;
            uint64_t timestamp_us = 0;
            std::array<int16_t, 3> accel{}, gyro{};
            int16_t temperature = 0;
            uint16_t index = 0;
        };
#pragma pack(pop)
        static_assert(sizeof(Packet) == 36);
#endif
    } // namespace Calibration

    void ImuCalibThread(imu_calib_args_s* args)
    {
#if IMU_CALIBRATION_MODE == 1
        static LibXR::STM32Flash flash(FLASH_REGIONS, FLASH_REGION_NUMBER);
        static LibXR::DatabaseRaw<LibXR::STM32Flash::MIN_WRITE_SIZE> db(flash);
        std::array<float, 3> bias{};
        LibXR::Database::KeyBase key("imu.gyro_bias", LibXR::RawData(bias));
        auto loaded = db.Get(key);
#endif
#if IMU_CALIBRATION_MODE == 2
        RttInit();
        auto log = [](const char* text) { RttWrite(0, text, std::strlen(text)); };
#endif
        Calibration::State shown = Calibration::State::Fault;
#if IMU_CALIBRATION_MODE == 1
        Calibration::gyro_calib_s calibration;
        bool submitted = false;
#else
        Calibration::Accel calibration;
#endif
#if IMU_CALIBRATION_MODE == 2
        uint32_t previous = 0;
#endif
        uint64_t previous_time = 0;
        bool have_previous = false;
        for (;;)
        {
            RawImuSample sample;
            while (args->imu->ReadRaw(sample))
            {
#if IMU_CALIBRATION_MODE == 2
                bool gap = have_previous && (sample.sequence != previous + 1 ||
                    sample.timestamp_us <= previous_time ||
                    sample.timestamp_us - previous_time > 2000);
                previous = sample.sequence;
#endif
                const bool backwards =
                    have_previous && sample.timestamp_us <= previous_time;
                previous_time = sample.timestamp_us;
                have_previous = true;
#if IMU_CALIBRATION_MODE == 1
                if (backwards)
                    calibration.state = Calibration::State::Fault;
                calibration.Update(sample);
                if (calibration.state == Calibration::State::Complete && !submitted)
                {
                    submitted = true;
                    bias = calibration.bias;
                    auto status = loaded == LibXR::ErrorCode::NOT_FOUND
                                      ? db.Add(key)
                                      : db.Set(key, key.raw_data_);
                    auto expected = bias;
                    if (status != LibXR::ErrorCode::OK ||
                        db.Get(key) != LibXR::ErrorCode::OK || bias != expected)
                        calibration.state = Calibration::State::Fault;
                }
#else
                if (backwards)
                    calibration.state = Calibration::State::Fault;
                bool ready = calibration.Update(sample, gap);
                if (calibration.state == Calibration::State::Collect)
                {
                    Calibration::Packet packet;
                    packet.state = uint8_t(calibration.state);
                    packet.window = calibration.completed;
                    packet.flags = ready ? 1 : 0;
                    packet.sequence = sample.sequence;
                    packet.timestamp_us = sample.timestamp_us;
                    packet.accel = sample.accel;
                    packet.gyro = sample.gyro;
                    packet.temperature = sample.temperature_raw;
                    packet.index = calibration.count;
                    if (RttWrite(1, &packet, sizeof(packet)) != sizeof(packet))
                        calibration.Reject();
                    else if (ready)
                        calibration.Commit();
                }
#endif
                if (shown != calibration.state)
                {
                    shown = calibration.state;
                    switch (shown)
                    {
                    case Calibration::State::Wait:
                    case Calibration::State::Next:
                        args->led->SetRGB(16, 16, 0);
                        break;
                    case Calibration::State::Collect:
                        args->led->SetRGB(0, 0, 32);
                        break;
                    case Calibration::State::Complete:
                        args->led->SetRGB(0, 32, 0);
#if IMU_CALIBRATION_MODE == 2
                        log("calibration complete\n");
#endif
                        break;
                    case Calibration::State::Unstable:
                        args->led->SetRGB(32, 0, 0);
                        break;
                    case Calibration::State::Fault:
                        args->led->SetRGB(32, 0, 0);
#if IMU_CALIBRATION_MODE == 2
                        log("calibration failed\n");
#endif
                        break;
                    }
                }
            }
            LibXR::Thread::Sleep(1);
        }
    }
} // namespace JLUni

#endif
