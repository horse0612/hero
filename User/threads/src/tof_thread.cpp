#include "tof_thread.hpp"
#include <algorithm>
#include <array>

namespace JLUni
{
    void TOFThread(TOFThreadArgs* args)
    {
        using namespace TOF_N;
        LibXR::Semaphore semaphore;
        LibXR::ReadOperation operation(semaphore);
        std::array<uint8_t, 9> buffer{};
        uint8_t size = 0;
        for (;;)
        {
            // Wait for new bytes, not for the retained partial frame to complete.
            if (args->uart.Read({nullptr, 0}, operation) != LibXR::ErrorCode::OK) continue;
            for (;;)
            {
                const uint8_t expected = TFminiPlus::ExpectFrameSize({buffer.data(), size});
                if (expected != 0 && size < expected)
                {
                    const auto count = static_cast<uint8_t>(std::min<size_t>(expected - size, args->uart.read_port_->Size()));
                    if (count == 0) break;
                    if (args->uart.Read({buffer.data() + size, count}, operation)
                        != LibXR::ErrorCode::OK)
                    {
                        break;
                    }
                    size += count;
                    continue;
                }
                uint8_t consumed = 1;
                if (expected != 0)
                {
                    measurement_s measurement{};
                    const auto result = args->tof.UpdateFrame({buffer.data(), expected}, measurement);
                    if (result == update_result_e::MEASUREMENT)
                    {
                        static_cast<void>(args->measurements.Publish(measurement));
                    }
                    if (result != update_result_e::NONE) consumed = expected;
                }
                // Preserve a possible next header after a bad candidate.
                for (uint8_t i = consumed; i < size; ++i) buffer[i - consumed] = buffer[i];
                size -= consumed;
            }
        }
    }
}
