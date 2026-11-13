#pragma once
#include "tof.hpp"
#include "last_buffer.hpp"

namespace JLUni
{
    struct TOFThreadArgs
    {
        LibXR::UART& uart; // Same UART as the driver; this thread is its sole reader.
        TOF_N::TFminiPlus& tof;
        Algo::LastBuffer<TOF_N::measurement_s>& measurements;
    };
    void TOFThread(TOFThreadArgs* args);
}
