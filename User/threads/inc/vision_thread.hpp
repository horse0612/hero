#pragma once

namespace JLUni
{
    struct VisionThreadArgs
    {
        JLUni::Vision_N::Vision& vision;
        LibXR::Topic* imu_topic;
    };
    void OS_VisionCallback(VisionThreadArgs* const args);
}
