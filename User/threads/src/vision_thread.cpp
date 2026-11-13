#include "vision_thread.hpp"
#include "vision.hpp"

namespace JLUni
{
    void OS_VisionCallback(VisionThreadArgs* const args)
    {
        while (App_IsInitialized() == 0)
        {
            LibXR::Thread::Sleep(1);
        }
        for (;;)
        {
            args->vision.Update();
            Task_VS_Callback();
            LibXR::Thread::Sleep(1);
        }
    }
}
