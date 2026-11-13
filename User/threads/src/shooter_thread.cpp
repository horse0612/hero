#include "shooter_thread.hpp"
#include "shooter.hpp"

namespace JLUni
{
    void OS_ShooterCallback(ShooterThreadArgs* const args)
    {
        while (App_IsInitialized() == 0)
        {
            LibXR::Thread::Sleep(1);
        }
        for (;;)
        {
            args->shooter.Update();
            CAN_RefreshShooterFeedback();
            Task_Shooter_Callback();

            LibXR::Thread::Sleep(1);
        }
    }
}