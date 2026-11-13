#include "remote_ctrl_thread.hpp"

namespace JLUni::Remote_Ctrl_N
{
    void RemoteCtrlThread(RemoteCtrlThreadArgs* args)
    {
        RemoteCtrl::Frame frame{};
        LibXR::Semaphore semaphore;
        LibXR::ReadOperation operation(semaphore);
        for (;;)
        {
            if (args->remote.Read(frame, operation) != LibXR::ErrorCode::OK) continue;
            args->remote.Parse(frame);
            Chassis_N::chassis_cmd_s command{};
            if (args->remote.GetPolicy() == policy_e::chassis)
            {
                const auto& state = args->remote.State();
                command.vx = 3.F * state.lever[3];
                command.vy = -3.F * state.lever[2];
                command.wz = -7.F * state.lever[0];
            }
            static_cast<void>(args->commands.Publish(command));
        }
    }
}
