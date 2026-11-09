#pragma once

#include "chassis_command.hpp"
#include "last_buffer.hpp"
#include "remote_ctrl.hpp"

namespace JLUni::Remote_Ctrl_N
{
    struct RemoteCtrlThreadArgs
    {
        RemoteCtrl& remote;
        Algo::LastBuffer<Chassis_N::chassis_cmd_s>& commands;
    };

    void RemoteCtrlThread(RemoteCtrlThreadArgs* args);
}
