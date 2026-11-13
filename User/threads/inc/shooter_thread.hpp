#pragma once

#include "shooter.hpp"

namespace JLUni
{
    struct ShooterThreadArgs
    {
        Shooter& shooter;
    };
    void OS_ShooterCallback(ShooterThreadArgs* const args);
}
