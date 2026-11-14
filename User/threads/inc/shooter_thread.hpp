#pragma once

#include "shooter.hpp"

namespace JLUni
{
    struct ShooterThreadArgs
    {
        Shooter_N::Shooter& shooter;
    };
    void OS_ShooterCallback(ShooterThreadArgs* const args);
}
