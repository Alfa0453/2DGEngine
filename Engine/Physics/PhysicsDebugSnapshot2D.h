#pragma once

#include "PhysicsSettings2D.h"
#include "PhysicsStats2D.h"
#include "PhysicsDebugDrawSettings2D.h"

namespace Engine
{
    struct PhysicsDebugSnapshot2D
    {
        PhysicsSettings2D Settings;

        PhysicsStats2D Stats;

        PhysicsDebugDrawSettings2D DrawSettings;
    };
}