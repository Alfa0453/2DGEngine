#pragma once

#include <cstddef>

namespace Engine
{
    struct PhysicsDebugStatus2D
    {
        std::size_t ActiveColliders = 0;

        std::size_t SpatialCells = 0;

        std::size_t CandidatePairs = 0;

        std::size_t Contacts = 0;

        std::size_t SleepingBodies = 0;
    };
}