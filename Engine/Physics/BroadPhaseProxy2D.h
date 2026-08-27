#pragma once

#include <cstdint>
#include <vector>

#include "../Math/Bounds2D.h"
#include "SpatialCell2D.h"

namespace Engine
{
    class Collider2D;


    struct BroadPhaseProxy2D
    {
        Collider2D* Collider = nullptr;

        Bounds2D Bounds;

        std::vector<SpatialCell2D> Cells;

        std::uint64_t TransformWorldVersion = 0;

        std::uint64_t ColliderBoundsRevision = 0;
    };
}