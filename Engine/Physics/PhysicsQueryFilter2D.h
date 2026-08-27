#pragma once

#include "CollisionLayer2D.h"

namespace Engine
{
    class Collider2D;

    struct PhysicsQueryFilter2D
    {
        CollisionLayerMask2D LayerMask = 0xFFFFFFFFu;

        bool IncludeTriggers = true;

        const Collider2D* Ignore = nullptr;
    };
}