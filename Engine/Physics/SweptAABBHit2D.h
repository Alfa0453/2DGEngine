#pragma once

#include "../Math/Vector2.h"

namespace Engine
{
    struct SweptAABBHit2D
    {
        bool Hit = false;

        float Time = 1.0f;

        Vector2 Normal{0.0f, 0.0f};
    };
}