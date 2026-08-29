#pragma once

#include "../Math/Vector2.h"

namespace Engine
{
    struct CachedContact2D
    {
        Vector2 Point{0.0f, 0.0f};

        float NormalImpulse = 0.0f;

        float TangentImpulse = 0.0f;
    };
}