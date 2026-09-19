#pragma once

#include "../Math/Vector2.h"

namespace Engine
{
    class Collider2D;

    struct PhysicsCCDDebug2D
    {
        Collider2D* MovingCollider = nullptr;

        Collider2D* TargetCollider = nullptr;

        Vector2 Start{0.0f, 0.0f};

        Vector2 End{0.0f, 0.0f};

        Vector2 ImpactPoint{0.0f, 0.0f};

        Vector2 ImpactNormal{0.0f, 0.0f};

        float TimeOfImpact = 0.0f;

        bool Hit = false;
    };
}