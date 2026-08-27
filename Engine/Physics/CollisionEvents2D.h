#pragma once

#include "../Scene/EntityHandle.h"

namespace Engine
{
    class Collider2D;

    struct CollisionBeginEvent2D
    {
        EntityHandle A;
        EntityHandle B;

        Collider2D* ColliderA = nullptr;
        Collider2D* ColliderB = nullptr;

        bool IsTrigger = false;
    };

    struct CollisionStayEvent2D
    {
        EntityHandle A;
        EntityHandle B;

        Collider2D* ColliderA = nullptr;
        Collider2D* ColliderB = nullptr;

        bool IsTrigger = false;
    };

    struct CollisionEndEvent2D
    {
        EntityHandle A;
        EntityHandle B;

        Collider2D* ColliderA = nullptr;
        Collider2D* ColliderB = nullptr;

        bool WasTrigger = false;
    };

    struct SweptTriggerEvent2D
    {
        EntityHandle A;
        EntityHandle B;

        Collider2D* ColliderA = nullptr;
        Collider2D* ColliderB = nullptr;

        float TimeOfImpact = 0.0f;
    };
}