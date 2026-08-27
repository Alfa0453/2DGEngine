#pragma once

#include <algorithm>

namespace Engine
{
    struct PhysicsMaterial2D
    {
        float Restitution = 0.0f;

        float StaticFriction = 0.6f;

        float DynamicFriction = 0.4f;

        void Clamp()
        {
            Restitution = std::clamp(Restitution, 0.0f, 1.0f);

            StaticFriction = std::max(0.0f, StaticFriction);

            DynamicFriction = std::max(0.0f, DynamicFriction);

            if (DynamicFriction > StaticFriction)
            {
                DynamicFriction = StaticFriction;
            }
        }
    };
}