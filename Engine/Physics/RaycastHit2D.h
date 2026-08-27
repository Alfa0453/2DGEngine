#pragma once

#include "../Math/Vector2.h"
#include "../Scene/EntityHandle.h"

namespace Engine
{
    class Collider2D;


    struct RaycastHit2D
    {
        bool Hit = false;

        EntityHandle Entity;

        Collider2D* Collider = nullptr;

        Vector2 Point{0.0f, 0.0f};

        Vector2 Normal{0.0f, 0.0f};

        float Distance = 0.0f;

        float Fraction = 0.0f;
    };
}