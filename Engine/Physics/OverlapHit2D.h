#pragma once

#include "../Scene/EntityHandle.h"

namespace Engine
{
    class Collider2D;


    struct OverlapHit2D
    {
        EntityHandle Entity;

        Collider2D* Collider = nullptr;
    };
}