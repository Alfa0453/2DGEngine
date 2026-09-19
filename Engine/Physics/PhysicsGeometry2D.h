#pragma once

#include "../Math/Vector2.h"

namespace Engine
{
    class PhysicsGeometry2D
    {
    public:
    
        static Vector2 ClosestPointOnSegment(const Vector2& point, const Vector2& segmentA, const Vector2& segmentB);
    };
}