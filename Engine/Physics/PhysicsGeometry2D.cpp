#include "PhysicsGeometry2D.h"

#include <algorithm>

namespace Engine
{
    Vector2 PhysicsGeometry2D::ClosestPointOnSegment(const Vector2& point, const Vector2& segmentA, const Vector2& segmentB)
    {
        const Vector2 segment = segmentB - segmentA;

        const float segmentLengthSquared = segment.LengthSquared();

        constexpr float epsilon = 0.000001f;

        if (segmentLengthSquared <= epsilon)
        {
            return segmentA;
        }

        float t = Vector2::Dot(point - segmentA, segment) / segmentLengthSquared;

        t = std::clamp(t, 0.0f, 1.0f);

        return segmentA + segment * t;
    }
}