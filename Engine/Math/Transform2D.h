#pragma once

#include "Vector2.h"

namespace Engine
{
    struct Transform2D
    {
        Vector2 Position{ 0.0f, 0.0f };
        
        float Rotation = 0.0f;

        Vector2 Scale{ 1.0f, 1.0f };

        Transform2D() = default;

        Transform2D(const Vector2& position, float rotation, const Vector2& scale)
            : Position(position), Rotation(rotation), Scale(scale)
        {
        }

        Vector2 TransformPoint(const Vector2& point) const;

        Vector2 InverseTransformPoint(const Vector2& point) const;

        static Transform2D Combine(const Transform2D& parent, const Transform2D& local);

        static Transform2D InverseCombine(const Transform2D& parent, const Transform2D& world);
    };
}