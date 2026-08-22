#pragma once

#include "Vector2.h"

namespace Engine
{
    struct Bounds2D
    {
        Vector2 Min{ 0.0f, 0.0f };

        Vector2 Max{ 0.0f, 0.0f };

        Bounds2D() = default;

        Bounds2D(const Vector2& min, const Vector2& max);

        static Bounds2D FromPositionSize(const Vector2& position, const Vector2& size);

        Vector2 GetCenter() const;

        Vector2 GetSize() const;

        float GetWidth() const;

        float GetHeight() const;

        bool Contains(const Vector2& point) const;

        bool Intersects(const Bounds2D& other) const;

        void Encapsulate(const Vector2& point);

        void Encapsulate(const Bounds2D& other);

        void Expand(float amount);

        void Expand(const Vector2& amount);
    };
}