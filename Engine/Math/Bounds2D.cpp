#include "Bounds2D.h"

#include <algorithm>

namespace Engine
{
    Bounds2D::Bounds2D(const Vector2& min, const Vector2& max)
        : Min(std::min(min.X, max.X), std::min(min.Y, max.Y)),
          Max(std::max(min.X, max.X), std::max(min.Y, max.Y))
    {
    }

    Bounds2D Bounds2D::FromPositionSize(const Vector2 &position, const Vector2 &size)
    {
        return Bounds2D(position, position + size);
    }

    Vector2 Bounds2D::GetCenter() const
    {
        return (Min + Max) * 0.5f;
    }

    Vector2 Bounds2D::GetSize() const
    {
        return Max - Min;
    }

    float Bounds2D::GetWidth() const
    {
        return Max.X - Min.X;
    }

    float Bounds2D::GetHeight() const
    {
        return Max.Y - Min.Y;
    }

    bool Bounds2D::Contains(const Vector2& point) const
    {
        return point.X >= Min.X &&
               point.X <= Max.X &&
               point.Y >= Min.Y &&
               point.Y <= Max.Y;
    }

    bool Bounds2D::Intersects(const Bounds2D& other) const
    {
        return Min.X <= other.Max.X &&
               Max.X >= other.Min.X &&
               Min.Y <= other.Max.Y &&
               Max.Y >= other.Min.Y;
    }

    void Bounds2D::Encapsulate(const Vector2& point)
    {
        Min.X = std::min(Min.X, point.X);

        Min.Y = std::min(Min.Y, point.Y);

        Max.X = std::max(Max.X, point.X);

        Max.Y = std::max(Max.Y, point.Y);
    }

    void Bounds2D::Encapsulate(const Bounds2D& other)
    {
        Encapsulate(other.Min);

        Encapsulate(other.Max);
    }

    void Bounds2D::Expand(float amount)
    {
        Expand( { amount, amount } );
    }

    void Bounds2D::Expand(const Vector2& amount)
    {
        Min -= amount;
        Max += amount;
    }
}