#pragma once

#include "Vector2.h"

namespace Engine
{
    struct Rect
    {
        Vector2 Position;
        Vector2 Size;

        Rect() = default;

        Rect(const Vector2& position, const Vector2& size)
            : Position(position),
              Size(size)
        {
        }

        Rect(float x, float y, float width, float height)
            : Position(x, y),
              Size(width, height)
        {
        }

        float Left() const
        {
            return Position.X;
        }

        float Right() const
        {
            return Position.X + Size.X;
        }

        float Top() const
        {
            return Position.Y;
        }

        float Bottom() const
        {
            return Position.Y + Size.Y;
        }

        Vector2 Center() const
        {
            return 
            {
                Position.X + Size.X * 0.5f,
                Position.Y + Size.Y * 0.5f
            };
        }
    };
}