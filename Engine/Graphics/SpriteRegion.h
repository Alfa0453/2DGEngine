#pragma once

#include "../Math/Rect.h"

#include <cstdint>

namespace Engine
{
    struct SpriteRegion
    {
        Rect SourceRect;

        SpriteRegion() = default;

        explicit SpriteRegion(const Rect& sourceRect)
            : SourceRect(sourceRect)
        {
        }

        SpriteRegion(const Vector2& position, const Vector2& size)
            : SourceRect(position, size)
        {
        }

        const Vector2& GetPosition() const
        {
            return SourceRect.Position;
        }

        const Vector2& GetSize() const
        {
            return SourceRect.Size;
        }

        static SpriteRegion FromGrid(std::int32_t column, std::int32_t row, const Vector2& cellSize)
        {
            return SpriteRegion(
                {
                    static_cast<float>(column) * cellSize.X,

                    static_cast<float>(row) * cellSize.Y
                },

                cellSize
            );
        }
    };
}