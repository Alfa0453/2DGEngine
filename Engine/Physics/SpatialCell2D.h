#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

namespace Engine
{
    struct SpatialCell2D
    {
        std::int32_t X = 0;
        std::int32_t Y = 0;

        bool operator==(const SpatialCell2D& other) const
        {
            return X == other.X && Y == other.Y;
        }
    };

    struct SpatialCell2DHash
    {
        std::size_t operator()(const SpatialCell2D& cell) const
        {
            const std::size_t hx = std::hash<std::int32_t>{}(cell.X);

            const std::size_t hy = std::hash<std::int32_t>{}(cell.Y);

            return hx ^ (hy + 0x9e3779b9 + (hx << 6) + (hx >> 2));
        }
    };
}