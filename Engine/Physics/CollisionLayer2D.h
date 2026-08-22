#pragma once

#include <cstdint>

namespace Engine
{
    using CollisionLayerMask2D = std::uint32_t;

    namespace CollisionLayer2D
    {
        inline constexpr CollisionLayerMask2D None = 0;

        inline constexpr CollisionLayerMask2D Default = 1u << 0;

        inline constexpr CollisionLayerMask2D Player = 1u << 1;

        inline constexpr CollisionLayerMask2D Enemy = 1u << 2;

        inline constexpr CollisionLayerMask2D World = 1u << 3;

        inline constexpr CollisionLayerMask2D Pickup = 1u << 4;

        inline constexpr CollisionLayerMask2D Trigger = 1u << 5;

        inline constexpr CollisionLayerMask2D All = 0xFFFFFFFFu;
    }
}