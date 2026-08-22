#pragma once

#include <cstddef>
#include <functional>

namespace Engine
{
    class Collider2D;

    struct ColliderPair2D
    {
        Collider2D* A = nullptr;

        Collider2D* B = nullptr;

        static ColliderPair2D Make(Collider2D* a, Collider2D* b)
        {
            if (a < b)
            {
                return{a, b};
            }

            return{b, a};
        }

        bool operator==(const ColliderPair2D& other) const
        {
            return A == other.A && B == other.B;
        }
    };

    struct ColliderPair2DHash
    {
        std::size_t operator()(const ColliderPair2D& pair) const
        {
            const std::size_t h1 = std::hash<Collider2D*>{}(pair.A);

            const std::size_t h2 = std::hash<Collider2D*>{}(pair.B);

            return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
        }
    };
}