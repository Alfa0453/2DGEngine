#pragma once

#include <cstddef>
#include <vector>

namespace Engine
{
    class Rigidbody2D;

    struct PhysicsIsland2D
    {
        std::vector<Rigidbody2D*> Bodies;

        std::vector<std::size_t> ContactIndices;

        void Clear()
        {
            Bodies.clear();

            ContactIndices.clear();
        }

        bool Empty() const
        {
            return Bodies.empty() && ContactIndices.empty();
        }
    };
}