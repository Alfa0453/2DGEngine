#pragma once

#include <cstddef>
#include <vector>

namespace Engine
{
    class Rigidbody2D;
    class Joint2D;

    struct PhysicsIsland2D
    {
        std::vector<Rigidbody2D*> Bodies;

        std::vector<std::size_t> ContactIndices;

        std::vector<Joint2D*> Joints;

        void Clear()
        {
            Bodies.clear();

            ContactIndices.clear();

            Joints.clear();
        }

        bool Empty() const
        {
            return Bodies.empty() && ContactIndices.empty() && Joints.empty();
        }
    };
}