#pragma once

#include "Collider2D.h"
#include <unordered_set>
#include <vector>

namespace Engine
{
    class Collider2D;


    struct PhysicsQueryContext2D
    {
        std::vector<Collider2D*> Candidates;

        std::unordered_set<Collider2D*> Unique;

        void Reset()
        {
            Candidates.clear();

            Unique.clear();
        }
    };
}