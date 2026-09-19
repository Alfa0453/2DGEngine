#pragma once

#include "../Math/Vector2.h"

#include <vector>

namespace Engine
{
    struct Polygon2D
    {
        std::vector<Vector2> Vertices;

        std::vector<Vector2> Normals;
    };
}
