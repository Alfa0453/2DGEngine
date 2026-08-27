#pragma once

#include "../Math/Vector2.h"

namespace Engine
{
    struct OrientedBox2D
    {
        // World-space center.
        Vector2 Center{0.0f, 0.0f};

        // Half width / half height
        // BEFORE orientation is applied.
        Vector2 HalfExtents{0.0f, 0.0f};

        // Unit X axis of the rotated Box.
        Vector2 AxisX{1.0f, 0.0f};

        // Unit Y axis of the rotated Box.
        Vector2 AxisY{0.0f, 1.0f};

        // World-space Box corners.
        //
        // 0 = top-left
        // 1 = top-right
        // 2 = bottom-right
        // 3 = bottom-left
        Vector2 Vertices[4]
        {
            Vector2{0.0f, 0.0f},
            Vector2{0.0f, 0.0f},
            Vector2{0.0f, 0.0f},
            Vector2{0.0f, 0.0f}
        };
    };
}