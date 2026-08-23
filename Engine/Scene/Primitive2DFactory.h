#pragma once

#include "../Graphics/PrimitiveShape2D.h"
#include "../Math/Vector2.h"

namespace Engine
{
    class Entity;
    class Scene;

    class Primitive2DFactory
    {
    public:
        
        static Entity* Create(Scene& scene, PrimitiveShape2D shape, const Vector2& position, const Vector2& size);

        static const char* GetDefaultName(PrimitiveShape2D shape);
    };
}