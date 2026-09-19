#pragma once

#include "../../Engine/Math/Vector2.h"

namespace Engine
{
    class Scene;
    class Entity;
    class Rigidbody2D;
    class BoxCollider2D;
    class CircleCollider2D;
}

namespace Tests
{
    Engine::Entity* CreateStaticBox(Engine::Scene& scene, const Engine::Vector2& position, const Engine::Vector2& size);

    Engine::Entity* CreateDynamicBox(Engine::Scene& scene, const Engine::Vector2& position, const Engine::Vector2& size);

    Engine::Entity* CreateDynamicCircle(Engine::Scene& scene, const Engine::Vector2& position, float radius);
}