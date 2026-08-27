#include "PhysicsDebugRenderer2D.h"

#include "OrientedBox2D.h"
#include "PhysicsWorld2D.h"
#include "Collider2D.h"
#include "BoxCollider2D.h"
#include "CircleCollider2D.h"
#include "Rigidbody2D.h"
#include "CollisionManifold2D.h"

#include "../Scene/Scene.h"
#include "../Scene/Entity.h"

#include "../Graphics/Renderer2D.h"

#include "../Math/Bounds2D.h"
#include "../Math/Rect.h"
#include "../Math/Color.h"
#include "SpatialCell2D.h"

#include <cmath>

namespace
{
    Engine::Color GetColliderDebugColor(const Engine::Collider2D& collider, bool drawSleepingState)
    {
        if (collider.IsTrigger())
        {
            return Engine::Color::Yellow();
        }
        
        if (drawSleepingState)
        {
            Engine::Entity* owner = collider.GetOwner();

            if (owner)
            {
                Engine::Rigidbody2D* body = owner->GetComponent<Engine::Rigidbody2D>();

                if (body && body->IsSleeping())
                {
                    return Engine::Color::Blue();
                }
            }
        }

        return Engine::Color::Green();
    }
}

namespace Engine
{
    void PhysicsDebugRenderer2D::Draw(const Scene& scene, Renderer2D& renderer) const
    {
        const PhysicsWorld2D& world = scene.GetPhysicsWorld();

        // Spatial grid first

        if (m_DrawSpatialGrid)
        {
            DrawSpatialGrid(world, renderer);
        }

        // Colliders / AABBs

        if (m_DrawColliders || m_DrawAABBs)
        {
            for (Collider2D* collider : world.GetactiveColliders())
            {
                if (!collider)
                {
                    continue;
                }

                if (m_DrawColliders)
                {
                    DrawCollider(*collider, renderer);
                }

                if (m_DrawAABBs)
                {
                    DrawAABB(*collider, renderer);
                }
            }
        }

        // Contacts last

        if (m_DrawContacts)
        {
            DrawContacts(world, renderer);
        }
    }

    void PhysicsDebugRenderer2D::DrawBoxCollider(const BoxCollider2D& box, Renderer2D& renderer, const Color& color) const
    {
        const Bounds2D bounds = box.GetWorldBounds();

        renderer.DrawRectOutline(Rect(bounds.Min, bounds.GetSize()), color);
    }

    void PhysicsDebugRenderer2D::DrawCircleCollider(const CircleCollider2D& circle, Renderer2D& renderer, const Color& color) const
    {
        const Vector2 center = circle.GetWorldCenter();

        const float radius = circle.GetWorldRadius();

        constexpr int segments = 32;

        constexpr float twoPi = 6.28318530718f;

        Vector2 previous{center.X + radius, center.Y};

        for (int i = 1; i <= segments; ++i)
        {
            const float t = static_cast<float>(i) / static_cast<float>(segments);

            const float angle = t * twoPi;

            const Vector2 current
            {
                center.X + std::cos(angle) * radius,
                center.Y + std::sin(angle) * radius
            };

            renderer.DrawLine(previous, current, color);

            previous = current;
        }
    }

    void PhysicsDebugRenderer2D::DrawCollider(const Collider2D& collider, Renderer2D& renderer) const
    {
        const Color color = GetColliderDebugColor(collider, m_DrawSleepingState);

        switch (collider.GetShape())
        {
            case ColliderShape2D::Box:
            {
                DrawBoxCollider(static_cast<const BoxCollider2D&>(collider), renderer, color);

                DrawOBB(static_cast<const BoxCollider2D&>(collider), renderer);
                
                break;
            }

            case ColliderShape2D::Circle:
            {
                DrawCircleCollider(static_cast<const CircleCollider2D&>(collider), renderer, color);

                break;
            }
        }
    }

    void PhysicsDebugRenderer2D::DrawAABB(const Collider2D& collider, Renderer2D& renderer) const
    {
        const Bounds2D bounds = collider.GetWorldBounds();

        renderer.DrawRectOutline(Rect(bounds.Min, bounds.GetSize()), Color::Cyan());
    }

    void PhysicsDebugRenderer2D::DrawOBB(const BoxCollider2D& collider, Renderer2D& renderer) const
    {
        const OrientedBox2D box = collider.GetWorldOrientedBox();

        renderer.DrawLine(box.Vertices[0], box.Vertices[1], Color::Yellow());

        renderer.DrawLine(box.Vertices[1], box.Vertices[2], Color::Yellow());

        renderer.DrawLine(box.Vertices[2], box.Vertices[3], Color::Yellow());

        renderer.DrawLine(box.Vertices[3], box.Vertices[0], Color::Yellow());

        renderer.DrawLine(box.Center, box.Center + box.AxisX * 30.0f, Color::Red());

        renderer.DrawLine(box.Center, box.Center + box.AxisY * 30.0f, Color::Green());
    }


    void PhysicsDebugRenderer2D::DrawSpatialGrid(const PhysicsWorld2D& world, Renderer2D& renderer) const
    {
        const float cellSize = world.GetSpatialCellSize();

        for (const auto& entry : world.GetSpatialGrid())
        {
            const SpatialCell2D& cell = entry.first;

            const Vector2 position
            {
                static_cast<float>(cell.X) * cellSize,
                static_cast<float>(cell.Y) * cellSize
            };

            const Vector2 size{cellSize, cellSize};

            renderer.DrawRectOutline(Rect(position, size), Color::Gray());
        }
    }

    void PhysicsDebugRenderer2D::DrawContacts(const PhysicsWorld2D& world, Renderer2D& renderer) const
    {
        constexpr float normalLength = 40.0f;

        for (const CollisionManifold2D& manifold : world.GetCurrentContacts())
        {
            if (!manifold.A || manifold.B)
            {
                continue;
            }

            const Vector2 centerA = manifold.A->GetWorldBounds().GetCenter();

            const Vector2 centerB = manifold.B->GetWorldBounds().GetCenter();

            const Vector2 contactCenter = (centerA + centerB) * 0.5f;

            // Normal

            const Vector2 normalEnd = contactCenter + manifold.Normal * normalLength;

            renderer.DrawLine(contactCenter, normalEnd, Color::Red());

            // Penetration

            const Vector2 penetrationEnd = contactCenter + manifold.Normal * manifold.Penetration;

            renderer.DrawLine(contactCenter, penetrationEnd, Color::Yellow());
        }
    }

    void PhysicsDebugRenderer2D::SetDrawColliders(bool enabled)
    {
        m_DrawColliders = enabled;
    }

    bool PhysicsDebugRenderer2D::GetDrawColliders() const
    {
        return m_DrawColliders;
    }

    void PhysicsDebugRenderer2D::SetDrawAABBs(bool enabled)
    {
        m_DrawAABBs = enabled;
    }

    bool PhysicsDebugRenderer2D::GetDrawAABBs() const
    {
        return m_DrawAABBs;
    }

    void PhysicsDebugRenderer2D::SetDrawContacts(bool enabled)
    {
        m_DrawContacts = enabled;
    }

    bool PhysicsDebugRenderer2D::GetDrawContacts() const
    {
        return m_DrawContacts;
    }

    void PhysicsDebugRenderer2D::SetDrawSleepingState(bool enabled)
    {
        m_DrawSleepingState = enabled;
    }

    bool PhysicsDebugRenderer2D::GetDrawSleepingState() const
    {
        return m_DrawSleepingState;
    }

    void PhysicsDebugRenderer2D::SetDrawSpatialGrid(bool enabled)
    {
        m_DrawSpatialGrid = enabled;
    }

    bool PhysicsDebugRenderer2D::GetDrawSpatialGrid() const
    {
        return m_DrawSpatialGrid;
    }
}