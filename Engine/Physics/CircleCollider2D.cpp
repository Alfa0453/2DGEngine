#include "CircleCollider2D.h"

#include "../Scene/Entity.h"
#include "../Scene/TransformComponent.h"

#include <algorithm>
#include <cmath>

namespace Engine
{
    CircleCollider2D::CircleCollider2D()
        : Collider2D(ColliderShape2D::Circle)
    {
    }

    CircleCollider2D::CircleCollider2D(float radius)
        : Collider2D(ColliderShape2D::Circle)
    {
        SetRadius(radius);
    }

    void CircleCollider2D::SetRadius(float radius)
    {
        m_Radius = std::max(0.0f, radius);

        MarkBoundsDirty();
    }

    float CircleCollider2D::GetRadius() const
    {
        return m_Radius;
    }

    void CircleCollider2D::SetOffset(const Vector2& offset)
    {
        m_Offset = offset;

        MarkBoundsDirty();
    }

    const Vector2& CircleCollider2D::GetOffset() const
    {
        return m_Offset;
    }

    Vector2 CircleCollider2D::GetWorldCenter() const
    {
        const Entity* owner = GetOwner();

        if (!owner)
        {
            return{0.0f, 0.0f};
        }

        const TransformComponent* transform = owner->GetComponent<TransformComponent>();

        if (!transform)
        {
            return{0.0f, 0.0f};
        }

        const Transform2D& world = transform->GetWorldTransform();

        return world.TransformPoint(m_Offset);
    }

    float CircleCollider2D::GetWorldRadius() const
    {
        const Entity* owner = GetOwner();

        if (!owner)
        {
            return 0.0f;
        }

        const TransformComponent* transform = owner->GetComponent<TransformComponent>();

        if (!transform)
        {
            return 0.0f;
        }

        const Transform2D& world = transform->GetWorldTransform();

        const float scale = std::max(std::abs(world.Scale.X), std::abs(world.Scale.Y));

        return m_Radius * scale;
    }

    Bounds2D CircleCollider2D::GetWorldBounds() const
    {
        const Vector2 center = GetWorldCenter();

        const float radius = GetWorldRadius();

        const Vector2 extent{radius, radius};

        return Bounds2D(center - extent, center + extent);
    }
}