#include "CapsuleCollider2D.h"

#include "../Scene/Entity.h"
#include "../Scene/TransformComponent.h"

#include <algorithm>
#include <cmath>

namespace Engine
{
    CapsuleCollider2D::CapsuleCollider2D()
        : Collider2D(ColliderShape2D::Capsule)
    {
    }

    CapsuleCollider2D::CapsuleCollider2D(float radius, float halfHeight)
        : Collider2D(ColliderShape2D::Capsule)
    {
        SetRadius(radius);

        SetHalfHeight(halfHeight);
    }

    void CapsuleCollider2D::SetOffset(const Vector2& offset)
    {
        if (m_Offset.X == offset.X && m_Offset.Y == offset.Y)
        {
            return;
        }

        m_Offset = offset;

        MarkBoundsDirty();
    }

    const Vector2& CapsuleCollider2D::GetOffset() const
    {
        return m_Offset;
    }

    void CapsuleCollider2D::SetRadius(float radius)
    {
        radius = std::max(0.0f, radius);

        if (m_Radius == radius)
        {
            return;
        }

        m_Radius = radius;

        MarkBoundsDirty();
    }

    float CapsuleCollider2D::GetRadius() const
    {
        return m_Radius;
    }

    void CapsuleCollider2D::SetHalfHeight(float halfHeight)
    {
        halfHeight = std::max(0.0f, halfHeight);

        if (m_HalfHeight == halfHeight)
        {
            return;
        }

        m_HalfHeight = halfHeight;

        MarkBoundsDirty();
    }

    float CapsuleCollider2D::GetHalfHeight() const
    {
        return m_HalfHeight;
    }

    Capsule2D CapsuleCollider2D::GetWorldCapsule() const
    {
        Capsule2D capsule;

        Entity* owner = GetOwner();

        if (!owner)
        {
            return capsule;
        }

        TransformComponent* transform = owner->GetComponent<TransformComponent>();

        if (!transform)
        {
            return capsule;
        }

        const Transform2D& world = transform->GetWorldTransform();

        // WORLD CENTER

        const Vector2 rotatedOffset = Vector2::Rotate(m_Offset, world.Rotation);

        const Vector2 worldCenter = world.TransformPoint(m_Offset);

        // WORLD CAPSULE AXIS

        const Vector2 localAxis{0.0f, 1.0f};

        const Vector2 worldAxis = Vector2::Rotate(localAxis, world.Rotation);

        // CENTER SEGMENT

        const float halfHeightScale = std::abs(world.Scale.Y);

        const float radiusScale = std::max(std::abs(world.Scale.X), std::abs(world.Scale.Y));

        const float worldHalfHeight = m_HalfHeight * halfHeightScale;
        
        const float worldRadius = m_Radius * radiusScale;

        capsule.PointA = worldCenter - worldAxis * worldHalfHeight;

        capsule.PointB = worldCenter + worldAxis * worldHalfHeight;

        capsule.Radius = worldRadius;

        return capsule;
    }

    Bounds2D CapsuleCollider2D::GetWorldBounds() const
    {
        const Capsule2D capsule = GetWorldCapsule();

        Bounds2D bounds;

        bounds.Min = 
        {
            std::min(capsule.PointA.X, capsule.PointB.X) - capsule.Radius,

            std::min(capsule.PointA.Y, capsule.PointB.Y) - capsule.Radius
        };

        bounds.Max =
        {
            std::max(capsule.PointA.X, capsule.PointB.X) + capsule.Radius,

            std::max(capsule.PointA.Y, capsule.PointB.Y) + capsule.Radius
        };

        return bounds;
    }
}