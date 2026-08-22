#include "BoxCollider2D.h"

#include "../Math/Bounds2D.h"
#include "../Scene/Entity.h"
#include "../Scene/TransformComponent.h"

#include <algorithm>
#include <cmath>

namespace Engine
{
    BoxCollider2D::BoxCollider2D()
        : Collider2D(ColliderShape2D::Box)
    {
    }

    BoxCollider2D::BoxCollider2D(const Vector2& size)
        : Collider2D(ColliderShape2D::Box)
    {
        SetSize(size);
    }

    void BoxCollider2D::SetSize(const Vector2& size)
    {
        m_Size.X = std::max(0.0f, size.X);

        m_Size.Y = std::max(0.0f, size.Y);
    }

    const Vector2& BoxCollider2D::GetSize() const
    {
        return m_Size;
    }

    void BoxCollider2D::SetOffset(const Vector2& offset)
    {
        m_Offset = offset;
    }

    const Vector2& BoxCollider2D::GetOffset() const
    {
        return m_Offset;
    }

    Bounds2D BoxCollider2D::GetWorldBounds() const
    {
        const Entity* owner = GetOwner();

        if (!owner)
        {
            return Bounds2D();
        }

        const TransformComponent* transform = owner->GetComponent<TransformComponent>();

        if (!transform)
        {
            return Bounds2D();
        }

        const Transform2D& world = transform->GetWorldTransform();

        // Transform collider offset
        const Vector2 worldCenter = world.TransformPoint(m_Offset);

        // Scale collider size
        const Vector2 scaledSize
        {
            std::abs(m_Size.X * world.Scale.X),

            std::abs(m_Size.Y * world.Scale.Y)
        };

        const Vector2 half = scaledSize * 0.5f;

        // Box corners around center
        const Vector2 localCorners[4]
        {
            {-half.X, -half.Y},
            { half.X, -half.Y},
            { half.X,  half.Y},
            {-half.X,  half.Y}
        };

        // First transformed corner
        const Vector2 first = worldCenter + Vector2::Rotate(localCorners[0], world.Rotation);

        Bounds2D bounds;

        bounds.Min = first;

        bounds.Max = first;

        // Remaining corners
        for (int i = 1; i < 4; ++i)
        {
            const Vector2 corner = worldCenter + Vector2::Rotate(localCorners[i], world.Rotation);

            bounds.Encapsulate(corner);
        }

        return bounds;
    }
}