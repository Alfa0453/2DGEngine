#include "BoxCollider2D.h"

#include "../Math/Bounds2D.h"
#include "../Scene/Entity.h"
#include "../Scene/TransformComponent.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

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

        MarkBoundsDirty();
    }

    const Vector2& BoxCollider2D::GetSize() const
    {
        return m_Size;
    }

    void BoxCollider2D::SetOffset(const Vector2& offset)
    {
        m_Offset = offset;

        MarkBoundsDirty();
    }

    const Vector2& BoxCollider2D::GetOffset() const
    {
        return m_Offset;
    }

    Bounds2D BoxCollider2D::GetWorldBounds() const
    {
        const OrientedBox2D box = GetWorldOrientedBox();

        Bounds2D bounds;

        bounds.Min = box.Vertices[0];

        bounds.Max = box.Vertices[0];

        for (std::size_t i = 1; i < 4; ++i)
        {
            bounds.Encapsulate(box.Vertices[i]);
        }

        return bounds;
    }

    OrientedBox2D BoxCollider2D::GetWorldOrientedBox() const
    {
        OrientedBox2D box;

        const Entity* owner = GetOwner();

        if (!owner)
        {
            return box;
        }

        const TransformComponent* transform = owner->GetComponent<TransformComponent>();

        if (!transform)
        {
            return box;
        }

        const Transform2D& world = transform->GetWorldTransform();

        box.Center = world.TransformPoint(m_Offset);

        const Vector2 scaledSize{std::abs(m_Size.X * world.Scale.X), std::abs(m_Size.Y * world.Scale.Y)};

        box.HalfExtents = scaledSize * 0.5f;

        box.AxisX = Vector2::Rotate(Vector2{1.0f, 0.0f}, world.Rotation);

        box.AxisY = Vector2::Rotate(Vector2{0.0f, 1.0f}, world.Rotation);

        const Vector2 extentX = box.AxisX * box.HalfExtents.X;

        const Vector2 extentY = box.AxisY * box.HalfExtents.Y;

        // Top-left
        box.Vertices[0] = box.Center - extentX - extentY;

        // Top-right
        box.Vertices[1] = box.Center + extentX - extentY;

        // Botom-right
        box.Vertices[2] = box.Center + extentX + extentY;

        // Bottom-left
        box.Vertices[3] = box.Center - extentX + extentY;

        return box;
    }

    Vector2 BoxCollider2D::GetWorldCenter() const
    {
        return GetWorldOrientedBox().Center;
    }

    Vector2 BoxCollider2D::GetWorldHalfExtents() const
    {
        return GetWorldOrientedBox().HalfExtents;
    }
}