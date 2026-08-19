#include "TransformComponent.h"

namespace Engine
{
    TransformComponent::TransformComponent(const Transform2D& transform)
        : m_Transform(transform)
    {
    }

    Transform2D& TransformComponent::GetTransform()
    {
        return m_Transform;
    }

    const Transform2D& TransformComponent::GetTransform() const
    {
        return m_Transform;
    }

    void TransformComponent::SetPosition(const Vector2& position)
    {
        m_Transform.Position = position;
    }

    const Vector2& TransformComponent::GetPosition() const
    {
        return m_Transform.Position;
    }

    void TransformComponent::SetRotation(float rotation)
    {
        m_Transform.Rotation = rotation;
    }

    float TransformComponent::GetRotation() const
    {
        return m_Transform.Rotation;
    }

    void TransformComponent::SetScale(const Vector2& scale)
    {
        m_Transform.Scale = scale;
    }

    const Vector2& TransformComponent::GetScale() const
    {
        return m_Transform.Scale;
    }
}