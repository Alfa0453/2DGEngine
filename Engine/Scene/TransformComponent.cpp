#include "TransformComponent.h"
#include "Entity.h"

namespace Engine
{
    TransformComponent::TransformComponent(const Transform2D& transform)
        : m_LocalTransform(transform)
    {
    }

    Transform2D& TransformComponent::GetLocalTransform()
    {
        return m_LocalTransform;
    }

    const Transform2D& TransformComponent::GetLocalTransform() const
    {
        return m_LocalTransform;
    }

    const Transform2D& TransformComponent::GetWorldTransform() const
    {
        if (!m_WorldTransformDirty)
        {
            return m_CachedWorldTransform;
        }

        m_CachedWorldTransform = m_LocalTransform;

        Entity* owner = GetOwner();

        if (owner)
        {
            EntityHandle parentHandle = owner->GetParent();

            Entity* parent = parentHandle.Get();

            if (parent)
            {
                TransformComponent* parentTransform = parent->GetComponent<TransformComponent>();

                if (parentTransform)
                {
                    const Transform2D parentWorld = parentTransform->GetWorldTransform();

                    m_CachedWorldTransform = Transform2D::Combine(parentWorld, m_LocalTransform);
                }
            }   
        }

        m_WorldTransformDirty = false;

        ++m_WorldVersion;
        ++m_RecalculationCount;

        return m_CachedWorldTransform;
    }

    Vector2 TransformComponent::GetWorldPosition() const
    {
        return GetWorldTransform().Position;
    }

    float TransformComponent::GetWorldRotation() const
    {
        return GetWorldTransform().Rotation;
    }

    Vector2 TransformComponent::GetWorldScale() const
    {
        return GetWorldTransform().Scale;
    }

    void TransformComponent::SetLocalPosition(const Vector2& position)
    {
        m_LocalTransform.Position = position;

        MarkDirty();
    }

    const Vector2& TransformComponent::GetLocalPosition() const
    {
        return m_LocalTransform.Position;
    }

    void TransformComponent::SetLocalRotation(float rotation)
    {
        m_LocalTransform.Rotation = rotation;

        MarkDirty();
    }

    float TransformComponent::GetLocalRotation() const
    {
        return m_LocalTransform.Rotation;
    }

    void TransformComponent::SetLocalScale(const Vector2& scale)
    {
        m_LocalTransform.Scale = scale;

        MarkDirty();
    }

    const Vector2& TransformComponent::GetLocalScale() const
    {
        return m_LocalTransform.Scale;
    }

    void TransformComponent::SetPosition(const Vector2& position)
    {
        SetLocalPosition(position);
    }

    const Vector2& TransformComponent::GetPosition() const
    {
        return GetLocalPosition();
    }

    void TransformComponent::SetRotation(float rotation)
    {
        SetLocalRotation(rotation);
    }

    float TransformComponent::GetRotation() const
    {
        return GetLocalTransform().Rotation;
    }

    void TransformComponent::SetScale(const Vector2& scale)
    {
        SetLocalScale(scale);
    }

    const Vector2& TransformComponent::GetScale() const
    {
        return GetLocalTransform().Scale;
    }

    void TransformComponent::MarkDirty()
    {
        if (m_WorldTransformDirty)
        {
            return;
        }

        m_WorldTransformDirty = true;

        Entity* owner = GetOwner();

        if (!owner)
        {
            return;
        }

        for (const EntityHandle& childHandle : owner->GetChildren())
        {
            Entity* child = childHandle.Get();

            if (!child)
            {
                continue;
            }

            TransformComponent* childTransform = child->GetComponent<TransformComponent>();

            if (childTransform)
            {
                childTransform->MarkDirty();
            }
        }
    }

    bool TransformComponent::IsWorldTransformDirty() const
    {
        return m_WorldTransformDirty;
    }

    void TransformComponent::Translate(const Vector2& offset)
    {
        m_LocalTransform.Position += offset;

        MarkDirty();
    }

    void TransformComponent::RotateBy(float degrees)
    {
        m_LocalTransform.Rotation += degrees;

        MarkDirty();
    }

    void TransformComponent::ScaleBy(const Vector2& multiplier)
    {
        m_LocalTransform.Scale *= multiplier;

        MarkDirty();
    }

    std::uint64_t TransformComponent::GetWorldVersion() const
    {
        GetWorldTransform();

        return m_WorldVersion;
    }

    std::uint64_t TransformComponent::GetRecalculationCount() const
    {
        return m_RecalculationCount;
    }

    void TransformComponent::SetWorldPosition(const Vector2& position)
    {
        Entity* owner = GetOwner();

        if (!owner)
        {
            SetLocalPosition(position);

            return;
        }

        EntityHandle parentHandle = owner->GetParent();

        Entity* parent = parentHandle.Get();

        if (!parent)
        {
            SetLocalPosition(position);

            return;
        }

        TransformComponent* parentTransform = parent->GetComponent<TransformComponent>();

        if (!parentTransform)
        {
            SetLocalPosition(position);

            return;
        }

        const Transform2D& parentWorld = parentTransform->GetWorldTransform();

        SetLocalPosition(parentWorld.InverseTransformPoint(position));
    }

    void TransformComponent::SetWorldRotation(float rotation)
    {
        Entity* owner = GetOwner();

        if (!owner)
        {
            SetLocalRotation(rotation);

            return;
        }

        EntityHandle parentHandle = owner->GetParent();

        Entity* parent =parentHandle.Get();

        if (!parent)
        {
            SetLocalRotation(rotation);

            return;
        }

        TransformComponent* parentTransform = parent->GetComponent<TransformComponent>();

        if (!parentTransform)
        {
            SetLocalRotation(rotation);

            return;
        }

        const float parentWorldRotation = parentTransform->GetWorldRotation();

        SetLocalRotation(rotation - parentWorldRotation);
    }

    void TransformComponent::SetWorldScale(const Vector2& scale)
    {
        Entity* owner = GetOwner();

        if (!owner)
        {
            SetLocalScale(scale);

            return;
        }

        EntityHandle parentHandle = owner->GetParent();

        Entity* parent = parentHandle.Get();

        if (!parent)
        {
            SetLocalScale(scale);

            return;
        }

        TransformComponent* parentTransform = parent->GetComponent<TransformComponent>();

        if (!parentTransform)
        {
            SetLocalScale(scale);

            return;
        }

        const Vector2 parentWorldScale = parentTransform->GetWorldScale();

        SetLocalScale(Vector2::SafeDivide(scale, parentWorldScale));
    }

    void TransformComponent::SetWorldTransform(const Transform2D& transform)
    {
        Entity* owner = GetOwner();

        if (!owner)
        {
            m_LocalTransform = transform;

            MarkDirty();

            return;
        }

        EntityHandle parentHandle = owner->GetParent();

        Entity* parent = parentHandle.Get();

        if (!parent)
        {
            m_LocalTransform = transform;

            MarkDirty();

            return;
        }

        TransformComponent* parentTransform = parent->GetComponent<TransformComponent>();

        if (!parentTransform)
        {
            m_LocalTransform = transform;

            MarkDirty();

            return;
        }

        const Transform2D& parentWorld = parentTransform->GetWorldTransform();

        m_LocalTransform = Transform2D::InverseCombine(parentWorld, transform);

        MarkDirty();
    }
}