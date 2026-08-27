#include "Collider2D.h"

namespace Engine
{
    Collider2D::Collider2D(ColliderShape2D shape) 
        : m_Shape(shape)
    {
    }

    ColliderShape2D Collider2D::GetShape() const
    {
        return m_Shape;
    }

    void Collider2D::SetLayer(CollisionLayerMask2D layer)
    {
        m_Layer = layer;
    }

    CollisionLayerMask2D Collider2D::GetLayer() const
    {
        return m_Layer;
    }

    void Collider2D::SetMask(CollisionLayerMask2D mask)
    {
        m_Mask = mask;
    }

    CollisionLayerMask2D Collider2D::GetMask() const
    {
        return m_Mask;
    }

    void Collider2D::SetTrigger(bool trigger)
    {
        m_IsTrigger = trigger;
    }

    bool Collider2D::IsTrigger() const
    {
        return m_IsTrigger;
    }

    void Collider2D::SetEnabled(bool enabled)
    {
        if (m_IsEnabled == enabled)
        {
            return;
        }

        m_IsEnabled = enabled;

        MarkBoundsDirty();
    }

    bool Collider2D::IsEnabled() const
    {
        return m_IsEnabled;
    }

    bool Collider2D::CanInteractWith(const Collider2D& other) const
    {
        if (!m_IsEnabled || !other.m_IsEnabled)
        {
            return false;
        }

        const bool thisWantsOther = (m_Mask & other.m_Layer) != 0;

        const bool otherWantsThis = (other.m_Mask & m_Layer) != 0;

        return thisWantsOther && otherWantsThis;
    }

    void Collider2D::SetPhysicsMaterial(const PhysicsMaterial2D& material)
    {
        m_PhysicsMaterial = material;

        m_PhysicsMaterial.Clamp();
    }

    const PhysicsMaterial2D& Collider2D::GetPhysicsMaterial() const
    {
        return m_PhysicsMaterial;
    }

    PhysicsMaterial2D& Collider2D::GetPhysicsMaterial()
    {
        return m_PhysicsMaterial;
    }

    std::uint64_t Collider2D::GetBoundsRevision() const
    {
        return m_BoundsRevision;
    }

    void Collider2D::MarkBoundsDirty()
    {
        ++m_BoundsRevision;

        if (m_BoundsRevision == 0)
        {
            m_BoundsRevision = 1;
        }
    }
}