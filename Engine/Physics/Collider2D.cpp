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
        m_IsEnabled = enabled;
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
}