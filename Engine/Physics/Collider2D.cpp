#include "Collider2D.h"

#include <algorithm>
#include <cmath>

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

    void Collider2D::SetOneWay(bool oneWay)
    {
        m_OneWay = oneWay;
    }

    bool Collider2D::IsOneWay() const
    {
        return m_OneWay;
    }

    void Collider2D::SetOneWayAxis(const Vector2& worldAxis)
    {
        // Store a normalized axis so the dot-product test in the solver is a
        // true cosine. Reject degenerate input and keep the previous axis.

        const float lengthSquared = worldAxis.LengthSquared();

        constexpr float epsilon = 0.000001f;

        if (lengthSquared <= epsilon)
        {
            return;
        }

        m_OneWayAxis = worldAxis * (1.0f / std::sqrt(lengthSquared));
    }

    const Vector2& Collider2D::GetOneWayAxis() const
    {
        return m_OneWayAxis;
    }

    void Collider2D::SetOneWayThreshold(float cosineThreshold)
    {
        m_OneWayThreshold = std::clamp(cosineThreshold, -1.0f, 1.0f);
    }

    float Collider2D::GetOneWayThreshold() const
    {
        return m_OneWayThreshold;
    }

    void Collider2D::SetSurfaceVelocity(const Vector2& worldVelocity)
    {
        m_SurfaceVelocity = worldVelocity;
    }

    const Vector2& Collider2D::GetSurfaceVelocity() const
    {
        return m_SurfaceVelocity;
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