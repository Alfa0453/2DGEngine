#include "Rigidbody2D.h"

#include <algorithm>

namespace Engine
{
    void Rigidbody2D::SetBodyType(BodyType2D type)
    {
        m_BodyType = type;

        RecalculateInverseMass();
    }

    BodyType2D Rigidbody2D::GetBodyType() const
    {
        return m_BodyType;
    }

    void Rigidbody2D::SetVelocity(const Vector2& velocity)
    {
        m_Velocity = velocity;
    }

    const Vector2& Rigidbody2D::GetVelocity() const
    {
        return m_Velocity;
    }

    void Rigidbody2D::AddVelocity(const Vector2& deltaVelocity)
    {
        m_Velocity += deltaVelocity;
    }

    void Rigidbody2D::SetMass(float mass)
    {
        m_Mass = std::max(0.0001f, mass);

        RecalculateInverseMass();
    }

    float Rigidbody2D::GetMass() const
    {
        return m_Mass;
    }

    float Rigidbody2D::GetInverseMass() const
    {
        return m_InverseMass;
    }

    void Rigidbody2D::SetGravityScale(float gravityScale)
    {
        m_GravityScale = gravityScale;
    }

    float Rigidbody2D::GetGravityScale() const
    {
        return m_GravityScale;
    }

    void Rigidbody2D::SetLinearDamping(float damping)
    {
        m_LinearDamping = std::max(0.0f, damping);
    }

    float Rigidbody2D::GetLinearDamping() const
    {
        return m_LinearDamping;
    }

    void Rigidbody2D::AddForce(const Vector2& force)
    {
        if (!IsDynamic())
        {
            return;
        }

        m_AccumulatedForce += force;
    }

    void Rigidbody2D::AddImpulse(const Vector2& impulse)
    {
        if (!IsDynamic())
        {
            return;
        }

        m_Velocity += impulse * m_InverseMass;
    }

    void Rigidbody2D::ClearForces()
    {
        m_AccumulatedForce = {0.0f, 0.0f};
    }

    const Vector2& Rigidbody2D::GetAccumulatedForce() const
    {
        return m_AccumulatedForce;
    }

    bool Rigidbody2D::IsStatic() const
    {
        return m_BodyType == BodyType2D::Static;
    }

    bool Rigidbody2D::IsKinematic() const
    {
        return m_BodyType == BodyType2D::Kinematic;
    }

    bool Rigidbody2D::IsDynamic() const
    {
        return m_BodyType == BodyType2D::Dynamic;
    }

    void Rigidbody2D::RecalculateInverseMass()
    {
        if (!IsDynamic())
        {
            m_InverseMass = 0.0f;
        }

        m_InverseMass = 1.0f / m_Mass;
    }
}