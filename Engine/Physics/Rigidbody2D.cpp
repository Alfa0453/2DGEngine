#include "Rigidbody2D.h"

#include <algorithm>

namespace Engine
{
    void Rigidbody2D::SetBodyType(BodyType2D type)
    {
        m_BodyType = type;

        m_IsSleeping = false;

        m_SleepTimer = 0.0f;

        RecalculateInverseMass();
    }

    BodyType2D Rigidbody2D::GetBodyType() const
    {
        return m_BodyType;
    }

    void Rigidbody2D::SetVelocity(const Vector2& velocity)
    {
        if (IsDynamic())
        {
            Wake();
        }

        m_Velocity = velocity;
    }

    const Vector2& Rigidbody2D::GetVelocity() const
    {
        return m_Velocity;
    }

    void Rigidbody2D::AddVelocity(const Vector2& deltaVelocity)
    {
        if (!IsDynamic())
        {
            return;
        }

        Wake();

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
        if (IsDynamic() && gravityScale != m_GravityScale)
        {
            Wake();
        }
        
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

        if (force.X == 0.0f && force.Y == 0.0f)
        {
            return;
        }

        Wake();

        m_AccumulatedForce += force;
    }

    void Rigidbody2D::AddImpulse(const Vector2& impulse)
    {
        if (!IsDynamic())
        {
            return;
        }

        if (impulse.X == 0.0f && impulse.Y == 0.0f)
        {
            return;
        }

        Wake();

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

            return;
        }

        m_InverseMass = 1.0f / m_Mass;
    }

    bool Rigidbody2D::IsSleeping() const
    {
        return m_IsSleeping;
    }

    bool Rigidbody2D::CanSleep() const
    {
        return IsDynamic() && m_AllowSleep;
    }

    void Rigidbody2D::SetAllowSleep(bool allowSleep)
    {
        m_AllowSleep = allowSleep;

        if (!m_AllowSleep)
        {
            Wake();
        }
    }

    float Rigidbody2D::GetSleepTimer() const
    {
        return m_SleepTimer;
    }

    void Rigidbody2D::Sleep()
    {
        if (!CanSleep())
        {
            return;
        }

        m_IsSleeping = true;

        m_SleepTimer = 0.0f;

        m_Velocity = {0.0f, 0.0f};

        m_AccumulatedForce = {0.0f, 0.0f};
    }

    void Rigidbody2D::Wake()
    {
        m_IsSleeping = false;

        m_SleepTimer = 0.0f;
    }

    void Rigidbody2D::AddSleepTime(float deltaTime)
    {
        m_SleepTimer += deltaTime;
    }

    void Rigidbody2D::ResetSleepTimer()
    {
        m_SleepTimer = 0.0f;
    }

    void Rigidbody2D::SetVelocityFromPhysics(const Vector2& velocity)
    {
        m_Velocity = velocity;
    }

    void Rigidbody2D::SetCollisionDetectionMode(CollisionDetectionMode2D mode)
    {
        m_CollisionDetectionMode = mode;

        if (IsDynamic())
        {
            Wake();
        }
    }

    CollisionDetectionMode2D Rigidbody2D::GetCollisionDetectionMode() const
    {
        return m_CollisionDetectionMode;
    }

    void Rigidbody2D::SetPreviousPosition(const Vector2& position)
    {
        m_PreviousPosition = position;
    }

    const Vector2& Rigidbody2D::GetPreviousPosition() const
    {
        return m_PreviousPosition;
    }
}