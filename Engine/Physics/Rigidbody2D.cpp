#include "Rigidbody2D.h"

#include "BoxCollider2D.h"
#include "CircleCollider2D.h"

#include "../Scene/Entity.h"
#include "../Scene/TransformComponent.h"
#include "OrientedBox2D.h"

#include <algorithm>
#include <cstdlib>
#include <cmath>

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

        m_AngularVelocity = 0.0f;

        m_AccumulatedTorque = 0.0f;
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

    void Rigidbody2D::SetAngularVelocity(float angularVelocity)
    {
        if (!IsDynamic())
        {
            m_AngularVelocity = angularVelocity;

            return;
        }

        if (angularVelocity == m_AngularVelocity)
        {
            return;
        }

        Wake();

        m_AngularVelocity = angularVelocity;
    }

    float Rigidbody2D::GetAngularVelocity() const
    {
        return m_AngularVelocity;
    }

    void Rigidbody2D::AddAngularVelociy(float deltaAngularVelocity)
    {
        if (!IsDynamic())
        {
            return;
        }

        if (std::abs(deltaAngularVelocity) <= 0.000001f)
        {
            return;
        }

        Wake();

        m_AngularVelocity += deltaAngularVelocity;
    }

    void Rigidbody2D::SetAngularDamping(float damping)
    {
        m_AngularDamping = std::max(0.0f, damping);
    }

    float Rigidbody2D::GetangularDamping() const
    {
        return m_AngularDamping;
    }

    void Rigidbody2D::AddTorque(float torque)
    {
        if (!IsDynamic())
        {
            return;
        }

        if (std::abs(torque) <= 0.000001f)
        {
            return;
        }

        Wake();

        m_AccumulatedTorque += torque;
    }

    float Rigidbody2D::GetAccumulatedTorque() const
    {
        return m_AccumulatedTorque;
    }

    void Rigidbody2D::ClearTorque()
    {
        m_AccumulatedTorque = 0.0f;
    }

    void Rigidbody2D::SetAngularVelocityFromPhysics(float angularVelocity)
    {
        m_AngularVelocity = angularVelocity;
    }

    float Rigidbody2D::GetMomenOfInertia() const
    {
        const float mass = GetMass();

        if (mass <= 0.0f)
        {
            return 0.0f;
        }

        Entity* owner = GetOwner();

        if (!owner)
        {
            return 0.0f;
        }

        if (BoxCollider2D* box = owner->GetComponent<BoxCollider2D>())
        {
            const OrientedBox2D obb = box->GetWorldOrientedBox();

            const float width = obb.HalfExtents.X * 2.0f;

            const float height = obb.HalfExtents.Y * 2.0f;

            float inertia = mass * (width * width + height * height) / 12.0f;

            if (TransformComponent* transform = owner->GetComponent<TransformComponent>())
            {
                const Vector2 bodyCenter = transform->GetWorldTransform().Position;

                const Vector2 offset = obb.Center - bodyCenter;

                inertia += mass * offset.LengthSqured();
            }

            return inertia;
        }

        if (CircleCollider2D* circle = owner->GetComponent<CircleCollider2D>())
        {
            const float radius = circle->GetWorldRadius();

            float inertia = 0.5f * mass * radius * radius;

            if (TransformComponent* transform = owner->GetComponent<TransformComponent>())
            {
                const Vector2 bodyCenter = transform->GetWorldTransform().Position;

                const Vector2 circleCenter = circle->GetWorldCenter();

                const Vector2 offset = circleCenter - bodyCenter;

                inertia += mass * offset.LengthSqured();
            }

            return inertia;
        }

        return 0.0f;
    }

    float Rigidbody2D::GetInverseInertia() const
    {
        // Static and kinematic bodies do not respond rotationalltt y to physical impulse.

        if (!IsDynamic())
        {
            return 0.0f;
        }

        const float inertia = GetMomenOfInertia();

        constexpr float epsilon = 0.000001f;

        if (inertia <= epsilon)
        {
            return 0.0f;
        }

        return 1.0f / inertia;
    }
}