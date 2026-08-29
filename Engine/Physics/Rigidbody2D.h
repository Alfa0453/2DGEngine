#pragma once

#include "BodyType2D.h"
#include "CollisionDetectionMode2D.h"

#include "../Scene/Component.h"
#include "../Math/Vector2.h"

namespace Engine
{
    class PhysicsWorld2D;

    class Rigidbody2D : public Component
    {
    public:

        Rigidbody2D() = default;

        void SetBodyType(BodyType2D type);

        BodyType2D GetBodyType() const;

        void SetVelocity(const Vector2& velocity);

        const Vector2& GetVelocity() const;

        void AddVelocity(const Vector2& deltaVelocity);

        void SetMass(float mass);

        float GetMass() const;

        float GetInverseMass() const;

        void SetGravityScale(float gravityScale);

        float GetGravityScale() const;

        void SetLinearDamping(float damping);

        float GetLinearDamping() const;

        void AddForce(const Vector2& force);

        void AddImpulse(const Vector2& impulse);

        void ClearForces();

        const Vector2& GetAccumulatedForce() const;

        bool IsStatic() const;

        bool IsKinematic() const;

        bool IsDynamic() const;

        bool IsSleeping() const;

        bool CanSleep() const;

        void SetAllowSleep(bool allowSleep);

        void Sleep();

        void Wake();

        float GetSleepTimer() const;

        void SetCollisionDetectionMode(CollisionDetectionMode2D mode);

        CollisionDetectionMode2D GetCollisionDetectionMode() const;

        void SetPreviousPosition(const Vector2& position);

        const Vector2& GetPreviousPosition() const;

        void SetAngularVelocity(float angularVelocity);

        float GetAngularVelocity() const;

        void AddAngularVelociy(float deltaAngularVelocity);

        void SetAngularDamping(float damping);

        float GetangularDamping() const;

        void AddTorque(float torque);

        float GetAccumulatedTorque() const;

        void ClearTorque();

        float GetMomenOfInertia() const;

        float GetInverseInertia() const;

    protected:

        friend class PhysicsWorld2D;

    private:
        
        void RecalculateInverseMass();

        void SetVelocityFromPhysics(const Vector2& velocity);

        void SetAngularVelocityFromPhysics(float angularVelocity);

        void AddSleepTime(float deltaTime);

        void ResetSleepTimer();

    private:

        BodyType2D m_BodyType = BodyType2D::Dynamic;

        Vector2 m_Velocity{0.0f, 0.0f};

        Vector2 m_AccumulatedForce{0.0f, 0.0f};

        float m_Mass = 1.0f;

        float m_InverseMass = 1.0f;

        float m_GravityScale = 1.0f;

        float m_LinearDamping = 0.0f;

        bool m_AllowSleep = true;

        bool m_IsSleeping = false;

        float m_SleepTimer = 0.0f;

        float m_AngularVelocity = 0.0f;

        float m_AccumulatedTorque = 0.0f;

        float m_AngularDamping = 0.0f;

        CollisionDetectionMode2D m_CollisionDetectionMode = CollisionDetectionMode2D::Discrete;

        Vector2 m_PreviousPosition{0.0f, 0.0f};
    };
}