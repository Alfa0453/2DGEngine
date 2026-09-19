#pragma once

#include "BodyType2D.h"
#include "CollisionDetectionMode2D.h"

#include "../Scene/Component.h"
#include "../Math/Vector2.h"

#include <vector>

namespace Engine
{
    class PhysicsWorld2D;
    class PolygonCollider2D;

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

        // Applies a continuous force at an arbitrary world-space point.
        //
        // Unlike AddForce (which is treated as acting through the center of
        // mass and therefore produces no rotation), a force applied off the
        // center generates a torque equal to cross(r, force), where r is the
        // lever arm from the body's world center of mass to worldPoint. This
        // is the correct way to model explosions, thrusters, recoil, or any
        // hit that should also spin the body. The torque contribution is
        // suppressed automatically when the body has fixed rotation.
        void AddForceAtPosition(const Vector2& force, const Vector2& worldPoint);

        // Instantaneous equivalent of AddForceAtPosition: applies a linear
        // impulse at a world-space point, changing both linear velocity
        // (impulse * inverseMass) and angular velocity
        // (inverseInertia * cross(r, impulse)) in a single step.
        void AddImpulseAtPosition(const Vector2& impulse, const Vector2& worldPoint);

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

        void AddAngularVelocity(float deltaAngularVelocity);

        // Rotation lock. When enabled the body behaves as if it had infinite
        // rotational inertia: GetInverseInertia() returns 0, so contacts,
        // joints and torque can never spin it, and any existing spin is
        // cleared. This is the standard tool for player/character bodies and
        // top-down actors that must translate but never topple or rotate.
        void SetFixedRotation(bool fixedRotation);

        bool IsFixedRotation() const;

        void SetAngularDamping(float damping);

        float GetAngularDamping() const;

        void AddTorque(float torque);

        float GetAccumulatedTorque() const;

        void ClearTorque();

        float GetMomentOfInertia() const;

        float GetInverseInertia() const;

        void SetVelocityFromPhysics(const Vector2& velocity);

        void SetAngularVelocityFromPhysics(float angularVelocity);

    protected:

        friend class PhysicsWorld2D;

    private:
        
        void RecalculateInverseMass();

        void AddSleepTime(float deltaTime);

        void ResetSleepTimer();

        static float CalculateCapsuleInertia(float mass, float radius, float halfHeight);

        static float CalculatePolygonInertia(float mass, const PolygonCollider2D& polygon);

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

        // When true the body's rotational degree of freedom is locked (see
        // SetFixedRotation). Reflected through GetInverseInertia().
        bool m_FixedRotation = false;

        CollisionDetectionMode2D m_CollisionDetectionMode = CollisionDetectionMode2D::Discrete;

        Vector2 m_PreviousPosition{0.0f, 0.0f};
    };
}