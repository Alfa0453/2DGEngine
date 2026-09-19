#pragma once

#include "Joint2D.h"

#include "../Math/Vector2.h"

namespace Engine
{
    class PrismaticJoint2D final : public Joint2D
    {
    public:

        PrismaticJoint2D(Rigidbody2D* bodyA, Rigidbody2D* bodyB, const Vector2& localAnchorA, const Vector2& localAnchorB, const Vector2& localAxisA);

        void SetLocalAnchorA(const Vector2& anchor);

        const Vector2& GetLocalAnchorA() const;

        void SetLocalAnchorB(const Vector2& anchor);

        const Vector2& GetLocalAnchorB() const;

        void SetLocalAxisA(const Vector2& axis);

        const Vector2& GetLocalAxisA() const;

        void SetBiasFactor(float biasFactor);

        float GetBiasFactor() const;

        // ---------------------------------------------------------------
        // MOTOR
        // ---------------------------------------------------------------
        //
        // The motor drives the translation speed along the joint axis toward
        // GetMotorSpeed() (in world units/second), using no more than
        // GetMaxMotorForce(). Use it for elevators, sliding doors, pistons,
        // and moving platforms.

        void EnableMotor(bool enable);

        bool IsMotorEnabled() const;

        void SetMotorSpeed(float unitsPerSecond);

        float GetMotorSpeed() const;

        void SetMaxMotorForce(float maxForce);

        float GetMaxMotorForce() const;

        // Force the motor actually applied on the last solved step
        // (accumulated motor impulse / dt).
        float GetMotorForce(float inverseDeltaTime) const;

        // ---------------------------------------------------------------
        // LIMIT
        // ---------------------------------------------------------------
        //
        // When enabled, the translation along the axis (see GetTranslation) is
        // clamped to [lower, upper], in world units. Use it to bound a
        // slider's travel.

        void EnableLimit(bool enable);

        bool IsLimitEnabled() const;

        void SetLimits(float lower, float upper);

        float GetLowerLimit() const;

        float GetUpperLimit() const;

        void Prepare(PhysicsWorld2D& world, float deltaTime) override;

        void WarmStart(PhysicsWorld2D& world) override;

        void SolveVelocity(PhysicsWorld2D& world) override;

        bool SolvePosition(PhysicsWorld2D& world) override;

        float GetTranslation() const;

    private:

        Vector2 m_LocalAnchorA{0.0f, 0.0f};

        Vector2 m_LocalAnchorB{0.0f, 0.0f};

        Vector2 m_LocalAxisA{1.0f, 0.0f};

        Vector2 m_WorldAxis{1.0f, 0.0f};

        Vector2 m_WorldPerpendicular{0.0f, 1.0f};

        Vector2 m_Ra{0.0f, 0.0f};

        Vector2 m_Rb{0.0f, 0.0f};

        float m_ReferenceAngle = 0.0f;

        float m_LinearEffectiveMass = 0.0f;

        float m_AngularEffectiveMass = 0.0f;

        float m_LinearBias = 0.0f;

        float m_AngularBias = 0.0f;

        float m_AccumulatedLinearImpulse = 0.0f;

        float m_AccumulatedAngularImpulse = 0.0f;

        float m_BiasFactor = 0.2f;

        // MOTOR / LIMIT CONFIGURATION

        bool m_EnableMotor = false;

        float m_MotorSpeed = 0.0f;

        float m_MaxMotorForce = 0.0f;

        bool m_EnableLimit = false;

        float m_LowerTranslation = 0.0f;

        float m_UpperTranslation = 0.0f;

        // AXIAL SOLVER STATE (rebuilt every Prepare)

        // Jacobian scalars for the axial constraint (Box2D convention):
        //   s1 = cross(rA + d, axis), s2 = cross(rB, axis), d = anchorB - anchorA.
        float m_AxialS1 = 0.0f;

        float m_AxialS2 = 0.0f;

        // Effective mass along the axis:
        //   1 / (invMassA + invMassB + invIa*s1^2 + invIb*s2^2).
        float m_AxialMass = 0.0f;

        // Current translation along the axis, sampled in Prepare.
        float m_AxialTranslation = 0.0f;

        float m_InverseDeltaTime = 0.0f;

        // Accumulated axial impulses (reset each Prepare).
        float m_MotorImpulse = 0.0f;

        float m_LowerImpulse = 0.0f;

        float m_UpperImpulse = 0.0f;
    };
}