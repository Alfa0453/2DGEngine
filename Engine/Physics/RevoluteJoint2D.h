#pragma once

#include "Joint2D.h"

#include "../Math/Matrix2x2.h"
#include "../Math/Vector2.h"

namespace Engine
{
    class RevoluteJoint2D final : public Joint2D
    {
    public:

        RevoluteJoint2D(Rigidbody2D* bodyA, Rigidbody2D* bodyB, const Vector2& localAnchorA, const Vector2& localAnchorB);

        void SetLocalAnchorA(const Vector2& anchor);

        const Vector2& GetLocalAnchorA() const;

        void SetLocalAnchorB(const Vector2& anchor);

        const Vector2& GetLocalAnchorB() const;

        void SetBiasFactor(float biasFactor);

        float GetBiasFactor() const;

        // ---------------------------------------------------------------
        // MOTOR
        // ---------------------------------------------------------------
        //
        // The motor drives the relative angular velocity (angularVelocityB -
        // angularVelocityA) toward GetMotorSpeed(), using no more than
        // GetMaxMotorTorque() of torque. Use it for powered hinges: turrets,
        // motorized doors, rotating platforms, wheels.

        void EnableMotor(bool enable);

        bool IsMotorEnabled() const;

        void SetMotorSpeed(float radiansPerSecond);

        float GetMotorSpeed() const;

        void SetMaxMotorTorque(float maxTorque);

        float GetMaxMotorTorque() const;

        // Torque the motor actually applied on the last solved step, in
        // torque units (accumulated motor impulse / dt).
        float GetMotorTorque(float inverseDeltaTime) const;

        // ---------------------------------------------------------------
        // LIMIT
        // ---------------------------------------------------------------
        //
        // When enabled, the relative joint angle is clamped to
        // [lowerAngle, upperAngle], both measured in radians relative to the
        // bodies' orientation at the moment the joint was constructed. Use it
        // for ragdoll joints, swinging doors, levers.

        void EnableLimit(bool enable);

        bool IsLimitEnabled() const;

        void SetLimits(float lowerRadians, float upperRadians);

        float GetLowerLimit() const;

        float GetUpperLimit() const;

        // Current relative angle (radians) and relative angular speed
        // (radians/sec). Angle is measured relative to the reference angle
        // captured at construction.
        float GetJointAngle() const;

        float GetJointSpeed() const;

        void Prepare(PhysicsWorld2D& world, float deltaTime) override;

        void WarmStart(PhysicsWorld2D& world) override;

        void SolveVelocity(PhysicsWorld2D& world) override;

        bool SolvePosition(PhysicsWorld2D& world) override;

    private:

        Vector2 m_LocalAnchorA{0.0f, 0.0f};

        Vector2 m_LocalAnchorB{0.0f, 0.0f};

        Vector2 m_Ra{0.0f, 0.0f};

        Vector2 m_Rb{0.0f, 0.0f};

        Matrix2x2 m_EffectiveMass;

        Vector2 m_Bias{0.0f, 0.0f};

        Vector2 m_AccumulatedImpulse{0.0f, 0.0f};

        float m_BiasFactor = 0.2f;

        // MOTOR / LIMIT CONFIGURATION

        bool m_EnableMotor = false;

        float m_MotorSpeed = 0.0f;

        float m_MaxMotorTorque = 0.0f;

        bool m_EnableLimit = false;

        float m_LowerAngle = 0.0f;

        float m_UpperAngle = 0.0f;

        // Relative angle (radians) between the two bodies at construction.
        // Joint angle is measured relative to this so that limits are
        // expressed in the joint's own frame.
        float m_ReferenceAngle = 0.0f;

        // MOTOR / LIMIT SOLVER STATE (rebuilt every Prepare)

        // Effective mass of the shared angular axis: 1 / (invInertiaA + invInertiaB).
        float m_AxialMass = 0.0f;

        // 1 / deltaTime for the current step, used to scale limit bias.
        float m_InverseDeltaTime = 0.0f;

        // Relative joint angle sampled in Prepare (radians).
        float m_JointAngle = 0.0f;

        // Accumulated axial impulses (reset each Prepare; clamped so limits are
        // one-sided and the motor respects its torque budget).
        float m_MotorImpulse = 0.0f;

        float m_LowerImpulse = 0.0f;

        float m_UpperImpulse = 0.0f;
    };
}