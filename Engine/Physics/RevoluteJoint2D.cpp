#include "RevoluteJoint2D.h"

#include "PhysicsWorld2D.h"
#include "Rigidbody2D.h"

#include "../Scene/Entity.h"
#include "../Scene/TransformComponent.h"

#include <algorithm>
#include <cmath>

namespace Engine
{
    RevoluteJoint2D::RevoluteJoint2D(Rigidbody2D* bodyA, Rigidbody2D* bodyB, const Vector2& localAnchorA, const Vector2& localAnchorB)
        : Joint2D(bodyA, bodyB), m_LocalAnchorA(localAnchorA), m_LocalAnchorB(localAnchorB)
    {
        // Capture the relative orientation of the two bodies at construction.
        // The joint angle (and therefore the limits) are measured relative to
        // this reference so that the joint starts at angle 0.

        Entity* entityA = m_BodyA ? m_BodyA->GetOwner() : nullptr;

        Entity* entityB = m_BodyB ? m_BodyB->GetOwner() : nullptr;

        if (entityA && entityB)
        {
            TransformComponent* transformA = entityA->GetComponent<TransformComponent>();

            TransformComponent* transformB = entityB->GetComponent<TransformComponent>();

            if (transformA && transformB)
            {
                constexpr float degreesToRadians = 0.017453292519943295f;

                const float relativeDegrees = transformB->GetWorldTransform().Rotation - transformA->GetWorldTransform().Rotation;

                m_ReferenceAngle = relativeDegrees * degreesToRadians;
            }
        }
    }

    void RevoluteJoint2D::SetLocalAnchorA(const Vector2& anchor)
    {
        m_LocalAnchorA = anchor;

        m_AccumulatedImpulse = {0.0f, 0.0f};

        if (m_BodyA)
        {
            m_BodyA->Wake();
        }

        if (m_BodyB)
        {
            m_BodyB->Wake();
        }
    }

    const Vector2& RevoluteJoint2D::GetLocalAnchorA() const
    {
        return m_LocalAnchorA;
    }

    void RevoluteJoint2D::SetLocalAnchorB(const Vector2& anchor)
    {
        m_LocalAnchorB = anchor;

        m_AccumulatedImpulse = {0.0f, 0.0f};

        if (m_BodyA)
        {
            m_BodyA->Wake();
        }

        if (m_BodyB)
        {
            m_BodyB->Wake();
        }
    }

    const Vector2& RevoluteJoint2D::GetLocalAnchorB() const
    {
        return m_LocalAnchorB;
    }

    void RevoluteJoint2D::SetBiasFactor(float biasFactor)
    {
        m_BiasFactor = std::max(0.0f, biasFactor);
    }

    float RevoluteJoint2D::GetBiasFactor() const
    {
        return m_BiasFactor;
    }

    void RevoluteJoint2D::EnableMotor(bool enable)
    {
        if (m_EnableMotor == enable)
        {
            return;
        }

        m_EnableMotor = enable;

        // Waking is required so an idle assembly starts responding to the motor.

        if (m_BodyA)
        {
            m_BodyA->Wake();
        }

        if (m_BodyB)
        {
            m_BodyB->Wake();
        }
    }

    bool RevoluteJoint2D::IsMotorEnabled() const
    {
        return m_EnableMotor;
    }

    void RevoluteJoint2D::SetMotorSpeed(float radiansPerSecond)
    {
        if (m_MotorSpeed == radiansPerSecond)
        {
            return;
        }

        m_MotorSpeed = radiansPerSecond;

        if (m_EnableMotor)
        {
            if (m_BodyA)
            {
                m_BodyA->Wake();
            }

            if (m_BodyB)
            {
                m_BodyB->Wake();
            }
        }
    }

    float RevoluteJoint2D::GetMotorSpeed() const
    {
        return m_MotorSpeed;
    }

    void RevoluteJoint2D::SetMaxMotorTorque(float maxTorque)
    {
        m_MaxMotorTorque = std::max(0.0f, maxTorque);

        if (m_EnableMotor)
        {
            if (m_BodyA)
            {
                m_BodyA->Wake();
            }

            if (m_BodyB)
            {
                m_BodyB->Wake();
            }
        }
    }

    float RevoluteJoint2D::GetMaxMotorTorque() const
    {
        return m_MaxMotorTorque;
    }

    float RevoluteJoint2D::GetMotorTorque(float inverseDeltaTime) const
    {
        return m_MotorImpulse * inverseDeltaTime;
    }

    void RevoluteJoint2D::EnableLimit(bool enable)
    {
        if (m_EnableLimit == enable)
        {
            return;
        }

        m_EnableLimit = enable;

        // Clear accumulated limit impulses so a freshly toggled limit does not
        // apply stale corrective forces.

        m_LowerImpulse = 0.0f;

        m_UpperImpulse = 0.0f;

        if (m_BodyA)
        {
            m_BodyA->Wake();
        }

        if (m_BodyB)
        {
            m_BodyB->Wake();
        }
    }

    bool RevoluteJoint2D::IsLimitEnabled() const
    {
        return m_EnableLimit;
    }

    void RevoluteJoint2D::SetLimits(float lowerRadians, float upperRadians)
    {
        // Keep the invariant lower <= upper regardless of caller ordering.

        m_LowerAngle = std::min(lowerRadians, upperRadians);

        m_UpperAngle = std::max(lowerRadians, upperRadians);

        m_LowerImpulse = 0.0f;

        m_UpperImpulse = 0.0f;

        if (m_EnableLimit)
        {
            if (m_BodyA)
            {
                m_BodyA->Wake();
            }

            if (m_BodyB)
            {
                m_BodyB->Wake();
            }
        }
    }

    float RevoluteJoint2D::GetLowerLimit() const
    {
        return m_LowerAngle;
    }

    float RevoluteJoint2D::GetUpperLimit() const
    {
        return m_UpperAngle;
    }

    float RevoluteJoint2D::GetJointAngle() const
    {
        Entity* entityA = m_BodyA ? m_BodyA->GetOwner() : nullptr;

        Entity* entityB = m_BodyB ? m_BodyB->GetOwner() : nullptr;

        if (!entityA || !entityB)
        {
            return 0.0f;
        }

        TransformComponent* transformA = entityA->GetComponent<TransformComponent>();

        TransformComponent* transformB = entityB->GetComponent<TransformComponent>();

        if (!transformA || !transformB)
        {
            return 0.0f;
        }

        constexpr float degreesToRadians = 0.017453292519943295f;

        const float relativeDegrees = transformB->GetWorldTransform().Rotation - transformA->GetWorldTransform().Rotation;

        return relativeDegrees * degreesToRadians - m_ReferenceAngle;
    }

    float RevoluteJoint2D::GetJointSpeed() const
    {
        const float angularVelocityA = m_BodyA ? m_BodyA->GetAngularVelocity() : 0.0f;

        const float angularVelocityB = m_BodyB ? m_BodyB->GetAngularVelocity() : 0.0f;

        return angularVelocityB - angularVelocityA;
    }

    void RevoluteJoint2D::Prepare(PhysicsWorld2D& world, float deltaTime)
    {
        m_Bias = {0.0f, 0.0f};

        if (!m_Enabled || deltaTime <= 0.0f)
        {
            return;
        }

        Entity* entityA = m_BodyA ? m_BodyA->GetOwner() : nullptr;

        Entity* entityB = m_BodyB ? m_BodyB->GetOwner() : nullptr;

        if (!entityA || !entityB)
        {
            return;
        }

        TransformComponent* transformA = entityA->GetComponent<TransformComponent>();

        TransformComponent* transformB = entityB->GetComponent<TransformComponent>();

        if (!transformA || !transformB)
        {
            return;
        }

        const Transform2D& worldA = transformA->GetWorldTransform();

        const Transform2D& worldB = transformB->GetWorldTransform();

        // WORLD LEVER ARMS

        m_Ra = Vector2::Rotate(m_LocalAnchorA, worldA.Rotation);

        m_Rb = Vector2::Rotate(m_LocalAnchorB, worldB.Rotation);

        // WORLD ANCHORS

        const Vector2 anchorA = worldA.Position + m_Ra;

        const Vector2 anchorB = worldB.Position + m_Rb;

        // SOLVER MASS

        const float inverseMassA = world.GetConstraintInverseMass(m_BodyA);

        const float inverseMassB = world.GetConstraintInverseMass(m_BodyB);

        const float inverseInertiaA = world.GetConstraintInverseInertia(m_BodyA);

        const float inverseInertiaB = world.GetConstraintInverseInertia(m_BodyB);

        const float totalInverseMass = inverseMassA + inverseMassB;

        Matrix2x2 K;

        K.M00 = totalInverseMass + inverseInertiaA * m_Ra.Y * m_Ra.Y + inverseInertiaB * m_Rb.Y * m_Rb.Y ;
        
        K.M01 = - inverseInertiaA * m_Ra.X * m_Ra.Y - inverseInertiaB * m_Rb.X * m_Rb.Y;

        K.M10 = K.M01;

        K.M11 = totalInverseMass + inverseInertiaA * m_Ra.X * m_Ra.X + inverseInertiaB * m_Rb.X * m_Rb.X;

        if (!K.Inverse(m_EffectiveMass))
        {
            m_EffectiveMass = Matrix2x2{0.0f, 0.0f, 0.0f, 0.0f};
        }

        // POSITION ERROR BIAS

        const Vector2 error = anchorB - anchorA;

        m_Bias = error * (m_BiasFactor / deltaTime);

        // -------------------------------------------------------------
        // MOTOR / LIMIT AXIAL SETUP
        // -------------------------------------------------------------
        //
        // The motor and both limits act on the single shared angular axis, so
        // they use a common scalar effective mass: 1 / (invIa + invIb).

        const float totalInverseInertia = inverseInertiaA + inverseInertiaB;

        constexpr float epsilon = 0.000001f;

        m_AxialMass = totalInverseInertia > epsilon ? 1.0f / totalInverseInertia : 0.0f;

        m_InverseDeltaTime = 1.0f / deltaTime;

        // Current relative joint angle in radians (relative to the reference
        // captured at construction).

        constexpr float degreesToRadians = 0.017453292519943295f;

        const float relativeDegrees = worldB.Rotation - worldA.Rotation;

        m_JointAngle = relativeDegrees * degreesToRadians - m_ReferenceAngle;

        // These axial constraints are not warm started across substeps; reset
        // their accumulators so clamping stays one-sided within this step.

        m_MotorImpulse = 0.0f;

        m_LowerImpulse = 0.0f;

        m_UpperImpulse = 0.0f;
    }

    void RevoluteJoint2D::WarmStart(PhysicsWorld2D& world)
    {
        if (!m_Enabled)
        {
            return;
        }

        const float inverseMassA = world.GetConstraintInverseMass(m_BodyA);

        const float inverseMassB = world.GetConstraintInverseMass(m_BodyB);

        const float inverseInertiaA = world.GetConstraintInverseInertia(m_BodyA);

        const float inverseInertiaB = world.GetConstraintInverseInertia(m_BodyB);

        Vector2 velocityA = m_BodyA ? m_BodyA->GetVelocity() : Vector2{0.0f, 0.0f};

        Vector2 velocityB = m_BodyB ? m_BodyB->GetVelocity() : Vector2{0.0f, 0.0f};

        float angularVelocityA = m_BodyA ? m_BodyA->GetAngularVelocity() : 0.0f;

        float angularVelocityB = m_BodyB ? m_BodyB->GetAngularVelocity() : 0.0f;

        const Vector2 impulse = m_AccumulatedImpulse;

        if (m_BodyA && inverseMassA > 0.0f)
        {
            velocityA -= impulse * inverseMassA;
        }

        if (m_BodyA && inverseInertiaA > 0.0f)
        {
            angularVelocityA -= world.Cross(m_Ra, impulse) * inverseInertiaA;
        }

        if (m_BodyB && inverseMassB > 0.0f)
        {
            velocityB += impulse * inverseMassB;
        }

        if (m_BodyB && inverseInertiaB > 0.0f)
        {
            angularVelocityB += world.Cross(m_Rb, impulse) * inverseInertiaB;
        }

        if (m_BodyA)
        {
            m_BodyA->SetVelocityFromPhysics(velocityA);

            m_BodyA->SetAngularVelocityFromPhysics(angularVelocityA);
        }

        if (m_BodyB)
        {
            m_BodyB->SetVelocityFromPhysics(velocityB);

            m_BodyB->SetAngularVelocityFromPhysics(angularVelocityB);
        }
    }

    void RevoluteJoint2D::SolveVelocity(PhysicsWorld2D& world)
    {
        if (!m_Enabled)
        {
            return;
        }

        const float inverseMassA = world.GetConstraintInverseMass(m_BodyA);

        const float inverseMassB = world.GetConstraintInverseMass(m_BodyB);

        const float inverseInertiaA = world.GetConstraintInverseInertia(m_BodyA);

        const float inverseInertiaB = world.GetConstraintInverseInertia(m_BodyB);

        Vector2 velocityA = m_BodyA ? m_BodyA->GetVelocity() : Vector2{0.0f, 0.0f};

        Vector2 velocityB = m_BodyB ? m_BodyB->GetVelocity() : Vector2{0.0f, 0.0f};

        float angularVelocityA = m_BodyA ? m_BodyA->GetAngularVelocity() : 0.0f;

        float angularVelocityB = m_BodyB ? m_BodyB->GetAngularVelocity() : 0.0f;

        // -------------------------------------------------------------
        // MOTOR
        //
        // Drive the relative angular velocity toward m_MotorSpeed. The total
        // accumulated motor impulse is clamped to +/- (maxTorque * dt) so the
        // motor can never exceed its torque budget.
        // -------------------------------------------------------------

        if (m_EnableMotor && m_AxialMass > 0.0f && m_InverseDeltaTime > 0.0f)
        {
            const float relativeAngularVelocity = angularVelocityB - angularVelocityA;

            float deltaImpulse = -m_AxialMass * (relativeAngularVelocity - m_MotorSpeed);

            const float oldImpulse = m_MotorImpulse;

            const float maxImpulse = m_MaxMotorTorque / m_InverseDeltaTime;

            m_MotorImpulse = std::clamp(oldImpulse + deltaImpulse, -maxImpulse, maxImpulse);

            deltaImpulse = m_MotorImpulse - oldImpulse;

            angularVelocityA -= inverseInertiaA * deltaImpulse;

            angularVelocityB += inverseInertiaB * deltaImpulse;
        }

        // -------------------------------------------------------------
        // LIMITS
        //
        // Two one-sided inequality constraints keep m_JointAngle within
        // [m_LowerAngle, m_UpperAngle]. The max(C, 0) * invDt term is a
        // speculative bias that begins arresting relative rotation just before
        // the limit is reached (anti-tunneling); residual penetration is
        // pushed out in SolvePosition.
        // -------------------------------------------------------------

        if (m_EnableLimit && m_AxialMass > 0.0f && m_InverseDeltaTime > 0.0f)
        {
            // LOWER LIMIT: jointAngle - lower >= 0

            {
                const float C = m_JointAngle - m_LowerAngle;

                const float relativeAngularVelocity = angularVelocityB - angularVelocityA;

                float deltaImpulse = -m_AxialMass * (relativeAngularVelocity + std::max(C, 0.0f) * m_InverseDeltaTime);

                const float oldImpulse = m_LowerImpulse;

                m_LowerImpulse = std::max(oldImpulse + deltaImpulse, 0.0f);

                deltaImpulse = m_LowerImpulse - oldImpulse;

                angularVelocityA -= inverseInertiaA * deltaImpulse;

                angularVelocityB += inverseInertiaB * deltaImpulse;
            }

            // UPPER LIMIT: upper - jointAngle >= 0 (signs mirrored)

            {
                const float C = m_UpperAngle - m_JointAngle;

                const float relativeAngularVelocity = angularVelocityA - angularVelocityB;

                float deltaImpulse = -m_AxialMass * (relativeAngularVelocity + std::max(C, 0.0f) * m_InverseDeltaTime);

                const float oldImpulse = m_UpperImpulse;

                m_UpperImpulse = std::max(oldImpulse + deltaImpulse, 0.0f);

                deltaImpulse = m_UpperImpulse - oldImpulse;

                angularVelocityA += inverseInertiaA * deltaImpulse;

                angularVelocityB -= inverseInertiaB * deltaImpulse;
            }
        }

        // PIVOT VELOCITIES

        const Vector2 pointVelocityA = world.GetConstraintPointVelocity(velocityA, angularVelocityA, m_Ra);

        const Vector2 pointVelocityB = world.GetConstraintPointVelocity(velocityB, angularVelocityB, m_Rb);

        const Vector2 constraintVelocity = pointVelocityB - pointVelocityA;

        // IMPULSE

        const Vector2 rhs = (constraintVelocity + m_Bias) * -1.0f;

        const Vector2 deltaImpulse = m_EffectiveMass.Multiply(rhs);

        m_AccumulatedImpulse += deltaImpulse;

        // APPLY BODY A

        if (m_BodyA && inverseMassA > 0.0f)
        {
            velocityA -= deltaImpulse * inverseMassA;
        }

        if (m_BodyA && inverseInertiaA > 0.0f)
        {
            angularVelocityA -= world.Cross(m_Ra, deltaImpulse) * inverseInertiaA;
        }

        // APPLY BODY B

        if (m_BodyB && inverseMassB > 0.0f)
        {
            velocityB += deltaImpulse * inverseMassB;
        }

        if (m_BodyB && inverseInertiaB > 0.0f)
        {
            angularVelocityB += world.Cross(m_Rb, deltaImpulse) * inverseInertiaB;
        }

        // WRITE BACK

        if (m_BodyA)
        {
            m_BodyA->SetVelocityFromPhysics(velocityA);

            m_BodyA->SetAngularVelocityFromPhysics(angularVelocityA);
        }

        if (m_BodyB)
        {
            m_BodyB->SetVelocityFromPhysics(velocityB);

            m_BodyB->SetAngularVelocityFromPhysics(angularVelocityB);
        }
    }

    bool RevoluteJoint2D::SolvePosition(PhysicsWorld2D& world)
    {
        if (!m_Enabled)
        {
            return false;
        }

        Entity* entityA = m_BodyA ? m_BodyA->GetOwner() : nullptr;

        Entity* entityB = m_BodyB ? m_BodyB->GetOwner() : nullptr;

        if (!entityA || !entityB)
        {
            return false;
        }

        TransformComponent* transformA = entityA->GetComponent<TransformComponent>();

        TransformComponent* transformB = entityB->GetComponent<TransformComponent>();

        if (!transformA || !transformB)
        {
            return false;
        }

        // -------------------------------------------------------------
        // LIMIT POSITION CORRECTION
        //
        // Push the joint angle back inside [lower, upper] when it has drifted
        // past a limit. This runs before (and independently of) the
        // point-constraint correction so a locked limit is still enforced when
        // the pivot itself is already within tolerance.
        // -------------------------------------------------------------

        bool corrected = false;

        if (m_EnableLimit)
        {
            const float inverseInertiaA = world.GetConstraintInverseInertia(m_BodyA);

            const float inverseInertiaB = world.GetConstraintInverseInertia(m_BodyB);

            const float totalInverseInertia = inverseInertiaA + inverseInertiaB;

            constexpr float epsilon = 0.000001f;

            if (totalInverseInertia > epsilon)
            {
                constexpr float degreesToRadians = 0.017453292519943295f;

                constexpr float radiansToDegrees = 57.29577951308232f;

                constexpr float angularSlop = 0.0349066f;        // ~2 degrees

                constexpr float maxAngularCorrection = 0.1396263f; // ~8 degrees

                const float relativeDegrees = transformB->GetWorldTransform().Rotation - transformA->GetWorldTransform().Rotation;

                const float jointAngle = relativeDegrees * degreesToRadians - m_ReferenceAngle;

                float C = 0.0f;

                if (jointAngle < m_LowerAngle)
                {
                    C = std::clamp(jointAngle - m_LowerAngle, -maxAngularCorrection, 0.0f);
                }
                else if (jointAngle > m_UpperAngle)
                {
                    C = std::clamp(jointAngle - m_UpperAngle, 0.0f, maxAngularCorrection);
                }

                if (std::abs(C) > angularSlop)
                {
                    // Angular impulse that removes the violation C, split
                    // between the two bodies by their inverse inertia.

                    const float angularImpulse = -C / totalInverseInertia;

                    if (inverseInertiaA > 0.0f)
                    {
                        transformA->RotateBy(-inverseInertiaA * angularImpulse * radiansToDegrees);
                    }

                    if (inverseInertiaB > 0.0f)
                    {
                        transformB->RotateBy(inverseInertiaB * angularImpulse * radiansToDegrees);
                    }

                    corrected = true;
                }
            }
        }

        const Transform2D& worldA = transformA->GetWorldTransform();

        const Transform2D& worldB = transformB->GetWorldTransform();

        const Vector2 rA = Vector2::Rotate(m_LocalAnchorA, worldA.Rotation);

        const Vector2 rB = Vector2::Rotate(m_LocalAnchorB, worldB.Rotation);

        const Vector2 anchorA = worldA.Position + rA;

        const Vector2 anchorB = worldB.Position + rB;

        Vector2 error = anchorB - anchorA;

        const float errorSquared = error.LengthSquared();

        constexpr float slop = 0.01f;

        if (errorSquared <= slop * slop)
        {
            // Pivot is already satisfied; report whether the limit block above
            // performed any correction this iteration.

            return corrected;
        }

        // LIMIT EXTREME CORRECTION

        constexpr float maxCorrection = 20.0f;

        const float errorLength = std::sqrt(errorSquared);

        if (errorLength > maxCorrection)
        {
            error *= maxCorrection / errorLength;
        }

        const float inverseMassA = world.GetConstraintInverseMass(m_BodyA);

        const float inverseMassB = world.GetConstraintInverseMass(m_BodyB);

        const float inverseInertiaA = world.GetConstraintInverseInertia(m_BodyA);

        const float inverseInertiaB = world.GetConstraintInverseInertia(m_BodyB);

        const float totalInverseMass = inverseMassA + inverseMassB;

        Matrix2x2 K;

        K.M00 = totalInverseMass + inverseInertiaA * rA.Y * rA.Y + inverseInertiaB * rB.Y * rB.Y ;
        
        K.M01 = - inverseInertiaA * rA.X * rA.Y - inverseInertiaB * rB.X * rB.Y;

        K.M10 = K.M01;

        K.M11 = totalInverseMass + inverseInertiaA * rA.X * rA.X + inverseInertiaB * rB.X * rB.X;

        Matrix2x2 inverseK;

        if (!K.Inverse(inverseK))
        {
            return false;
        }

        const Vector2 correctionImpulse = inverseK.Multiply(error * -1.0f);

        if (inverseMassA > 0.0f)
        {
            transformA->Translate(correctionImpulse * -inverseMassA);
        }

        if (inverseMassB > 0.0f)
        {
            transformB->Translate(correctionImpulse * inverseMassB);
        }

        const float angularCorrectionA = -world.Cross(rA, correctionImpulse) * inverseInertiaA;

        const float angularCorrectionB = world.Cross(rB, correctionImpulse) * inverseInertiaB;

        constexpr float radiansToDegrees = 57.29577951308232f;

        if (inverseInertiaA > 0.0f)
        {
            transformA->RotateBy(angularCorrectionA * radiansToDegrees);
        }

        if (inverseInertiaB > 0.0f)
        {
            transformB->RotateBy(angularCorrectionB * radiansToDegrees);
        }

        return true;
    }
}