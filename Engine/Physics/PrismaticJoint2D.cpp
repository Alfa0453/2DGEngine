#include "PrismaticJoint2D.h"

#include "PhysicsWorld2D.h"
#include "Rigidbody2D.h"

#include "../Scene/Entity.h"
#include "../Scene/TransformComponent.h"

#include <algorithm>
#include <cmath>

namespace Engine
{
    PrismaticJoint2D::PrismaticJoint2D(Rigidbody2D* bodyA, Rigidbody2D* bodyB, const Vector2& localAnchorA, const Vector2& localAnchorB, const Vector2& localAxisA)
        : Joint2D(bodyA, bodyB), m_LocalAnchorA(localAnchorA), m_LocalAnchorB(localAnchorB)
    {
        SetLocalAxisA(localAxisA);

        Entity* entityA = m_BodyA ? m_BodyA->GetOwner() : nullptr;

        Entity* entityB = m_BodyB ? m_BodyB->GetOwner() : nullptr;

        if (entityA && entityB)
        {
            TransformComponent* transformA = entityA->GetComponent<TransformComponent>();

            TransformComponent* transformB = entityB->GetComponent<TransformComponent>();

            if (transformA && transformB)
            {
                m_ReferenceAngle = transformB->GetWorldTransform().Rotation - transformA->GetWorldTransform().Rotation;
            }
        }
    }

    void PrismaticJoint2D::SetLocalAnchorA(const Vector2& anchor)
    {
        m_LocalAnchorA = anchor;

        m_AccumulatedLinearImpulse = 0.0f;

        m_AccumulatedAngularImpulse = 0.0f;

        if (m_BodyA)
        {
            m_BodyA->Wake();
        }

        if (m_BodyB)
        {
            m_BodyB->Wake();
        }
    }

    const Vector2& PrismaticJoint2D::GetLocalAnchorA() const
    {
        return m_LocalAnchorA;
    }

    void PrismaticJoint2D::SetLocalAnchorB(const Vector2& anchor)
    {
        m_LocalAnchorB = anchor;

        m_AccumulatedLinearImpulse = 0.0f;

        m_AccumulatedAngularImpulse = 0.0f;

        if (m_BodyA)
        {
            m_BodyA->Wake();
        }

        if (m_BodyB)
        {
            m_BodyB->Wake();
        }
    }

    const Vector2& PrismaticJoint2D::GetLocalAnchorB() const
    {
        return m_LocalAnchorB;
    }

    void PrismaticJoint2D::SetLocalAxisA(const Vector2& axis)
    {
        const float lengthSquared = axis.LengthSqured();

        constexpr float epsilon = 0.000001f;

        if (lengthSquared <= epsilon)
        {
            m_LocalAxisA = {1.0f, 0.0f};
        }
        else 
        {
            m_LocalAxisA = axis * (1.0f / std::sqrt(lengthSquared));
        }

        m_AccumulatedLinearImpulse = 0.0f;

        m_AccumulatedAngularImpulse = 0.0f;

        if (m_BodyA)
        {
            m_BodyA->Wake();
        }

        if (m_BodyB)
        {
            m_BodyB->Wake();
        }
    }

    const Vector2& PrismaticJoint2D::GetLocalAxisA() const
    {
        return m_LocalAxisA;
    }

    void PrismaticJoint2D::SetBiasFactor(float biasFactor)
    {
        m_BiasFactor = std::max(0.0f, biasFactor);
    }

    float PrismaticJoint2D::GetBiasFactor() const
    {
        return m_BiasFactor;
    }

    void PrismaticJoint2D::Prepare(PhysicsWorld2D& world, float deltaTime)
    {
        m_LinearEffectiveMass = 0.0f;

        m_AngularEffectiveMass = 0.0f;

        m_LinearBias = 0.0f;

        m_AngularBias = 0.0f;

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

        // WORLD ANCHOR LEVER ARMS

        m_Ra = Vector2::Rotate(m_LocalAnchorA, worldA.Rotation);

        m_Rb = Vector2::Rotate(m_LocalAnchorB, worldB.Rotation);

        // WORLD AXIS

        m_WorldAxis = Vector2::Rotate(m_LocalAxisA, worldA.Rotation);

        // PERPENDICULAR AXIS

        m_WorldPerpendicular = {-m_WorldAxis.Y, m_WorldAxis.X};

        // WORLD ANCHORS

        const Vector2 anchorA = worldA.Position + m_Ra;

        const Vector2 anchorB = worldB.Position + m_Rb;

        const Vector2 delta = anchorB - anchorA;

        // MASS / INERTIA

        const float inverseMassA = world.GetConstraintInverseMass(m_BodyA);

        const float inverseMassB = world.GetConstraintInverseMass(m_BodyB);

        const float inverseInertiaA = world.GetConstraintInverseInertia(m_BodyA);

        const float inverseInertiaB = world.GetConstraintInverseInertia(m_BodyB);

        // PERPENDICULAR LINEAR EFFECTIVE MASS

        const float raCrossPrep = world.Cross(m_Ra, m_WorldPerpendicular);

        const float rbCrossPrep = world.Cross(m_Rb, m_WorldPerpendicular);

        const float linearDenominator = 
            inverseMassA + inverseMassB + raCrossPrep * raCrossPrep * inverseInertiaA + 
            rbCrossPrep * rbCrossPrep * inverseInertiaB;

        constexpr float epsilon = 0.000001f;

        if (linearDenominator > epsilon)
        {
            m_LinearEffectiveMass = 1.0f / linearDenominator;
        }

        // ANGULAR EFFECTIVE MASS

        const float angularDenominator = inverseInertiaA + inverseInertiaB;

        if (angularDenominator > epsilon)
        {
            m_AngularEffectiveMass = 1.0f / angularDenominator;
        }

        // POSITION ERROR

        const float perpendicularError = Vector2::Dot(delta, m_WorldPerpendicular);

        m_LinearBias = m_BiasFactor * perpendicularError / deltaTime;

        // ANGULAR ERROR
        //
        // Transform rotation are degrees.
        // Physics angular velocity id radians/sec.
        // Convert angle error to radians.

        constexpr float degreesToRadians = 0.017453292519943295f;

        const float currentRelativeAngleDegrees = worldB.Rotation - worldA.Rotation;

        const float angleErrorDegrees = currentRelativeAngleDegrees - m_ReferenceAngle;

        const float angleErrorRadians = angleErrorDegrees * degreesToRadians;

        m_AngularBias = m_BiasFactor * angleErrorRadians / deltaTime;
    }

    void PrismaticJoint2D::WarmStart(PhysicsWorld2D& world)
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

        const Vector2 linearImpulse = m_WorldPerpendicular * m_AccumulatedLinearImpulse;

        // BODY A

        if (m_BodyA && inverseMassA > 0.0f)
        {
            velocityA -= linearImpulse * inverseMassA;
        }

        if (m_BodyA && inverseInertiaA > 0.0f)
        {
            angularVelocityA -= (world.Cross(m_Ra, linearImpulse) + m_AccumulatedAngularImpulse) * inverseInertiaA;
        }

        // BODY B

        if (m_BodyB && inverseMassB > 0.0f)
        {
            velocityB += linearImpulse * inverseMassB;
        }

        if (m_BodyB && inverseInertiaB > 0.0f)
        {
            angularVelocityB += (world.Cross(m_Rb, linearImpulse) + m_AccumulatedAngularImpulse) * inverseInertiaB;
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

    void PrismaticJoint2D::SolveVelocity(PhysicsWorld2D& world)
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

        // PERPENDICULAR LINEAR CONSTRAINT

        if (m_LinearEffectiveMass > 0.0f)
        {
            const Vector2 pointVelocityA = world.GetConstraintPointVelocity(velocityA, angularVelocityA, m_Ra);

            const Vector2 pointVelocityB = world.GetConstraintPointVelocity(velocityB, angularVelocityB, m_Rb);

            const Vector2 relativePointVelocity = pointVelocityB - pointVelocityA;

            const float velocityAlongPerpendicular = Vector2::Dot(relativePointVelocity, m_WorldPerpendicular);

            const float deltaLinearImpulse = -m_LinearEffectiveMass * (velocityAlongPerpendicular + m_LinearBias);

            m_AccumulatedLinearImpulse += deltaLinearImpulse;

            const Vector2 impulse = m_WorldPerpendicular * deltaLinearImpulse;

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
        }

        // ANGULAR CONSTRAINT

        if (m_AngularEffectiveMass > 0.0f)
        {
            const float relativeAngularVelocity = angularVelocityB - angularVelocityA;

            const float deltaAngularImpulse = -m_AngularEffectiveMass * (relativeAngularVelocity + m_AngularBias);

            m_AccumulatedAngularImpulse += deltaAngularImpulse;

            if (m_BodyA && inverseInertiaA > 0.0f)
            {
                angularVelocityA -= deltaAngularImpulse * inverseInertiaA;
            }

            if (m_BodyB && inverseInertiaB > 0.0f)
            {
                angularVelocityB += deltaAngularImpulse * inverseInertiaB;
            }
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

    bool PrismaticJoint2D::SolvePosition(PhysicsWorld2D& world)
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

        const Transform2D& worldA = transformA->GetWorldTransform();

        const Transform2D& worldB = transformB->GetWorldTransform();

        // CURRENT WORLD AXIS

        const Vector2 axis = Vector2::Rotate(m_LocalAxisA, worldA.Rotation);

        const Vector2 perpendicular{-axis.Y, axis.X};

        // CURRENT WORLD ANCHORS

        const Vector2 rA = Vector2::Rotate(m_LocalAnchorA, worldA.Rotation);

        const Vector2 rB = Vector2::Rotate(m_LocalAnchorB, worldB.Rotation);

        const Vector2 anchorA = worldA.Position + rA;

        const Vector2 anchorB = worldB.Position + rB;

        const Vector2 delta = anchorB - anchorA;

        const float perpendicularError = Vector2::Dot(delta, perpendicular);

        // ANGULAR ERROR

        float angleErrorDegrees = (worldB.Rotation - worldA.Rotation) - m_ReferenceAngle;

        // ERROR THRESHOLDS

        constexpr float linearSlop = 0.01f;

        constexpr float angularSlopDegrees = 0.25f;

        const bool linearNeedsCorrection = std::abs(perpendicularError) > linearSlop;

        const bool angularNeedsCorrection = std::abs(angleErrorDegrees) > angularSlopDegrees;

        if (!linearNeedsCorrection && !angularNeedsCorrection)
        {
            return false;
        }

        const float inverseMassA = world.GetConstraintInverseMass(m_BodyA);

        const float inverseMassB = world.GetConstraintInverseMass(m_BodyB);

        const float inverseInertiaA = world.GetConstraintInverseInertia(m_BodyA);

        const float inverseInertiaB = world.GetConstraintInverseInertia(m_BodyB);

        // LINEAR POSITION CORRCTION

        if (linearNeedsCorrection)
        {
            constexpr float maxLinearCorrection = 20.0f;

            const float clampedError = std::clamp(perpendicularError, -maxLinearCorrection, maxLinearCorrection);

            const float raCrossPrep = world.Cross(rA, perpendicular);

            const float rbCrossPrep = world.Cross(rB, perpendicular);

            const float denominator = 
                inverseMassA + inverseMassB + raCrossPrep * raCrossPrep * inverseInertiaA + 
                rbCrossPrep * rbCrossPrep * inverseInertiaB;

            constexpr float epsilon = 0.000001f;

            if (denominator > epsilon)
            {
                const float impulseMagnitude = -clampedError / denominator;

                const Vector2 correction = perpendicular * impulseMagnitude;

                if (inverseMassA > 0.0f)
                {
                    transformA->Translate(correction * -inverseMassA);
                }

                if (inverseInertiaB > 0.0f)
                {
                    transformB->Translate(correction * inverseMassB);
                }
            }
        }

        // ANGULAR POSITION CORRECTION

        if (angularNeedsCorrection)
        {
            constexpr float maxAngularCorrectionDegrees = 10.0f;

            angleErrorDegrees = std::clamp(angleErrorDegrees, -maxAngularCorrectionDegrees, maxAngularCorrectionDegrees);

            const float totalInverseInertia = inverseInertiaA + inverseInertiaB;

            constexpr float epsilon = 0.000001f;

            if (totalInverseInertia > epsilon)
            {
                const float correction = -angleErrorDegrees / totalInverseInertia;

                if (inverseInertiaA > 0.0f)
                {
                    transformA->RotateBy(-correction * inverseInertiaA);
                }

                if (inverseInertiaB > 0.0f)
                {
                    transformB->RotateBy(correction * inverseInertiaB);
                }
            }
        }

        return true;
    }

    float PrismaticJoint2D::GetTranslation() const
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

        const Transform2D& worldA = transformA->GetWorldTransform();

        const Transform2D& worldB = transformB->GetWorldTransform();

        const Vector2 rA = Vector2::Rotate(m_LocalAnchorA, worldA.Rotation);

        const Vector2 rB = Vector2::Rotate(m_LocalAnchorB, worldB.Rotation);

        const Vector2 anchorA = worldA.Position + rA;

        const Vector2 anchorB = worldB.Position + rB;

        const Vector2 axis = Vector2::Rotate(m_LocalAxisA, worldA.Rotation);

        return Vector2::Dot(anchorB - anchorA, axis);
    }
}