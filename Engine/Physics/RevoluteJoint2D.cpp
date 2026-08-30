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

        const Transform2D& worldA = transformA->GetWorldTransform();

        const Transform2D& worldB = transformB->GetWorldTransform();

        const Vector2 rA = Vector2::Rotate(m_LocalAnchorA, worldA.Rotation);

        const Vector2 rB = Vector2::Rotate(m_LocalAnchorB, worldB.Rotation);

        const Vector2 anchorA = worldA.Position + rA;

        const Vector2 anchorB = worldB.Position + rB;

        Vector2 error = anchorB - anchorA;

        const float errorSquared = error.LengthSqured();

        constexpr float slop = 0.01f;

        if (errorSquared <= slop * slop)
        {
            return false;
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