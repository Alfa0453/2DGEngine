#include "DistanceJoint2D.h"

#include "PhysicsWorld2D.h"
#include "Rigidbody2D.h"

#include "../Scene/Entity.h"
#include "../Scene/TransformComponent.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace Engine
{
    DistanceJoint2D::DistanceJoint2D(Rigidbody2D* bodyA, Rigidbody2D* bodyB, const Vector2& localAnchorA, const Vector2& localAnchorB, float targetLength)
        : Joint2D(bodyA, bodyB), m_LocalAnchorA(localAnchorA), m_LocalAnchorB(localAnchorB)
    {
        SetTargetLength(targetLength);
    }

    void DistanceJoint2D::SetLocalAnchorA(const Vector2& anchor)
    {
        m_LocalAnchorA = anchor;

        m_AccumulatedImpulse = 0.0f;

        if (m_BodyA)
        {
            m_BodyA->Wake();
        }

        if (m_BodyB)
        {
            m_BodyB->Wake();
        }
    }

    const Vector2& DistanceJoint2D::GetLocalAnchorA() const
    {
        return m_LocalAnchorA;
    }

    void DistanceJoint2D::SetLocalAnchorB(const Vector2& anchor)
    {
        m_LocalAnchorB = anchor;

        m_AccumulatedImpulse = 0.0f;

        if (m_BodyA)
        {
            m_BodyA->Wake();
        }

        if (m_BodyB)
        {
            m_BodyB->Wake();
        }
    }

    const Vector2& DistanceJoint2D::GetLocalAnchorB() const
    {
        return m_LocalAnchorB;
    }

    void DistanceJoint2D::SetTargetLength(float length)
    {
        m_TargetLength = std::max(0.0f, length);

        m_AccumulatedImpulse = 0.0f;

        if (m_BodyA)
        {
            m_BodyA->Wake();
        }

        if (m_BodyB)
        {
            m_BodyB->Wake();
        }
    }

    float DistanceJoint2D::GetTargetLength() const
    {
        return m_TargetLength;
    }

    void DistanceJoint2D::SetBiasFactor(float biasFactor)
    {
        m_BiasFactor = std::max(0.0f, biasFactor);
    }

    float DistanceJoint2D::GetBiasFactor() const
    {
        return m_BiasFactor;
    }

    void DistanceJoint2D::Prepare(PhysicsWorld2D& world, float deltaTime)
    {
        m_EffectiveMass = 0.0f;

        m_Bias = 0.0f;

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

        // JOINT AXIS

        Vector2 delta = anchorB - anchorA;

        const float distanceSquared = delta.LengthSqured();

        constexpr float epsilon = 0.000001f;

        float currentLength = 0.0f;

        if (distanceSquared > epsilon)
        {
            currentLength = std::sqrt(distanceSquared);

            m_Direction = delta * (1.0f / currentLength);
        }
        else 
        {
            m_Direction = {1.0f, 0.0f};
        }

        // EFFECTIVE MASS

        const float inverseMassA = world.GetConstraintInverseMass(m_BodyA);

        const float inverseMassB = world.GetConstraintInverseMass(m_BodyB);

        const float inverseInertiaA = world.GetConstraintInverseInertia(m_BodyA);

        const float inverseInertiaB = world.GetConstraintInverseInertia(m_BodyB);

        const float raCrossN = world.Cross(m_Ra, m_Direction);

        const float rbCrossN = world.Cross(m_Rb, m_Direction);

        const float denominator = 
            inverseMassA + inverseMassB + raCrossN * raCrossN * 
            inverseInertiaA + rbCrossN * rbCrossN * inverseInertiaB;

        if (denominator > epsilon)
        {
            m_EffectiveMass = 1.0f / denominator;
        }

        // POSITION ERROR BIAS

        const float error = currentLength - m_TargetLength;

        m_Bias = m_BiasFactor * error / deltaTime;
    }

    void DistanceJoint2D::WarmStart(PhysicsWorld2D& world)
    {
        if (!m_Enabled || std::abs(m_AccumulatedImpulse) <= 0.000001f)
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

        const Vector2 impulse = m_Direction * m_AccumulatedImpulse;

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

    void DistanceJoint2D::SolveVelocity(PhysicsWorld2D& world)
    {
        if (!m_Enabled || m_EffectiveMass <= 0.0f)
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

        // ANCHOR POINT VELOCITIES

        const Vector2 anchorVelocityA = world.GetConstraintPointVelocity(velocityA, angularVelocityA, m_Ra);

        const Vector2 anchorVelocityB = world.GetConstraintPointVelocity(velocityB, angularVelocityB, m_Rb);

        const Vector2 relativeVelocity = anchorVelocityB - anchorVelocityA;

        const float constraintVelocity = Vector2::Dot(relativeVelocity, m_Direction);

        // SOLVER IMPULSE

        const float deltaImpulse = -m_EffectiveMass * (constraintVelocity + m_Bias);

        m_AccumulatedImpulse += deltaImpulse;

        const Vector2 impulse = m_Direction * deltaImpulse;

        // BODY A

        if (m_BodyA && inverseMassA > 0.0f)
        {
            velocityA -= impulse * inverseMassA;
        }

        if (m_BodyA && inverseInertiaA > 0.0f)
        {
            angularVelocityA -= world.Cross(m_Ra, impulse) * inverseInertiaA;
        }

        // BODY B

        if (m_BodyB && inverseMassB > 0.0f)
        {
            velocityB += impulse * inverseMassB;
        }

        if (m_BodyB && inverseInertiaB > 0.0f)
        {
            angularVelocityB += world.Cross(m_Rb, impulse) * inverseInertiaB;
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

    bool DistanceJoint2D::SolvePosition(PhysicsWorld2D& world)
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

        Vector2 delta = anchorB - anchorA;

        const float distanceSquared = delta.LengthSqured();

        constexpr float epsilon = 0.000001f;

        if (distanceSquared <= epsilon)
        {
            return false;
        }

        const float currentLength = std::sqrt(distanceSquared);

        const Vector2 direction = delta * (1.0f / currentLength);

        float error = currentLength - m_TargetLength;

        constexpr float slop = 0.01f;

        if (std::abs(error) <= slop)
        {
            return false;
        }

        // Prevent huge position corrections.
        constexpr float maxCorrection = 20.0f;

        error = std::clamp(error, -maxCorrection, maxCorrection);

        const float inverseMassA = world.GetConstraintInverseMass(m_BodyA);

        const float inverseMassB = world.GetConstraintInverseMass(m_BodyB);

        const float inverseInertiaA = world.GetConstraintInverseInertia(m_BodyA);

        const float inverseInertiaB = world.GetConstraintInverseInertia(m_BodyB);

        const float raCrossN = world.Cross(rA, direction);

        const float rbCrossN = world.Cross(rB, direction);

        const float denominator = 
            inverseMassA + inverseMassB + raCrossN * raCrossN * inverseInertiaA +
            rbCrossN * rbCrossN * inverseInertiaB;

        if (denominator <= epsilon)
        {
            return false;
        }

        const float impulseMagnitude = -error / denominator;

        const Vector2 correctionImpulse = direction * impulseMagnitude;

        // POSITION A

        if (inverseMassA > 0.0f)
        {
            transformA->Translate(correctionImpulse * -inverseMassA);
        }

        // POSITION B

        if (inverseMassB > 0.0f)
        {
            transformB->Translate(correctionImpulse * inverseMassB);
        }

        return true;
    }
}