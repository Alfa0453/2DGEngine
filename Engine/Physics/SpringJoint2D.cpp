#include "SpringJoint2D.h"

#include "PhysicsWorld2D.h"
#include "Rigidbody2D.h"

#include "../Scene/Entity.h"
#include "../Scene/TransformComponent.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace Engine
{
    SpringJoint2D::SpringJoint2D(Rigidbody2D* bodyA, Rigidbody2D* bodyB, const Vector2& localAnchorA, const Vector2& localAnchorB, float resetLength)
        : Joint2D(bodyA, bodyB), m_LocalAnchorA(localAnchorA), m_LocalAnchorB(localAnchorB)
    {
        SetResetLength(resetLength);
    }

    void SpringJoint2D::SetLocalAnchorA(const Vector2& anchor)
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

    const Vector2& SpringJoint2D::GetLocalAnchorA() const
    {
        return m_LocalAnchorA;
    }

    void SpringJoint2D::SetLocalAnchorB(const Vector2& anchor)
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

    const Vector2& SpringJoint2D::GetLocalAnchorB() const
    {
        return m_LocalAnchorB;
    }

    void SpringJoint2D::SetResetLength(float length)
    {
        m_ResetLength = std::max(0.0f, length);

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

    float SpringJoint2D::GetResetLength() const
    {
        return m_ResetLength;
    }

    void SpringJoint2D::SetStiffness(float stiffness)
    {
        m_Stiffness = std::max(0.0f, stiffness);

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

    float SpringJoint2D::GetStiffness() const
    {
        return m_Stiffness;
    }

    void SpringJoint2D::SetDamping(float damping)
    {
        m_Damping = std::max(0.0f, damping);

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

    float SpringJoint2D::GetDamping() const
    {
        return m_Damping;
    }

    void SpringJoint2D::Prepare(PhysicsWorld2D& world, float deltaTime)
    {
        m_EffectiveMass = 0.0f;

        m_Gamma = 0.0f;

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

        Vector2 delta = anchorB - anchorA;

        const float distanceSquared = delta.LengthSquared();

        constexpr float epsilon = 0.000001f;

        if (m_Stiffness <= epsilon && m_Damping <= epsilon)
        {
            m_EffectiveMass = 0.0f;

            m_Gamma = 0.0f;

            m_Bias = 0.0f;

            m_AccumulatedImpulse = 0.0f;

            return;
        }

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

        // BODY MASS PROPERTIES

        const float inverseMassA = world.GetConstraintInverseMass(m_BodyA);

        const float inverseMassB = world.GetConstraintInverseMass(m_BodyB);

        const float inverseInertiaA = world.GetConstraintInverseInertia(m_BodyA);

        const float inverseInertiaB = world.GetConstraintInverseInertia(m_BodyB);

        const float raCrossN = world.Cross(m_Ra, m_Direction);

        const float rbCrossN = world.Cross(m_Rb, m_Direction);

        float inverseEffectiveMass = 
            inverseMassA + inverseMassB + raCrossN * raCrossN * inverseInertiaA + 
            rbCrossN * rbCrossN * inverseInertiaB;

        // SPRING SOFTNESS

        const float softnessDenominator = deltaTime * (m_Damping + deltaTime * m_Stiffness);

        if (softnessDenominator > epsilon)
        {
            m_Gamma = 1.0f / softnessDenominator;
        }

        // POSITION ERROR

        const float error = currentLength - m_ResetLength;

        m_Bias = error * deltaTime * m_Stiffness * m_Gamma;

        // SOFT EFFECTIVE MASS

        inverseEffectiveMass += m_Gamma;

        if (inverseEffectiveMass > epsilon)
        {
            m_EffectiveMass = 1.0f / inverseEffectiveMass;
        }
    }

    void SpringJoint2D::WarmStart(PhysicsWorld2D& world)
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

    void SpringJoint2D::SolveVelocity(PhysicsWorld2D& world)
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

        const float velocityAlongSpring = Vector2::Dot(relativeVelocity, m_Direction);

        // SOFT CONSTRAINT IMPULSE

        const float delaImpulse = -m_EffectiveMass * (velocityAlongSpring + m_Bias + m_Gamma * m_AccumulatedImpulse);

        m_AccumulatedImpulse += delaImpulse;

        const Vector2 impulse = m_Direction * delaImpulse;

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

    bool SpringJoint2D::SolvePosition(PhysicsWorld2D& world)
    {
        return false;
    }

    float SpringJoint2D::GetCurrentLength() const
    {
        if (!m_BodyA || !m_BodyB)
        {
            return 0.0f;
        }

        Entity* entityA = m_BodyA->GetOwner();

        Entity* entityB = m_BodyB->GetOwner();

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

        const Transform2D& worldA = transformA->GetWorldTransform();

        const Transform2D& worldB = transformB->GetWorldTransform();

        const Vector2 rA = Vector2::Rotate(m_LocalAnchorA, worldA.Rotation);

        const Vector2 rB = Vector2::Rotate(m_LocalAnchorB, worldB.Rotation);

        const Vector2 anchorA = worldA.Position + rA;

        const Vector2 anchorB = worldB.Position + rB;

        const Vector2 delta = anchorB - anchorA;

        return std::sqrt(delta.LengthSquared());
    }

    float SpringJoint2D::GetExtension() const
    {
        return GetCurrentLength() - m_ResetLength;
    }
}