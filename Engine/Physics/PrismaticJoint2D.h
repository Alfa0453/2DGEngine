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
    };
}