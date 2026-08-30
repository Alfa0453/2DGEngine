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
    };
}