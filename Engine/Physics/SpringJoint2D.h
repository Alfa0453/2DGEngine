#pragma once

#include "Joint2D.h"

#include "../Math/Vector2.h"

namespace Engine
{
    class SpringJoint2D final : public Joint2D
    {
    public:

        SpringJoint2D(Rigidbody2D* bodyA, Rigidbody2D* bodyB, const Vector2& localAnchorA, const Vector2& localAnchorB, float restLength);

        void SetLocalAnchorA(const Vector2& anchor);

        const Vector2& GetLocalAnchorA() const;

        void SetLocalAnchorB(const Vector2& anchor);

        const Vector2& GetLocalAnchorB() const;

        void SetResetLength(float length);

        float GetResetLength() const;

        void SetStiffness(float stiffness);

        float GetStiffness() const;

        void SetDamping(float damping);

        float GetDamping() const;

        void Prepare(PhysicsWorld2D& world, float deltaTime) override;

        void WarmStart(PhysicsWorld2D& world) override;

        void SolveVelocity(PhysicsWorld2D& world) override;

        bool SolvePosition(PhysicsWorld2D& world) override;

        float GetCurrentLength() const;

        float GetExtension() const;

    private:

        Vector2 m_LocalAnchorA{0.0f, 0.0f};

        Vector2 m_LocalAnchorB{0.0f, 0.0f};

        Vector2 m_Ra{0.0f, 0.0f};

        Vector2 m_Rb{0.0f, 0.0f};

        Vector2 m_Direction{1.0f, 0.0f};

        float m_ResetLength = 0.0f;

        float m_Stiffness = 20.0f;

        float m_Damping = 5.0f;

        float m_EffectiveMass = 0.0f;

        float m_Gamma = 0.0f;

        float m_Bias = 0.0f;

        float m_AccumulatedImpulse = 0.0f;
    };
}