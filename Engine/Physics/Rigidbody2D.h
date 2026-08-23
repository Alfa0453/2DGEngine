#pragma once

#include "BodyType2D.h"
#include "../Scene/Component.h"
#include "../Math/Vector2.h"

namespace Engine
{
    class Rigidbody2D : public Component
    {
    public:

        Rigidbody2D() = default;

        void SetBodyType(BodyType2D type);

        BodyType2D GetBodyType() const;

        void SetVelocity(const Vector2& velocity);

        const Vector2& GetVelocity() const;

        void AddVelocity(const Vector2& deltaVelocity);

        void SetMass(float mass);

        float GetMass() const;

        float GetInverseMass() const;

        void SetGravityScale(float gravityScale);

        float GetGravityScale() const;

        void SetLinearDamping(float damping);

        float GetLinearDamping() const;

        void AddForce(const Vector2& force);

        void AddImpulse(const Vector2& impulse);

        void ClearForces();

        const Vector2& GetAccumulatedForce() const;

        bool IsStatic() const;

        bool IsKinematic() const;

        bool IsDynamic() const;

    private:
        
        void RecalculateInverseMass();

    private:

        BodyType2D m_BodyType = BodyType2D::Dynamic;

        Vector2 m_Velocity{0.0f, 0.0f};

        Vector2 m_AccumulatedForce{0.0f, 0.0f};

        float m_Mass = 1.0f;

        float m_InverseMass = 1.0f;

        float m_GravityScale = 1.0f;

        float m_LinearDamping = 0.0f;
    };
}