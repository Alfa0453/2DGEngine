#pragma once

namespace Engine
{
    class PhysicsWorld2D;
    class Rigidbody2D;

    class Joint2D
    {
    public:

        Joint2D(Rigidbody2D* bodyA, Rigidbody2D* bodyB)
            : m_BodyA(bodyA), m_BodyB(bodyB)
        {
        }

        virtual ~Joint2D() = default;

        Rigidbody2D* GetBodyA() const
        {
            return m_BodyA;
        }

        Rigidbody2D* GetBodyB() const
        {
            return m_BodyB;
        }

        void SetEnabled(bool enabled)
        {
            m_Enabled = enabled;
        }

        bool IsEnabled() const
        {
            return m_Enabled;
        }

        // SOLVER LIFECYCLE

        virtual void Prepare(PhysicsWorld2D& world, float deltaTime) = 0;

        virtual void WarmStart(PhysicsWorld2D& world) = 0;

        virtual void SolveVelocity(PhysicsWorld2D& world) = 0;

        virtual bool SolvePosition(PhysicsWorld2D& world) = 0;

    protected:

        Rigidbody2D* m_BodyA = nullptr;

        Rigidbody2D* m_BodyB = nullptr;

        bool m_Enabled = true;
    };
}