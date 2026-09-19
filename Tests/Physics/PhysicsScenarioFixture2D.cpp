#include "PhysicsScenarioFixture2D.h"

#include <cmath>

namespace Tests
{
    PhysicsScenarioFixture2D::PhysicsScenarioFixture2D()
        : m_Physics(&m_Scene)
    {
    }

    Engine::Scene& PhysicsScenarioFixture2D::GetScene()
    {
        return m_Scene;
    }

    Engine::PhysicsWorld2D& PhysicsScenarioFixture2D::GetPhysics()
    {
        return m_Physics;
    }

    void PhysicsScenarioFixture2D::Step(std::size_t stepCount)
    {
        const float dt = m_Physics.GetSettings().FixedDeltaTime;

        for (std::size_t i = 0; i < stepCount; ++i)
        {
            m_Physics.Update(dt);
        }
    }

    void PhysicsScenarioFixture2D::StepSeconds(float seconds)
    {
        if (seconds <= 0.0f)
        {
            return;
        }

        const float dt = m_Physics.GetSettings().FixedDeltaTime;

        const std::size_t steps = static_cast<std::size_t>(std::ceil(seconds / dt));

        Step(steps);
    }
}