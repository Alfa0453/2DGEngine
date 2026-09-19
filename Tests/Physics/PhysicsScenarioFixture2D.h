#pragma once

#include "../../Engine/Scene/Scene.h"
#include "../../Engine/Physics/PhysicsWorld2D.h"

#include <memory>

namespace Tests
{
    class PhysicsScenarioFixture2D
    {
    public:

        PhysicsScenarioFixture2D();

        Engine::Scene& GetScene();

        Engine::PhysicsWorld2D& GetPhysics();

        void Step(std::size_t stepCount = 1);

        void StepSeconds(float seconds);

    private:
        
        Engine::Scene m_Scene;

        Engine::PhysicsWorld2D m_Physics;
    };
}