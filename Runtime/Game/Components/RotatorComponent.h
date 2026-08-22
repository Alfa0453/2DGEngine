#pragma once

#include "../../../Engine/Scene/Component.h"
#include "../../../Engine/Scene/Entity.h"
#include "../../../Engine/Scene/TransformComponent.h"

#include <iostream>


class RotatorComponent : public Engine::Component
{
public:

    void Start() override
    {
        std::cout << "Rotator started\n";
    }

    void Update(float deltaTime) override
    {
        Engine::Entity* owner = GetOwner();

        if (!owner)
        {
            return;
        }

        auto* transform = owner->GetComponent<Engine::TransformComponent>();

        if (!transform)
        {
            return;
        }

        transform->RotateBy(90.0f * deltaTime);
    }

    void OnDestroy() override
    {
        std::cout << "Rotator destroyed\n";
    }
};