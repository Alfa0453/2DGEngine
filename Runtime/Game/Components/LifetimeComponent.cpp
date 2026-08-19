#include "LifetimeComponent.h"

#include "../../../Engine/Scene/Entity.h"

#include <iostream>
#include <string>

LifetimeComponent::LifetimeComponent(float lifetime)
    : m_RemainingTime(lifetime)
{
}

void LifetimeComponent::Update(float deltaTime)
{
    m_RemainingTime -= deltaTime;

    if (m_RemainingTime <= 0.0f)
    {
        Engine::Entity* owner = GetOwner();

        if (owner)
        {
            std::cout << "Destoy called\n";
            owner->Destroy();
        }
    }
}