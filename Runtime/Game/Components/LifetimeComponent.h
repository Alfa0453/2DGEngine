#pragma once

#include "../../../Engine/Scene/Component.h"

class LifetimeComponent : public Engine::Component
{
public:
    explicit LifetimeComponent(float lifetime);

    void Update(float deltaTime) override;

private:
    float m_RemainingTime = 0.0f;
};