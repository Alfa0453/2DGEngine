#pragma once

#include "../../../Engine/Scene/Component.h"

namespace Engine
{
    class Input;
    class TransformComponent;
    class SpriteRendererComponent;
    class AnimatorComponent;
}

class PlayerControllerComponent : public Engine::Component
{
public:
    PlayerControllerComponent() = default;

    explicit PlayerControllerComponent(Engine::Input* input);

    PlayerControllerComponent(Engine::Input* input, float movementSpeed);

    void Start() override;

    void Update(float deltaTime) override;

    void SetMovementSpeed(float speed);

    float GetMovementSpeed() const;

    void SetSprintMultiplier(float multiplier);

    float GetSprintMultiplier() const;

private:
    Engine::Input* m_Input = nullptr;

    Engine::TransformComponent* m_Transform = nullptr;

    Engine::SpriteRendererComponent* m_SpriteRenderer = nullptr;

    Engine::AnimatorComponent* m_Animator = nullptr;

    float m_MovementSpeed = 250.0f;

    float m_SprintMultiplier = 2.0f;
};