#include "PlayerControllerComponent.h"

#include "../../../Engine/Scene/Entity.h"
#include "../../../Engine/Input/Input.h"
#include "../../../Engine/Input/KeyCode.h"
#include "../../../Engine/Scene/SpriteRendererComponent.h"
#include "../../../Engine/Scene/TransformComponent.h"
#include "../../../Engine/Scene/AnimatorComponent.h"
#include "../../../Engine/Math/Vector2.h"

PlayerControllerComponent::PlayerControllerComponent(Engine::Input* input)
    : m_Input(input)
{
}

PlayerControllerComponent::PlayerControllerComponent(Engine::Input* input, float movementSpeed)
    : m_Input(input),
      m_MovementSpeed(movementSpeed)
{
}

void PlayerControllerComponent::Start()
{
    Engine::Entity* owner = GetOwner();

    if (!owner)
    {
        return;
    }

    m_Transform = owner->GetComponent<Engine::TransformComponent>();

    m_SpriteRenderer = owner->GetComponent<Engine::SpriteRendererComponent>();

    m_Animator = owner->GetComponent<Engine::AnimatorComponent>();
}

void PlayerControllerComponent::Update(float deltaTime)
{
    if (!m_Input || !m_Transform || !m_Animator)
    {
        return;
    }

    // Attack

    const bool attackPressed = m_Input->WasKeyPressed(Engine::KeyCode::Space);

    const bool attackActive = m_Animator->GetCurrentStateName() == "Attack" && m_Animator->IsPlaying();

    if (attackPressed)
    {
        m_Animator->SetTrigger("Attack");
    }

    // Movement input
    Engine::Vector2 movementDirection{0.0f, 0.0f};

    if (m_Input->IsKeyDown(Engine::KeyCode::W))
    {
        movementDirection.Y -= 1.0f;
    }

    if (m_Input->IsKeyDown(Engine::KeyCode::S))
    {
        movementDirection.Y += 1.0f;
    }

    if (m_Input->IsKeyDown(Engine::KeyCode::A))
    {
        movementDirection.X -= 1.0f;
    }

    if (m_Input->IsKeyDown(Engine::KeyCode::D))
    {
        movementDirection.X += 1.0f;
    }

    // movment state
    const bool isMoving = movementDirection.X != 0.0f || movementDirection.Y != 0.0f;

    const bool isRunning = isMoving && m_Input->IsKeyDown(Engine::KeyCode::LeftShift);

    float currentSpeed = m_MovementSpeed;

    if (isRunning)
    {
        currentSpeed *= m_SprintMultiplier;
    }

    m_Animator->SetFloat("Speed", isMoving ? currentSpeed : 0.0f);

    m_Animator->SetBool("Running", isRunning);

    if (movementDirection.LengthSqured() > 0.0f)
    {
        movementDirection = movementDirection.Normalized();
    }

    if (!attackActive && isMoving)
    {
        m_Transform->Translate(movementDirection * currentSpeed * deltaTime);
    }

    
    /*
    if (isMoving)
    {
        movementDirection = movementDirection.Normalized();

        float speed = m_MovementSpeed;

        if (isRunning)
        {
            speed *= m_SprintMultiplier;
        }

        m_Animator->SetFloat("Speed", isMoving ? speed : 0.0f);

        m_Transform->Translate(movementDirection * speed * deltaTime);
    }*/

    // facing
    if (m_SpriteRenderer)
    {
        if (movementDirection.X < 0.0f)
        {
            m_SpriteRenderer->SetFlipX(true);
        }
        else if (movementDirection.X > 0.0f)
        {
            m_SpriteRenderer->SetFlipX(false);
        }
    }


    /*
    if (movementDirection.LengthSqured() > 0.0f)
    {
        movementDirection = movementDirection.Normalized();
    }

    float currentSpeed = m_MovementSpeed;

    if (m_Input->IsKeyDown(Engine::KeyCode::LeftShift))
    {
        currentSpeed *= m_SprintMultiplier;
    }

    m_Transform->Translate(movementDirection * currentSpeed * deltaTime);

    if (m_SpriteRenderer)
    {
        if (movementDirection.X < 0.0f)
        {
            m_SpriteRenderer->SetFlipX(true);
        }
        else if (movementDirection.X > 0.0f)
        {
            m_SpriteRenderer->SetFlipX(false);
        }
    }*/
}

void PlayerControllerComponent::SetMovementSpeed(float speed)
{
    if (speed < 0.0f)
    {
        speed = 0.0f;
    }

    m_MovementSpeed = speed;
}

float PlayerControllerComponent::GetMovementSpeed() const
{
    return m_MovementSpeed;
}

void PlayerControllerComponent::SetSprintMultiplier(float multiplier)
{
    if (multiplier < 1.0f)
    {
        multiplier = 1.0f;
    }

    m_SprintMultiplier = multiplier;
}

float PlayerControllerComponent::GetSprintMultiplier() const
{
    return m_SprintMultiplier;
}