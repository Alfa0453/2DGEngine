#include "Application.h"
#include "../../Runtime/Game/Components/PlayerControllerComponent.h"
#include "../Scene/AnimatorComponent.h"
#include "../Physics/BoxCollider2D.h"
#include "../Math/Bounds2D.h"
#include "../Scene/Primitive2DFactory.h"
#include "../Graphics/PrimitiveTextureFactory2D.h"

#include <iostream>
#include <SDL3/SDL.h>
#include <memory>
#include <cstdint>

namespace Engine
{
    Application::~Application()
    {
        Shutdown();
    }

    bool Application::Initialize()
    {
        std::cout << "Starting 2DGEngine...\n";

        if (!SDL_Init(SDL_INIT_VIDEO))
        {
            std::cerr << "Failed to initialize SDL: " << SDL_GetError() << '\n';

            return false;
        }

        std::cout << "SDL initialized successfully.\n";

        if (!m_Window.Initialize("2DGEngine", 1280, 720))
        {
            SDL_Quit();

            return false;
        }

        if (!m_Renderer.Initialize(m_Window.GetNativeWindow()))
        {
            m_Window.Shutdown();

            SDL_Quit();

            return false;
        }

        if (!PrimitiveTextureFactory2D::Initialize(m_Renderer.GetNativeRenderer()))
        {
            m_Window.Shutdown();

            SDL_Quit();

            return false;
        }

        m_PlayerIdleTexture.Load(m_Renderer.GetNativeRenderer(), "Content/Sprites/Right - Idle.png");

        m_PlayerWalkTexture.Load(m_Renderer.GetNativeRenderer(), "Content/Sprites/Right - Walking.png");

        m_PlayerRunTexture.Load(m_Renderer.GetNativeRenderer(), "Content/Sprites/Right - Running.png");

        m_PlayerAttackTexture.Load(m_Renderer.GetNativeRenderer(), "Content/Sprites/Right - Attacking.png");

        const Vector2 frameSize{480.0f, 480.0f};

        // Idle clips
        m_PlayerIdleClip.SetName("Idle");

        m_PlayerIdleClip.SetFramesPerSecond(12.0f);

        m_PlayerIdleClip.SetLooping(true);

        for (std::int32_t row = 0; row < 4; ++row)
        {
            for (std::int32_t column = 0; column < 4; ++column)
            {
                m_PlayerIdleClip.AddFrame(&m_PlayerIdleTexture, SpriteRegion::FromGrid(column, row, frameSize));
            }
        }

        // Walk
        m_PlayerWalkClip.SetName("Walk");

        m_PlayerWalkClip.SetFramesPerSecond(16.0f);

        m_PlayerWalkClip.SetLooping(true);

        for (std::int32_t row = 0; row < 5; ++row)
        {
            for (std::int32_t column = 0; column < 4; ++column)
            {
                m_PlayerWalkClip.AddFrame(&m_PlayerWalkTexture, SpriteRegion::FromGrid(column, row, frameSize));
            }
        }

        // Run clip
        m_PlayerRunClip.SetName("Run");

        m_PlayerRunClip.SetFramesPerSecond(24.0f);

        m_PlayerRunClip.SetLooping(true);

        for (std::int32_t row = 0; row < 3; ++row)
        {
            for (std::int32_t column = 0; column < 4; ++column)
            {
                m_PlayerRunClip.AddFrame(&m_PlayerRunTexture, SpriteRegion::FromGrid(column, row, frameSize));
            }
        }

        // Attack clip
        m_PlayerAttackClip.SetName("Attack");

        m_PlayerAttackClip.SetFramesPerSecond(24.0f);

        m_PlayerAttackClip.SetLooping(false);

        for (std::int32_t row = 0; row < 2; ++row)
        {
            for (std::int32_t column = 0; column < 5; ++column)
            {
                m_PlayerAttackClip.AddFrame(&m_PlayerAttackTexture, SpriteRegion::FromGrid(column, row, frameSize));
            }
        }

        m_Renderer.SetCamera(&m_Camera);

        m_Camera.SetPosition( {0.0f, 0.0f} );

        m_Camera.SetZoom(1.0f);

        m_Scene.SetName("TestScene");

        m_Player = m_Scene.CreateEntity("Player");

        m_PlayerID = m_Player->GetID();

        m_Player->AddTag("Player");

        m_PlayerHandle = m_Scene.CreateHandle(m_Player);

        TransformComponent* playerTransform = m_Player->AddComponent<TransformComponent>();

        playerTransform->SetLocalPosition({400.0f, 300.0f});

        playerTransform->SetLocalScale({1.0f, 1.0f});

        SpriteRendererComponent* playerSpriteRenderer = m_Player->AddComponent<SpriteRendererComponent>(&m_PlayerIdleTexture);

        playerSpriteRenderer->SetSortingLayer(SortingLayer::Characters);

        playerSpriteRenderer->SetOrderInLayer(0);

        BoxCollider2D* playerCollider = m_Player->AddComponent<BoxCollider2D>(Vector2{270.0f, 390.0f});

        playerCollider->SetOffset({0.0f, 6.0f});

        playerCollider->SetLayer(CollisionLayer2D::Player);

        playerCollider->SetMask(CollisionLayer2D::World | CollisionLayer2D::Enemy | CollisionLayer2D::Pickup);

        AnimatorComponent* playerAnimator = m_Player->AddComponent<AnimatorComponent>();

        playerAnimator->AddState("Idle", &m_PlayerIdleClip);

        playerAnimator->AddState("Walk", &m_PlayerWalkClip);

        playerAnimator->AddState("Run", &m_PlayerRunClip);

        playerAnimator->AddState("Attack", &m_PlayerAttackClip);

        // Parameters
        playerAnimator->AddFloatParameter("Speed", 0.0f);

        playerAnimator->AddBoolParameter("Running", false);

        playerAnimator->AddTriggerParameter("Attack");

        // Transitions
        AnimatorTransition2D idleToWalk;

        idleToWalk.FromState = "Idle";

        idleToWalk.ToState = "Walk";

        idleToWalk.Priority = 10;

        idleToWalk.Conditions.push_back(AnimatorCondition2D::FloatGreater("Speed", 0.01f));

        playerAnimator->AddTransition(idleToWalk);

        AnimatorTransition2D walkToIdle;

        walkToIdle.FromState = "Walk";

        walkToIdle.ToState = "Idle";

        walkToIdle.Priority = 10;

        walkToIdle.Conditions.push_back(AnimatorCondition2D::FloatLessOrEqual("Speed", 0.01f));

        playerAnimator->AddTransition(walkToIdle);

        AnimatorTransition2D walkToRun;

        walkToRun.FromState = "Walk";

        walkToRun.ToState = "Run";

        walkToRun.Priority = 10;

        walkToRun.Conditions.push_back(AnimatorCondition2D::Bool("Running", true));

        playerAnimator->AddTransition(walkToRun);

        AnimatorTransition2D runToWalk;

        runToWalk.FromState = "Run";

        runToWalk.ToState = "Walk";

        runToWalk.Priority = 10;

        runToWalk.Conditions.push_back(AnimatorCondition2D::Bool("Running", false));

        runToWalk.Conditions.push_back(AnimatorCondition2D::FloatGreater("Speed", 0.01f));

        playerAnimator->AddTransition(runToWalk);

        AnimatorTransition2D runToIdle;

        runToIdle.FromState = "Run";

        runToIdle.ToState = "Idle";

        runToIdle.Priority = 10;

        runToIdle.Conditions.push_back(AnimatorCondition2D::FloatLessOrEqual("Speed", 0.01f));

        playerAnimator->AddTransition(runToIdle);

        AnimatorTransition2D anyToAttack;

        anyToAttack.FromState = AnimatorAnyState;

        anyToAttack.ToState = "Attack";

        anyToAttack.Priority = 50;

        anyToAttack.Conditions.push_back(AnimatorCondition2D::Trigger("Attack"));

        playerAnimator->AddTransition(anyToAttack);

        AnimatorTransition2D attackToRun;

        attackToRun.FromState = "Attack";

        attackToRun.ToState = "Run";

        attackToRun.HasExitTime = true;

        attackToRun.ExiTime = 0.90f;

        attackToRun.Priority = 20;

        attackToRun.Conditions.push_back(AnimatorCondition2D::Bool("Running", true));

        attackToRun.Conditions.push_back(AnimatorCondition2D::FloatGreater("Speed", 0.01f));

        playerAnimator->AddTransition(attackToRun);

        AnimatorTransition2D attackToWalk;

        attackToWalk.FromState = "Attack";

        attackToWalk.ToState = "Walk";

        attackToWalk.HasExitTime = true;

        attackToWalk.ExiTime = 0.85f;

        attackToWalk.Priority = 20;

        attackToWalk.Conditions.push_back(AnimatorCondition2D::Bool("Running", false));

        attackToWalk.Conditions.push_back(AnimatorCondition2D::FloatGreater("Speed", 0.01f));

        playerAnimator->AddTransition(attackToWalk);

        AnimatorTransition2D attackToIdle;

        attackToIdle.FromState = "Attack";

        attackToIdle.ToState = "Idle";

        attackToIdle.HasExitTime = true;

        attackToIdle.ExiTime = 0.80f;

        attackToIdle.Priority = 20;

        attackToIdle.Conditions.push_back(AnimatorCondition2D::FloatLessOrEqual("Speed", 0.01f));

        playerAnimator->AddTransition(attackToIdle);

        playerAnimator->Play("Idle");

        PlayerControllerComponent* controller = m_Player->AddComponent<PlayerControllerComponent>(&m_Input, 200.0f);

        Entity* floor = Primitive2DFactory::Create(m_Scene, PrimitiveShape2D::Rectangle, {400.0f, 500.0f}, {600.0f, 40.0});

        auto* floorCollider = floor->AddComponent<BoxCollider2D>(Vector2{600.0f, 40.0f});

        auto* floorBody = floor->AddComponent<Rigidbody2D>();

        floorBody->SetBodyType(BodyType2D::Static);
        
        m_Scene.Start();

        m_Time.Initialize();

        m_Input.Initialize();

        m_IsRunning = true;

        return true;
    }

    void Application::Run()
    {
        while (m_IsRunning)
        {
            ProcessEvents();

            m_Time.Update();

            m_Input.Update();

            const float deltaTime = m_Time.GetDeltaTime();

            if (m_Input.WasKeyPressed(KeyCode::Escape))
            {
                m_IsRunning = false;
            }

            m_Scene.Update(deltaTime);

            m_Renderer.BeginFrame();

            m_Scene.Render(m_Renderer);

            Bounds2D cameraBounds = m_Camera.GetWorldBounds();

            cameraBounds.Expand(64.0f);

            Rect cameraRect(cameraBounds.Min, cameraBounds.GetSize());

            m_Renderer.DrawRectOutline(cameraRect, Color::Green());

            m_Renderer.EndFrame();
        }
    }

    void Application::ProcessEvents()
    {
        SDL_Event event;

        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_EVENT_QUIT)
            {
                m_IsRunning = false;
            }
        }
    }

    void Application::Shutdown()
    {
        if (!m_IsRunning && !m_Window.GetNativeWindow() && !m_Renderer.GetNativeRenderer())
        {
            return;
        }

        m_IsRunning = false;

        m_Scene.Clear();

        m_PlayerIdleTexture.Unload();

        m_PlayerRunTexture.Unload();

        m_PlayerAttackTexture.Unload();

        m_Renderer.Shutdown();

        m_Window.Shutdown();

        SDL_Quit();

        std::cout << "2DGEngine shutdown successfully.\n";
    }
}