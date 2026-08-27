#include "Application.h"
#include "../../Runtime/Game/Components/PlayerControllerComponent.h"
#include "../Scene/AnimatorComponent.h"
#include "../Physics/BoxCollider2D.h"
#include "../Physics/CircleCollider2D.h"
#include "../Math/Bounds2D.h"
#include "../Scene/Primitive2DFactory.h"
#include "../Graphics/PrimitiveTextureFactory2D.h"
#include "../Physics/PhysicsMaterial2D.h"

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

        m_Renderer.SetCamera(&m_Camera);

        m_Camera.SetPosition( {0.0f, 0.0f} );

        m_Camera.SetZoom(1.0f);

        m_Scene.SetName("TestScene");

        // Stone material
        PhysicsMaterial2D stone;

        stone.Restitution = 0.0f;

        stone.StaticFriction = 0.7f;

        stone.DynamicFriction = 0.5f;

        // Ice material
        PhysicsMaterial2D ice;

        ice.Restitution = 0.0f;

        ice.StaticFriction = 0.05f;

        ice.DynamicFriction = 0.02f;

        // Rubber
        PhysicsMaterial2D rubber;

        rubber.Restitution = 0.8f;

        rubber.StaticFriction = 0.9f;

        rubber.DynamicFriction = 0.7f;

        Entity* wall = Primitive2DFactory::Create(m_Scene, PrimitiveShape2D::Rectangle, {800.0f, 300.0f}, {10.0f, 300.0f});

        wall->AddComponent<BoxCollider2D>(Vector2{10.0f, 300.0f});

        auto* wallBody = wall->AddComponent<Rigidbody2D>();

        wallBody->SetBodyType(BodyType2D::Static);

        Entity* wall2 = Primitive2DFactory::Create(m_Scene, PrimitiveShape2D::Rectangle, {200.0f, 300.0f}, {10.0f, 300.0f});

        wall2->AddComponent<BoxCollider2D>(Vector2{10.0f, 300.0f});

        auto* wall2Body = wall2->AddComponent<Rigidbody2D>();

        wall2Body->SetBodyType(BodyType2D::Static);


        Entity* projectile = Primitive2DFactory::Create(m_Scene, PrimitiveShape2D::Circle, {400.0f, 300.0f}, {20.0f, 20.0f});

        auto* projectileCollider = projectile->AddComponent<CircleCollider2D>(20.0f);

        projectileCollider->SetPhysicsMaterial(rubber);

        auto* body = projectile->AddComponent<Rigidbody2D>();

        body->SetBodyType(BodyType2D::Dynamic);

        body->SetGravityScale(0.0f);

        body->SetCollisionDetectionMode(CollisionDetectionMode2D::Continuous);

        body->SetVelocity({2500.0f, 0.0f});

        m_PhysicsDebugRenderer.SetDrawColliders(true);

        m_PhysicsDebugRenderer.SetDrawAABBs(true);

        m_PhysicsDebugRenderer.SetDrawSpatialGrid(true);

        m_PhysicsDebugRenderer.SetDrawContacts(true);

        m_PhysicsDebugRenderer.SetDrawSleepingState(true);
        
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

            m_PhysicsDebugRenderer.Draw(m_Scene, m_Renderer);

            Bounds2D cameraBounds = m_Camera.GetWorldBounds();

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

        PrimitiveTextureFactory2D::Shutdown();

        m_PlayerIdleTexture.Unload();

        m_PlayerRunTexture.Unload();

        m_PlayerAttackTexture.Unload();

        m_Renderer.Shutdown();

        m_Window.Shutdown();

        SDL_Quit();

        std::cout << "2DGEngine shutdown successfully.\n";
    }
}