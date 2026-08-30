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

        Entity* ground = Primitive2DFactory::Create(m_Scene, PrimitiveShape2D::Rectangle, {640.0f, 710.0f}, {1100.0f, 50.0f});

        ground->AddComponent<BoxCollider2D>(Vector2{1000.0f, 50.0f});

        auto* groundBody = ground->AddComponent<Rigidbody2D>();

        groundBody->SetBodyType(BodyType2D::Static);

        Entity* wallL = Primitive2DFactory::Create(m_Scene, PrimitiveShape2D::Rectangle, {100.0f, 600.0f}, {500.0f, 50.0f});

        auto* wallLTransform = wallL->GetComponent<TransformComponent>();

        wallLTransform->SetWorldRotation(90.0f);

        wallL->AddComponent<BoxCollider2D>(Vector2{500.0f, 50.0f});

        auto* wallLBody = wallL->AddComponent<Rigidbody2D>();

        wallLBody->SetBodyType(BodyType2D::Static);

        Entity* boxA = Primitive2DFactory::Create(m_Scene, PrimitiveShape2D::Square, {650.0f, 100.0f}, {80.0f, 80.0f});

        boxA->AddComponent<BoxCollider2D>(Vector2{80.0f, 80.0f});

        Rigidbody2D* bodyA = boxA->AddComponent<Rigidbody2D>();

        bodyA->SetBodyType(BodyType2D::Static);

        Entity* boxB = Primitive2DFactory::Create(m_Scene, PrimitiveShape2D::Square, {650.0f, 150.0f}, {80.0f, 80.0f});

        boxB->AddComponent<BoxCollider2D>(Vector2{80.0f, 80.0f});

        Rigidbody2D* bodyB = boxB->AddComponent<Rigidbody2D>();

        bodyB->SetBodyType(BodyType2D::Dynamic);

        // Distance joint test.

        //m_TestDistanceJoint = std::make_unique<DistanceJoint2D>(bodyA, bodyB, Vector2{0.0f, 0.0f}, Vector2{0.0f, 0.0f}, 250.0f);

        //m_Scene.GetPhysicsWorld().AddJoint(m_TestDistanceJoint.get());

        m_TestRevoluteJoint = std::make_unique<RevoluteJoint2D>(bodyA, bodyB, Vector2{0.0f, 0.0f}, Vector2{0.0f, -100.0f});

        m_Scene.GetPhysicsWorld().AddJoint(m_TestRevoluteJoint.get());

        
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

        if (m_TestDistanceJoint)
        {
            m_Scene.GetPhysicsWorld().RemoveJoint(m_TestDistanceJoint.get());

            m_TestDistanceJoint.reset();
        }

        if (m_TestRevoluteJoint)
        {
            m_Scene.GetPhysicsWorld().RemoveJoint(m_TestRevoluteJoint.get());

            m_TestRevoluteJoint.reset();
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