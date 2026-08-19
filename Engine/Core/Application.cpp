#include "Application.h"
#include "../../Runtime/Game/Components/PlayerControllerComponent.h"

#include <iostream>
#include <SDL3/SDL.h>
#include <memory>

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

        m_Renderer.SetCamera(&m_Camera);

        m_Camera.SetPosition( {0.0f, 0.0f} );

        m_Camera.SetZoom(1.0f);

        if (!m_TestTexture.Load(m_Renderer.GetNativeRenderer(), "Content/Sprites/Player.png"))
        {
            m_Renderer.Shutdown();

            m_Window.Shutdown();

            return false;
        }

        if (!m_CoinTexture.Load(m_Renderer.GetNativeRenderer(), "Content/Sprites/Coin.png"))
        {
            m_Renderer.Shutdown();

            m_Window.Shutdown();

            return false;
        }

        m_Scene.SetName("TestScene");

        Entity* player = m_Scene.CreateEntity("Player");

        const EntityID playerID = player->GetID();

        Entity* found = m_Scene.FindEntityByID(playerID);

        if (found)
        {
            std::cout << "Found: " << found->GetName() << '\n';
        }

        Entity* enemy = m_Scene.CreateEntity("Enemy");

        const EntityID enemyID = enemy->GetID();

        enemy->Destroy();

        Entity* result = m_Scene.FindEntityByID(enemyID);

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

        m_TestTexture.Unload();

        m_CoinTexture.Unload();

        m_Renderer.Shutdown();

        m_Window.Shutdown();

        SDL_Quit();

        std::cout << "2DGEngine shutdown successfully.\n";
    }
}