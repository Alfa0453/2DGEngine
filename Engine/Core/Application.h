#pragma once

#include <memory>

#include "Window.h"
#include "../Graphics/Renderer2D.h"
#include "../Graphics/Texture2D.h"
#include "../Graphics/Camera2D.h"
#include "../Core/EngineTime.h"
#include "../Input/Input.h"
#include "../Scene/Entity.h"
#include "../Scene/EntityHandle.h"
#include "../Scene/SpriteRendererComponent.h"
#include "../Scene/TransformComponent.h"
#include "../Scene/Scene.h"

namespace Engine
{
    class Application
    {
    public:
        Application() = default;

        ~Application();

        bool Initialize();

        void Run();

        void Shutdown();

    private:
        void ProcessEvents();

    private:
        bool m_IsRunning = false;

        Window m_Window;

        Renderer2D m_Renderer;

        Camera2D m_Camera;

        Time m_Time;

        Input m_Input;

        Texture2D m_TestTexture;
        Texture2D m_CoinTexture;

        Scene m_Scene;

        EntityID m_PlayerID = InvalidEntityID;

        EntityHandle m_PlayerHandle;
    };
}