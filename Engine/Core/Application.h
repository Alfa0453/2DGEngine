#pragma once

#include <memory>

#include "Window.h"
#include "../Graphics/Renderer2D.h"
#include "../Graphics/Texture2D.h"
#include "../Graphics/Camera2D.h"
#include "../Graphics/AnimationClip2D.h"
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

        Texture2D m_PlayerIdleTexture;
        Texture2D m_PlayerWalkTexture;
        Texture2D m_PlayerRunTexture;
        Texture2D m_PlayerAttackTexture;

        AnimationClip2D m_PlayerIdleClip;
        AnimationClip2D m_PlayerWalkClip;
        AnimationClip2D m_PlayerRunClip;
        AnimationClip2D m_PlayerAttackClip;

        Scene m_Scene;

        EntityID m_PlayerID = InvalidEntityID;

        EntityHandle m_PlayerHandle;

        Entity* m_Player = nullptr;
    };
}