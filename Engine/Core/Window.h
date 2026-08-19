#pragma once

#include <string>

struct SDL_Window;

namespace Engine
{
    class Window
    {
    public:
        Window() = default;

        ~Window();

        bool Initialize(const std::string& title, int width, int height);

        void Shutdown();

        SDL_Window* GetNativeWindow() const;

        int GetWidth() const;
        int GetHeight() const;

    private:
        SDL_Window* m_Window = nullptr;

        int m_Width = 0;
        int m_Height = 0;

        std::string m_Title;
    };
}