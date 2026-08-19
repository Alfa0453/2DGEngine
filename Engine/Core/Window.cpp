#include "Window.h"

#define WIN32_LEAN_AND_MEAN
#define __STDC_WANT_LIB_EXT1__ 0
#include <iostream>
#include <SDL3/SDL.h>

namespace Engine
{
    Window::~Window()
    {
        Shutdown();
    }

    bool Window::Initialize(const std::string& title, int width, int height)
    {
        m_Title = title;
        m_Width = width;
        m_Height = height;

        m_Window = SDL_CreateWindow(m_Title.c_str(), m_Width, m_Height, 0);

        if (!m_Window)
        {
            std::cerr << "Failed to create SDL window: " << SDL_GetError() << '\n';

            return false;
        }

        std::cout
            << "Window created successfully: "
            << m_Title
            << " ("
            << m_Width
            << "x"
            << m_Height
            << ")\n";

        return true;
    }
    
    void Window::Shutdown()
    {
        if (m_Window)
        {
            SDL_DestroyWindow(m_Window);

            m_Window = nullptr;

            std::cout << "Window destroyed.\n";
        }
    }

    SDL_Window* Window::GetNativeWindow() const
    {
        return m_Window;
    }

    int Window::GetWidth() const
    {
        return m_Width;
    }

    int Window::GetHeight() const
    {
        return m_Height;
    }
}