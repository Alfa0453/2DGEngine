#include "Texture2D.h"

#include <iostream>
#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>

namespace Engine
{
    Texture2D::~Texture2D()
    {
        Unload();
    }

    bool Texture2D::Load(SDL_Renderer* renderer, const std::string& filePath)
    {
        if (!renderer)
        {
            std::cerr << "Texture2D::Load failed: renderer is null.\n";

            return false;
        }

        // Remove an existing texture first.
        Unload();

        m_Texture = IMG_LoadTexture(renderer, filePath.c_str());

        if (!m_Texture)
        {
            std::cerr << "Failed to load texture: "
                      << filePath << '\n'
                      << "SDL Error: "
                      << SDL_GetError() << '\n';

            return false;
        }

        if (!SDL_GetTextureSize(m_Texture, &m_Width, &m_Height))
        {
            std::cerr << "Failed to query texture size: "
                      << SDL_GetError() << '\n';

            SDL_DestroyTexture(m_Texture);

            m_Texture = nullptr;

            return false;
        }

        m_FilePath = filePath;

        std::cout << "Texture loaded successfully: "
                  << m_FilePath << " (" << m_Width
                  << "X" << m_Height << ")\n";

        return true;
    }

    void Texture2D::Unload()
    {
        if (m_Texture)
        {
            SDL_DestroyTexture(m_Texture);

            m_Texture = nullptr;
        }

        m_Width = 0.0f;
        m_Height = 0.0f;

        m_FilePath.clear();
    }

    SDL_Texture* Texture2D::GetNativeTexture() const
    {
        return m_Texture;
    }

    float Texture2D::GetWidth() const
    {
        return m_Width;
    }

    float Texture2D::GetHeight() const
    {
        return m_Height;
    }

    Vector2 Texture2D::GetSize() const
    {
        return { m_Width, m_Height };
    }

    bool Texture2D::IsValid() const
    {
        return m_Texture != nullptr;
    }
}