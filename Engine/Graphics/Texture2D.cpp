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

    bool Texture2D::CreateFromRGBA(SDL_Renderer* renderer, int width, int height, const std::uint8_t* pixels)
    {
        if (!renderer || !pixels || width <= 0 || height <= 0)
        {
            return false;
        }

        Unload();

        SDL_Surface* surface = SDL_CreateSurfaceFrom(width, height, SDL_PIXELFORMAT_RGBA32, const_cast<std::uint8_t*>(pixels), width * 4);

        if (!surface)
        {
            return false;
        }

        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);

        SDL_DestroySurface(surface);

        if (!texture)
        {
            return false;
        }

        m_Texture = texture;

        m_Width = width;

        m_Height = height;

        return true;
    }
}