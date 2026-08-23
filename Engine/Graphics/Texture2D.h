#pragma once

#include <string>
#include <cstdint>

#include "../Math/Vector2.h"

struct SDL_Renderer;
struct SDL_Texture;

namespace Engine
{
    class Texture2D
    {
    public:
        Texture2D() = default;

        ~Texture2D();

        bool Load(SDL_Renderer* renderer, const std::string& filePath);

        void Unload();

        SDL_Texture* GetNativeTexture() const;

        float GetWidth() const;
        float GetHeight() const;

        Vector2 GetSize() const;

        bool IsValid() const;

        bool CreateFromRGBA(SDL_Renderer* renderer, int width, int height, const std::uint8_t* pixels);

    private:
        SDL_Texture* m_Texture = nullptr;

        float m_Width = 0.0f;
        float m_Height = 0.0f;

        std::string m_FilePath;
    };
}