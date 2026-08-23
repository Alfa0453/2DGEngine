#include "PrimitiveTextureFactory2D.h"
#include "Texture2D.h"

#include <vector>
#include <cstdint>
#include <iostream>

namespace
{
    Engine::Texture2D g_WhiteTexture;

    Engine::Texture2D g_CircleTexture;

    Engine::Texture2D g_TriangleTexture;

    bool g_Initialized = false;
}

namespace Engine
{
    bool PrimitiveTextureFactory2D::Initialize(SDL_Renderer *renderer)
    {
        if (g_Initialized)
        {
            return true;
        }

        if (!renderer)
        {
            return false;
        }

        if (!CreateWhiteTexture(renderer))
        {
            Shutdown();

            return false;
        }

        if (!CreateCircleTexture(renderer))
        {
            Shutdown();

            return false;
        }

        if (!CreateTriangleTexture(renderer))
        {
            Shutdown();

            return false;
        }

        g_Initialized = true;

        return true;
    }

    Texture2D* PrimitiveTextureFactory2D::GetTexture(PrimitiveShape2D shape)
    {
        if (!g_Initialized)
        {
            return nullptr;
        }

        switch (shape)
        {
            case PrimitiveShape2D::Square:

            case PrimitiveShape2D::Rectangle:
                return &g_WhiteTexture;

            case PrimitiveShape2D::Circle:
                return &g_CircleTexture;

            case PrimitiveShape2D::Triangle:
                return &g_TriangleTexture;
        }

        return nullptr;
    }

    bool PrimitiveTextureFactory2D::CreateWhiteTexture(SDL_Renderer* renderer)
    {
        const std::uint8_t pixel[4]
        {
            255,
            255,
            255,
            255
        };

        return g_WhiteTexture.CreateFromRGBA(renderer, 1, 1, pixel);
    }

    bool PrimitiveTextureFactory2D::CreateCircleTexture(SDL_Renderer* renderer)
    {
        constexpr int size = 64;

        constexpr float center = size * 0.5f;

        constexpr float radius = size * 0.5f - 1.0f;

        std::vector<std::uint8_t> pixels(size * size * 4, 0);

        for (int y = 0; y < size; ++y)
        {
            for (int x = 0; x < size; ++x)
            {
                const float dx = (static_cast<float>(x) + 0.5f) - center;
                
                const float dy = (static_cast<float>(y) + 0.5f) - center;

                const float distanceSquared = dx * dx + dy * dy;

                const bool inside = distanceSquared <= radius * radius;

                const std::size_t index = static_cast<std::size_t>((y * size + x) * 4);

                pixels[index + 0] = 255;

                pixels[index + 1] = 255;

                pixels[index + 2] = 255;

                pixels[index + 3] = inside ? 255 : 0;
            }
        }

        return g_CircleTexture.CreateFromRGBA(renderer, size, size, pixels.data());
    }

    bool PrimitiveTextureFactory2D::CreateTriangleTexture(SDL_Renderer* renderer)
    {
        constexpr int size = 64;

        const Vector2 a{32.0f, 2.0f};

        const Vector2 b{2.0f, 62.0f};

        const Vector2 c{62.0f, 62.0f};

        std::vector<std::uint8_t> pixels(size * size * 4, 0);

        for (int y = 0; y < size; ++y)
        {
            for (int x = 0; x < size; ++x)
            {
                const Vector2 point
                {
                    static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f
                };

                const bool inside = PointInTriangle(point, a, b, c);

                const std::size_t index = static_cast<std::size_t>((y * size + x) * 4);

                pixels[index + 0] = 255;

                pixels[index + 1] = 255;

                pixels[index + 2] = 255;

                pixels[index + 3] = inside ? 255 : 0;
            }
        }

        return g_TriangleTexture.CreateFromRGBA(renderer, size, size, pixels.data());
    }

    float PrimitiveTextureFactory2D::Sign(const Vector2& p1, const Vector2& p2, const Vector2& p3)
    {
        return 
            (p1.X - p3.X) * (p2.Y - p3.Y) - (p2.X - p3.X) * (p1.Y - p3.Y);
    }

    bool PrimitiveTextureFactory2D::PointInTriangle(const Vector2& point, const Vector2& a, const Vector2& b, const Vector2& c)
    {
        const float d1 = Sign(point, a, b);

        const float d2 = Sign(point, b, c);

        const float d3 = Sign(point, c, a);

        const bool hasNegative = d1 < 0.0f || d2 < 0.0f || d3 < 0.0f;

        const bool hasPositive = d1 > 0.0f || d2 > 0.0f || d3 > 0.0f;

        return !(hasNegative && hasPositive);
    }

    bool PrimitiveTextureFactory2D::IsInitialized()
    {
        return g_Initialized;
    }

    void PrimitiveTextureFactory2D::Shutdown()
    {
        g_WhiteTexture.Unload();

        g_CircleTexture.Unload();

        g_TriangleTexture.Unload();

        g_Initialized = false;
    }
}