#pragma once

#include "PrimitiveShape2D.h"
#include "../Math/Vector2.h"

struct SDL_Renderer;

namespace Engine
{
    class Texture2D;

    class PrimitiveTextureFactory2D
    {
    public:

        static bool Initialize(SDL_Renderer* renderer);

        static void Shutdown();

        static Texture2D* GetTexture(PrimitiveShape2D shape);

        bool IsInitialized();

    private:

        static bool CreateWhiteTexture(SDL_Renderer* renderer);

        static bool CreateCircleTexture(SDL_Renderer* renderer);

        static bool CreateTriangleTexture(SDL_Renderer* renderer);

        static float Sign(const Vector2& p1, const Vector2& p2, const Vector2& p3);

        static bool PointInTriangle(const Vector2& point, const Vector2& a, const Vector2& b, const Vector2& c);
    };
}