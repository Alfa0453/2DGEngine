#pragma once

#include "../Math/Vector2.h"
#include "../Math/Color.h"
#include "../Math/Rect.h"
#include "../Graphics/Texture2D.h"
#include "../Graphics/Sprite.h"
#include "../Graphics/Camera2D.h"

struct SDL_Renderer;
struct SDL_Window;

namespace Engine 
{
    class Renderer2D
    {
    public:
        Renderer2D() = default;

        ~Renderer2D();

        bool Initialize(SDL_Window* window);

        void Shutdown();

        void BeginFrame(const Color& clearColor = Color(25, 25, 35, 255));

        void EndFrame();

        void DrawRect(const Vector2& position, const Vector2& size, const Color& color);

        void DrawRect(const Rect& rect, const Color& color);

        void DrawTexture(const Texture2D& texture, const Rect& destination);

        void DrawSprite(const Sprite& sprite);

        void SetCamera(Camera2D* camera);

        Camera2D* GetCamera() const;

        SDL_Renderer* GetNativeRenderer() const;

    private:
        SDL_Renderer* m_Renderer = nullptr;

        Camera2D* m_Camera = nullptr;
    };
}