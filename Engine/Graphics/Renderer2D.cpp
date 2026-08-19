#include "Renderer2D.h"
#include "Texture2D.h"

#include <iostream>
#include <SDL3/SDL.h>

namespace Engine
{
    Renderer2D::~Renderer2D()
    {
        Shutdown();
    }

    bool Renderer2D::Initialize(SDL_Window* window)
    {
        if (!window)
        {
            std::cerr 
                << "Renderer2D initialization failed: "
                << "Window is null.\n";

            return false;
        }

        m_Renderer = SDL_CreateRenderer(window, nullptr);

        if (!m_Renderer)
        {
            std::cerr
                << "Failed to create SDL renderer: " << SDL_GetError() << '\n';

            return false;
        }

        std::cout << "Renderer2D initialized successfully.\n";

        return true;
    }

    void Renderer2D::Shutdown()
    {
        if (m_Renderer)
        {
            SDL_DestroyRenderer(m_Renderer);

            m_Renderer = nullptr;

            std::cout << "Renderer2D shutdown successfully.\n";
        }
    }

    void Renderer2D::BeginFrame(const Color& clearColor)
    {
        if (!m_Renderer)
        {
            return;
        }

        SDL_SetRenderDrawColor(
            m_Renderer,
            clearColor.R,
            clearColor.G,
            clearColor.B,
            clearColor.A
        );

        SDL_RenderClear(m_Renderer);
    }

    void Renderer2D::EndFrame()
    {
        if (!m_Renderer)
        {
            return;
        }

        SDL_RenderPresent(m_Renderer);
    }

    void Renderer2D::DrawRect(const Vector2& position, const Vector2& size, const Color& color)
    {
        if (!m_Renderer)
        {
            return;
        }

        Vector2 screenPosition = position;

        Vector2 screenSize = size;

        if (m_Camera)
        {
            screenPosition = m_Camera->WorldToScreen(position);

            const float zoom = m_Camera->GetZoom();

            screenSize *= zoom;
        }

        SDL_FRect rectangle
        {
            screenPosition.X,
            screenPosition.Y,
            screenSize.X,
            screenSize.Y
        };

        SDL_SetRenderDrawColor(
            m_Renderer,
            color.R,
            color.G,
            color.B,
            color.A
        );

        SDL_RenderFillRect(m_Renderer, &rectangle);
    }

    void Renderer2D::DrawRect(const Rect& rect, const Color& color)
    {
        DrawRect(rect.Position, rect.Size, color);
    }

    void Renderer2D::DrawTexture(const Texture2D& texture, const Rect& destination)
    {
        if (!m_Renderer)
        {
            return;
        }

        if (!texture.IsValid())
        {
            return;
        }

        SDL_FRect destinationRect
        {
            destination.Position.X,
            destination.Position.Y,
            destination.Size.X,
            destination.Size.Y
        };

        if (!SDL_RenderTexture(m_Renderer, texture.GetNativeTexture(), nullptr, &destinationRect))
        {
            std::cerr << "Failed to render texture: " << SDL_GetError() << '\n';
        }
    }

    void Renderer2D::DrawSprite(const Sprite& sprite)
    {
        if (!m_Renderer)
        {
            return;
        }

        if (!sprite.IsValid())
        {
            return;
        }

        Texture2D* texture = sprite.GetTexture();

        if (!texture)
        {
            return;
        }

        const Vector2 baseSize = sprite.GetBaseSize();

        const Vector2 renderedSize = sprite.GetRenderedSize();

        Vector2 screenPosition = sprite.Transform.Position;

        float zoom = 1.0f;

        if (m_Camera)
        {
            screenPosition = m_Camera->WorldToScreen(sprite.Transform.Position);

            zoom = m_Camera->GetZoom();
        }

        SDL_FRect destination
        {
            screenPosition.X,
            screenPosition.Y,

            renderedSize.X * zoom,
            renderedSize.Y * zoom
        };

        SDL_SetTextureColorMod(
            texture->GetNativeTexture(),
            sprite.Tint.R,
            sprite.Tint.G,
            sprite.Tint.B
        );

        SDL_SetTextureAlphaMod(
            texture->GetNativeTexture(),
            sprite.Tint.A
        );

        SDL_FlipMode flipMode = SDL_FLIP_NONE;

        if (sprite.FlipX)
        {
            flipMode = static_cast<SDL_FlipMode>(flipMode | SDL_FLIP_HORIZONTAL);
        }

        if (sprite.FlipY)
        {
            flipMode = static_cast<SDL_FlipMode>(flipMode | SDL_FLIP_VERTICAL);
        }

        SDL_RenderTextureRotated(
            m_Renderer,
            texture->GetNativeTexture(),
            nullptr,
            &destination,
            static_cast<double>(sprite.Transform.Rotation),
            nullptr,
            flipMode
        );
    }

    void Renderer2D::SetCamera(Camera2D* camera)
    {
        m_Camera = camera;
    }

    Camera2D * Renderer2D::GetCamera() const
    {
        return m_Camera;
    }

    SDL_Renderer* Renderer2D::GetNativeRenderer() const
    {
        return m_Renderer;
    }
}