#pragma once

#include "../Math/Color.h"
#include "Component.h"

namespace Engine
{
    class Renderer2D;
    class Texture2D;

    class SpriteRendererComponent : public Component
    {
    public:
        SpriteRendererComponent() = default;

        explicit SpriteRendererComponent(Texture2D* texture);

        void SetTexture(Texture2D* texture);

        Texture2D* GetTexture() const;

        void SetTint(const Color& tint);

        const Color& GetTint() const;

        void SetFlipX(bool flip);

        bool GetFlipX() const;

        void SetFlipY(bool flip);

        bool GetFlipY() const;

        void Render(Renderer2D& renderer) const;

    private:
        Texture2D* m_Texture = nullptr;

        Color m_Tint = Color::White();

        bool m_FlipX = false;

        bool m_FlipY = false;
    };
}