#pragma once

#include "../Math/Color.h"
#include "../Math/Rect.h"
#include "../Math/Transform2D.h"
#include "Texture2D.h"

namespace Engine
{
    class Texture2D;

    class Sprite
    {
    public:
        Sprite() = default;

        explicit Sprite(Texture2D* texture);

        void SetTexture(Texture2D* texture);

        Texture2D* GetTexture() const;

        bool IsValid() const;

        Vector2 GetBaseSize() const;

        Vector2 GetRenderedSize() const;

        void SetSourceRect(const Rect& source);

        void ClearSourceRect();

        bool HasSourceRect() const;

        const Rect& GetSourceRect() const;

    public:
        Transform2D Transform;

        Vector2 Size{0.0f, 0.0f};

        Color Tint = Color::White();

        bool FlipX = false;
        bool FlipY = false;

    private:
        Texture2D* m_Texture = nullptr;

        Rect m_SourceRect;

        bool m_HasSourceRect = false;
    };
}