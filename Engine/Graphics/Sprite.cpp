#include "Sprite.h"

#include "Texture2D.h"

#include <cmath>

namespace Engine
{
    Sprite::Sprite(Texture2D* texture)
        : m_Texture(texture)
    {
    }

    void Sprite::SetTexture(Texture2D* texture)
    {
        m_Texture = texture;
    }

    Texture2D* Sprite::GetTexture() const
    {
        return m_Texture;
    }

    bool Sprite::IsValid() const
    {
        return m_Texture != nullptr && m_Texture->IsValid();
    }

    Vector2 Sprite::GetBaseSize() const
    {
        if (m_HasSourceRect)
        {
            return m_SourceRect.Size;
        }

        if (m_Texture)
        {
            return m_Texture->GetSize();
        }

        return{ 0.0f, 0.0f };
    }

    Vector2 Sprite::GetRenderedSize() const
    {
        const Vector2 baseSize = GetBaseSize();

        return 
        {
            baseSize.X * std::abs(Transform.Scale.X),
            baseSize.Y * std::abs(Transform.Scale.Y)
        };
    }

    void Sprite::SetSourceRect(const Rect& source)
    {
        m_SourceRect = source;

        m_HasSourceRect = true;
    }

    void Sprite::ClearSourceRect()
    {
        m_HasSourceRect = false;
    }

    bool Sprite::HasSourceRect() const
    {
        return m_HasSourceRect;
    }

    const Rect& Sprite::GetSourceRect() const
    {
        return m_SourceRect;
    }
}