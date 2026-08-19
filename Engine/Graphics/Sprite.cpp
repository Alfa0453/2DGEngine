#include "Sprite.h"

#include "Texture2D.h"

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
        if (!m_Texture)
        {
            return { 0.0f, 0.0f };
        }

        return m_Texture->GetSize();
    }

    Vector2 Sprite::GetRenderedSize() const
    {
        const Vector2 baseSize = GetBaseSize();

        return 
        {
            baseSize.X * Transform.Scale.X,
            baseSize.Y * Transform.Scale.Y
        };
    }
}