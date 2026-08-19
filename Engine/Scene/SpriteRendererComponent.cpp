#include "SpriteRendererComponent.h"

#include "Entity.h"
#include "../Graphics/Renderer2D.h"
#include "../Graphics/Sprite.h"
#include "../Graphics/Texture2D.h"
#include "TransformComponent.h"

namespace Engine
{
    SpriteRendererComponent::SpriteRendererComponent(Texture2D* texture)
        : m_Texture(texture)
    {
    }

    void SpriteRendererComponent::SetTexture(Texture2D* texture)
    {
        m_Texture = texture;
    }

    Texture2D* SpriteRendererComponent::GetTexture() const
    {
        return m_Texture;
    }

    void SpriteRendererComponent::SetTint(const Color& tint)
    {
        m_Tint = tint;
    }

    const Color& SpriteRendererComponent::GetTint() const
    {
        return m_Tint;
    }

    void SpriteRendererComponent::SetFlipX(bool flip)
    {
        m_FlipX = flip;
    }

    bool SpriteRendererComponent::GetFlipX() const
    {
        return m_FlipX;
    }

    void SpriteRendererComponent::SetFlipY(bool flip)
    {
        m_FlipY = flip;
    }

    bool SpriteRendererComponent::GetFlipY() const
    {
        return m_FlipY;
    }

    void SpriteRendererComponent::Render(Renderer2D& renderer) const
    {
        if (!m_Texture)
        {
            return;
        }

        Entity* owner = GetOwner();

        if (!owner)
        {
            return;

        }

        TransformComponent* transform = owner->GetComponent<TransformComponent>();

        if (!transform)
        {
            return;
        }

        Sprite sprite(m_Texture);

        sprite.Transform = transform->GetTransform();

        sprite.Tint = m_Tint;

        sprite.FlipX = m_FlipX;

        sprite.FlipY = m_FlipY;

        renderer.DrawSprite(sprite);
    }
}