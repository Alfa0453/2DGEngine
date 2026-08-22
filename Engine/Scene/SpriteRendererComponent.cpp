#include "SpriteRendererComponent.h"

#include "Entity.h"
#include "../Graphics/Renderer2D.h"
#include "../Graphics/Sprite.h"
#include "../Graphics/Texture2D.h"
#include "TransformComponent.h"

#include <cmath>
#include <cstdlib>

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

        sprite.Transform = transform->GetWorldTransform();

        sprite.Tint = m_Tint;

        sprite.FlipX = m_FlipX;

        sprite.FlipY = m_FlipY;

        if (m_UseSpriteRegion)
        {
            sprite.SetSourceRect(m_SpriteRegion.SourceRect);
        }
        else 
        {
            sprite.ClearSourceRect();
        }

        renderer.DrawSprite(sprite);
    }

    Bounds2D SpriteRendererComponent::GetWorldBounds() const
    {
        if (!m_Texture)
        {
            return {};
        }

        Entity* owner = GetOwner();

        if (!owner)
        {
            return {};
        }

        TransformComponent* transform = owner->GetComponent<TransformComponent>();

        if (!transform)
        {
            return {};
        }

        const Transform2D& world = transform->GetWorldTransform();

        const Vector2 baseSize = GetBaseSpriteSize();

        const Vector2 renderedSize
        {
            baseSize.X * std::abs(world.Scale.X),

            baseSize.Y * std::abs(world.Scale.Y)
        };

        const Vector2 center = world.Position + renderedSize * 0.5f;

        Vector2 corners[4]
        {
            world.Position,

            {
                world.Position.X + renderedSize.X,

                world.Position.Y
            },

            world.Position + renderedSize,

            {
                world.Position.X,

                world.Position.Y + renderedSize.Y
            }
        };

        for (Vector2& corner : corners)
        {
            const Vector2 offset = corner - center;

            corner = center + Vector2::Rotate(offset, world.Rotation);
        }

        Bounds2D bounds(corners[0], corners[0]);

        for (int i = 1; i < 4; ++i)
        {
            bounds.Encapsulate(corners[i]);
        }

        return bounds;
    }

    void SpriteRendererComponent::SetSortingLayer(SortingLayer layer)
    {
        m_SortingLayer = layer;
    }

    SortingLayer SpriteRendererComponent::GetSortingLayer() const
    {
        return m_SortingLayer;
    }

    void SpriteRendererComponent::SetOrderInLayer(std::int32_t order)
    {
        m_OrderInLayer = order;
    }

    std::int32_t SpriteRendererComponent::GetOrderInLayer() const
    {
        return m_OrderInLayer;
    }

    void SpriteRendererComponent::SetSpriteRegion(const SpriteRegion& region)
    {
        m_SpriteRegion = region;

        m_UseSpriteRegion = true;
    }

    void SpriteRendererComponent::SetSpriteFrame(const SpriteFrame2D& frame)
    {
        if (!frame.IsValid())
        {
            return;
        }

        SetTexture(frame.Texture);

        SetSpriteRegion(frame.Region);
    }

    void SpriteRendererComponent::ClearSpriteRegion()
    {
        m_UseSpriteRegion = false;
    }

    bool SpriteRendererComponent::HasSpriteRegion() const
    {
        return m_UseSpriteRegion;
    }

    const SpriteRegion& SpriteRendererComponent::GetSpriteRegion() const
    {
        return m_SpriteRegion;
    }

    Vector2 SpriteRendererComponent::GetBaseSpriteSize() const
    {
        if (m_UseSpriteRegion)
        {
            return m_SpriteRegion.GetSize();
        }

        if (m_Texture)
        {
            return m_Texture->GetSize();
        }

        return{ 0.0f, 0.0f };
    }
}