#pragma once

#include "Component.h"

#include "../Math/Color.h"
#include "../Math/Bounds2D.h"
#include "../Graphics/SortingLayer.h"
#include "../Graphics/SpriteFrame2D.h"

#include <cstdint>

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

        Bounds2D GetWorldBounds() const;

        void SetSortingLayer(SortingLayer layer);

        SortingLayer GetSortingLayer() const;

        void SetOrderInLayer(std::int32_t order);

        std::int32_t GetOrderInLayer() const;

        void SetSpriteRegion(const SpriteRegion& region);

        void SetSpriteFrame(const SpriteFrame2D& frame);

        void ClearSpriteRegion();

        bool HasSpriteRegion() const;

        const SpriteRegion& GetSpriteRegion() const;

        Vector2 GetBaseSpriteSize() const;

    private:
        Texture2D* m_Texture = nullptr;

        Color m_Tint = Color::White();

        SortingLayer m_SortingLayer = SortingLayer::Environment;

        std::int32_t m_OrderInLayer = 0;

        bool m_FlipX = false;

        bool m_FlipY = false;

        SpriteRegion m_SpriteRegion;

        bool m_UseSpriteRegion = false;
    };
}