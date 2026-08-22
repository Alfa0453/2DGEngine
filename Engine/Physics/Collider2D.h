#pragma once

#include "CollisionLayer2D.h"
#include "../Scene/Component.h"

namespace Engine
{
    class Bounds2D;

    enum class ColliderShape2D
    {
        Box
    };

    class Collider2D : public Component
    {
    public:
        
        explicit Collider2D(ColliderShape2D shape);

        virtual ~Collider2D() = default;

        ColliderShape2D GetShape() const;

        void SetLayer(CollisionLayerMask2D layer);

        CollisionLayerMask2D GetLayer() const;

        void SetMask(CollisionLayerMask2D mask);

        CollisionLayerMask2D GetMask() const;

        void SetTrigger(bool trigger);

        bool IsTrigger() const;

        void SetEnabled(bool enabled);

        bool IsEnabled() const;

        bool CanInteractWith(const Collider2D& other) const;

        virtual Bounds2D GetWorldBounds() const = 0;

    private:
        
        ColliderShape2D m_Shape;

        CollisionLayerMask2D m_Layer = CollisionLayer2D::Default;

        CollisionLayerMask2D m_Mask = CollisionLayer2D::All;

        bool m_IsTrigger = false;

        bool m_IsEnabled = true;
    };
}