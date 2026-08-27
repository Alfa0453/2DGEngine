#pragma once

#include "CollisionLayer2D.h"
#include "../Scene/Component.h"
#include "PhysicsMaterial2D.h"

namespace Engine
{
    class Bounds2D;

    enum class ColliderShape2D
    {
        Box,
        Circle
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

        void SetPhysicsMaterial(const PhysicsMaterial2D& material);

        const PhysicsMaterial2D& GetPhysicsMaterial() const;

        PhysicsMaterial2D& GetPhysicsMaterial();

        std::uint64_t GetBoundsRevision() const;

    protected:

        void MarkBoundsDirty();

    private:
        
        ColliderShape2D m_Shape;

        CollisionLayerMask2D m_Layer = CollisionLayer2D::Default;

        CollisionLayerMask2D m_Mask = CollisionLayer2D::All;

        bool m_IsTrigger = false;

        bool m_IsEnabled = true;

        PhysicsMaterial2D m_PhysicsMaterial;

        std::uint64_t m_BoundsRevision = 1;
    };
}