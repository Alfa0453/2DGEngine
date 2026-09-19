#pragma once

#include "CollisionLayer2D.h"
#include "../Scene/Component.h"
#include "PhysicsMaterial2D.h"
#include "../Math/Vector2.h"

namespace Engine
{
    class Bounds2D;

    enum class ColliderShape2D
    {
        Box,
        Circle,
        Capsule,
        Polygon
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

        // ---------------------------------------------------------------
        // ONE-WAY (PASS-THROUGH) PLATFORMS
        // ---------------------------------------------------------------
        //
        // A one-way collider only produces a solid contact when the other
        // body is on the "solid" side defined by GetOneWayAxis(); from every
        // other direction the contact is discarded and the body passes
        // through. This is the classic jump-through platform.
        //
        // The axis is the outward world-space surface normal of the solid
        // face. In this engine gravity is +Y (downward), so a floor you land
        // on from above uses the default axis (0, -1) — "up". A contact is
        // kept only when the collision normal pushing the other body away
        // from this platform aligns with the axis by more than
        // GetOneWayThreshold() (a dot-product cosine).

        void SetOneWay(bool oneWay);

        bool IsOneWay() const;

        void SetOneWayAxis(const Vector2& worldAxis);

        const Vector2& GetOneWayAxis() const;

        // Minimum cosine between the contact normal and the one-way axis for a
        // contact to count as solid. Default 0.5 (~60 degrees), which keeps a
        // body resting on top solid while letting it pass through the sides
        // and underside. Clamped to [-1, 1].
        void SetOneWayThreshold(float cosineThreshold);

        float GetOneWayThreshold() const;

        // ---------------------------------------------------------------
        // SURFACE VELOCITY (CONVEYORS)
        // ---------------------------------------------------------------
        //
        // A non-zero surface velocity makes the contact behave like a moving
        // belt: friction drives touching bodies toward this world-space
        // velocity instead of toward rest. Only the component along the
        // contact tangent has an effect. Leave at zero for ordinary surfaces.

        void SetSurfaceVelocity(const Vector2& worldVelocity);

        const Vector2& GetSurfaceVelocity() const;

        std::uint64_t GetBoundsRevision() const;

    protected:

        void MarkBoundsDirty();

    private:
        
        ColliderShape2D m_Shape;

        CollisionLayerMask2D m_Layer = CollisionLayer2D::Default;

        CollisionLayerMask2D m_Mask = CollisionLayer2D::All;

        bool m_IsTrigger = false;

        bool m_IsEnabled = true;

        bool m_OneWay = false;

        // Outward normal of the solid face for one-way platforms. Default
        // points "up" (-Y) because gravity is +Y in this engine.
        Vector2 m_OneWayAxis{0.0f, -1.0f};

        float m_OneWayThreshold = 0.5f;

        Vector2 m_SurfaceVelocity{0.0f, 0.0f};

        PhysicsMaterial2D m_PhysicsMaterial;

        std::uint64_t m_BoundsRevision = 1;
    };
}