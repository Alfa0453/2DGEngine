#pragma once

namespace Engine
{
    class Scene;
    class Renderer2D;
    class PhysicsWorld2D;

    class Collider2D;
    class BoxCollider2D;
    class CircleCollider2D;

    class Color;

    class PhysicsDebugRenderer2D
    {
    public:
        void SetDrawColliders(bool enabled);

        bool GetDrawColliders() const;

        void SetDrawAABBs(bool enabled);

        bool GetDrawAABBs() const;

        void SetDrawSpatialGrid(bool enabled);

        bool GetDrawSpatialGrid() const;

        void SetDrawContacts(bool enabled);

        bool GetDrawContacts() const;

        void SetDrawSleepingState(bool enabled);

        bool GetDrawSleepingState() const;

        void Draw(const Scene& scene, Renderer2D& renderer) const;

    private:
        void DrawCollider(const Collider2D& collider, Renderer2D& rendere) const;

        void DrawBoxCollider(const BoxCollider2D& box, Renderer2D& renderer, const Color& color) const;

        void DrawCircleCollider(const CircleCollider2D& circle, Renderer2D& renderer, const Color& color) const;

        void DrawAABB(const Collider2D& collider, Renderer2D& renderer) const;

        void DrawOBB(const BoxCollider2D& collider, Renderer2D& renderer) const;

        void DrawSpatialGrid(const PhysicsWorld2D& world, Renderer2D& renderer) const;

        void DrawContacts(const PhysicsWorld2D& world, Renderer2D& renderer) const;

    private:
        bool m_DrawColliders = true;

        bool m_DrawAABBs = false;

        bool m_DrawSpatialGrid = false;

        bool m_DrawContacts = true;

        bool m_DrawSleepingState = true;
    };
}