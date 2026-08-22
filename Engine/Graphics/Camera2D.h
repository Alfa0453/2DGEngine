#pragma once

#include "../Math/Bounds2D.h"
#include "../Math/Vector2.h"

namespace Engine
{
    class Camera2D
    {
    public:
        Camera2D() = default;

        void SetPosition(const Vector2& position);

        const Vector2& GetPosition() const;

        void Move(const Vector2& offset);

        void SetZoom(float zoom);

        float GetZoom() const;

        void SetViewportSize(const Vector2& size);

        const Vector2& GetViewportSize() const;

        Vector2 WorldToScreen(const Vector2& worldPosition) const;

        Vector2 ScreenToWorld(const Vector2& screenPosition) const;

        Bounds2D GetWorldBounds() const;

        bool IsBoundsVisible(const Bounds2D& bounds) const;

    private:
        Vector2 m_Position{ 0.0f, 0.0f };

        float m_Zoom = 1.0f;

        Vector2 m_ViewportSize{ 1280.0f, 720.0f };
    };
}