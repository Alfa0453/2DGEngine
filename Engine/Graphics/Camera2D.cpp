#include "Camera2D.h"

namespace Engine
{
    void Camera2D::SetPosition(const Vector2& position)
    {
        m_Position = position;
    }

    const Vector2& Camera2D::GetPosition() const
    {
        return m_Position;
    }

    void Camera2D::Move(const Vector2& offset)
    {
        m_Position += offset;
    }

    void Camera2D::SetZoom(float zoom)
    {
        constexpr float MinZoom = 0.01f;

        m_Zoom = zoom > MinZoom ? zoom : MinZoom;
    }

    float Camera2D::GetZoom() const
    {
        return m_Zoom;
    }

    void Camera2D::SetViewportSize(const Vector2& size)
    {
        m_ViewportSize.X = size.X > 0.0f ? size.X : 1.0f;

        m_ViewportSize.Y = size.Y > 0.0f ? size.Y : 1.0f;
    }

    const Vector2& Camera2D::GetViewportSize() const
    {
        return m_ViewportSize;
    }

    Vector2 Camera2D::WorldToScreen(const Vector2& worldPosition) const
    {
        return 
        {
            (worldPosition.X - m_Position.X) * m_Zoom,
            (worldPosition.Y - m_Position.Y) * m_Zoom
        };
    }

    Vector2 Camera2D::ScreenToWorld(const Vector2& screenPosition) const
    {
        return 
        {
            screenPosition.X / m_Zoom + m_Position.X,
            screenPosition.Y / m_Zoom + m_Position.Y
        };
    }

    Bounds2D Camera2D::GetWorldBounds() const
    {
        const float safeZoom = m_Zoom > 0.0f ? m_Zoom : 1.0f;

        const Vector2 visibleWorldSize
        {
            m_ViewportSize.X / safeZoom,

            m_ViewportSize.Y / safeZoom
        };

        return Bounds2D::FromPositionSize(m_Position, visibleWorldSize);
    }

    bool Camera2D::IsBoundsVisible(const Bounds2D& bounds) const
    {
        return GetWorldBounds().Intersects(bounds);
    }
}