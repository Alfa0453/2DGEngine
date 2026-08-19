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
        if (zoom <= 0.0f)
        {
            zoom = 0.01f;
        }

        m_Zoom = zoom;
    }

    float Camera2D::GetZoom() const
    {
        return m_Zoom;
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
}