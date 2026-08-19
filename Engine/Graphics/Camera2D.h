#pragma once

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

        Vector2 WorldToScreen(const Vector2& worldPosition) const;

        Vector2 ScreenToWorld(const Vector2& screenPosition) const;

    private:
        Vector2 m_Position{ 0.0f, 0.0f };

        float m_Zoom = 1.0f;
    };
}