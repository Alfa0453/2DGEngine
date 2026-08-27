#pragma once

#include "Collider2D.h"

#include "../Math/Vector2.h"
#include "../Math/Bounds2D.h"

#include <cstdint>

namespace Engine
{
    class CircleCollider2D final : public Collider2D
    {
    public:
        CircleCollider2D();

        explicit CircleCollider2D(float radius);

        void SetRadius(float radius);

        float GetRadius() const;

        void SetOffset(const Vector2& offset);

        const Vector2& GetOffset() const;

        Vector2 GetWorldCenter() const;

        float GetWorldRadius() const;

        Bounds2D GetWorldBounds() const override;

    private:
        float m_Radius = 0.5f;

        Vector2 m_Offset{0.0f, 0.0f};
    };
}