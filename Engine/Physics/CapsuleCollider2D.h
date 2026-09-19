#pragma once

#include "Collider2D.h"

#include "../Math/Vector2.h"
#include "../Math/Bounds2D.h"

namespace Engine
{
    struct Capsule2D
    {
        Vector2 PointA{0.0f, 0.0f};

        Vector2 PointB{0.0f, 0.0f};

        float Radius = 0.0f;
    };

    class CapsuleCollider2D final : public Collider2D
    {
    public:
        
        CapsuleCollider2D();

        CapsuleCollider2D(float radius, float halfHeight);

        void SetRadius(float radius);

        float GetRadius() const;

        void SetHalfHeight(float halfHeight);

        float GetHalfHeight() const;

        void SetOffset(const Vector2& offset);

        const Vector2& GetOffset() const;

        Capsule2D GetWorldCapsule() const;

        Bounds2D GetWorldBounds() const override;

    private:

        float m_Radius = 25.0f;

        float m_HalfHeight = 50.0f;

        Vector2 m_Offset{0.0f, 0.0f};
    };
}
