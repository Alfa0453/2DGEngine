#pragma once

#include "Collider2D.h"
#include "OrientedBox2D.h"

#include "../Math/Vector2.h"

namespace Engine
{
    class BoxCollider2D : public Collider2D
    {
    public:
        
        BoxCollider2D();

        explicit BoxCollider2D(const Vector2& size);

        void SetSize(const Vector2& size);

        const Vector2& GetSize() const;

        void SetOffset(const Vector2& offset);
        
        const Vector2& GetOffset() const;

        Bounds2D GetWorldBounds() const override;

        OrientedBox2D GetWorldOrientedBox() const;

        Vector2 GetWorldCenter() const;

        Vector2 GetWorldHalfExtents() const;

    private:

        Vector2 m_Size{1.0f, 1.0f};

        Vector2 m_Offset{0.0f, 0.0f};
    };
}