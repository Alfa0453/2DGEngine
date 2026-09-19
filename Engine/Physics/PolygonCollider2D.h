#pragma once

#include "Collider2D.h"
#include "Polygon2D.h"

#include "../Math/Vector2.h"
#include "../Math/Bounds2D.h"

#include <cstddef>
#include <vector>

namespace Engine
{
    class PolygonCollider2D final : public Collider2D
    {
    public:

        PolygonCollider2D();

        explicit PolygonCollider2D(const std::vector<Vector2>& vertices);

        void SetVertices(const std::vector<Vector2>& vertices);

        const std::vector<Vector2>& GetVertices() const;

        void ClearVertices();

        std::size_t GetVertexCount() const;

        void SetOffset(const Vector2& offset);

        const Vector2& GetOffset() const;

        bool IsValidPolygon() const;

        bool IsConvex() const;

        Polygon2D GetWorldPolygon() const;

        Bounds2D GetWorldBounds() const override;

    private:

        void RemoveDuplicateAdjancentVertices();

        void EnsureConsistentWinding();

        float CalculateSignedArea() const;

    private:

        std::vector<Vector2> m_Vertices;

        Vector2 m_Offset{0.0f, 0.0f};
    };
}