#include "PolygonCollider2D.h"

#include "../Scene/Entity.h"
#include "../Scene/TransformComponent.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace Engine
{
    PolygonCollider2D::PolygonCollider2D()
        : Collider2D(ColliderShape2D::Polygon)
    {
    }

    PolygonCollider2D::PolygonCollider2D(const std::vector<Vector2>& vertices)
        : Collider2D(ColliderShape2D::Polygon)
    {
        SetVertices(vertices);
    }

    void PolygonCollider2D::SetVertices(const std::vector<Vector2>& vertices)
    {
        m_Vertices = vertices;

        RemoveDuplicateAdjancentVertices();

        EnsureConsistentWinding();

        MarkBoundsDirty();
    }

    const std::vector<Vector2>& PolygonCollider2D::GetVertices() const
    {
        return m_Vertices;
    }

    void PolygonCollider2D::ClearVertices()
    {
        if (m_Vertices.empty())
        {
            return;
        }

        m_Vertices.clear();

        MarkBoundsDirty();
    }

    std::size_t PolygonCollider2D::GetVertexCount() const
    {
        return m_Vertices.size();
    }

    void PolygonCollider2D::SetOffset(const Vector2& offset)
    {
        if (m_Offset.X == offset.X && m_Offset.Y == offset.Y)
        {
            return;
        }

        m_Offset = offset;

        MarkBoundsDirty();
    }

    const Vector2& PolygonCollider2D::GetOffset() const
    {
        return m_Offset;
    }

    void PolygonCollider2D::RemoveDuplicateAdjancentVertices()
    {
        if (m_Vertices.size() < 2)
        {
            return;
        }

        constexpr float epsilonSquared = 0.000001f;

        std::vector<Vector2> cleaned;

        cleaned.reserve(m_Vertices.size());

        for (const Vector2& vertex : m_Vertices)
        {
            if (cleaned.empty())
            {
                cleaned.push_back(vertex);

                continue;
            }

            const Vector2 delta = vertex - cleaned.back();

            if (delta.LengthSquared() > epsilonSquared)
            {
                cleaned.push_back(vertex);
            }
        }

        // Polygon closure is implicit.
        //
        // If the final point repeats this first point, remove the duplicate.

        if (cleaned.size() >= 2)
        {
            const Vector2 closingDelta = cleaned.front() - cleaned.back();

            if (closingDelta.LengthSquared() <= epsilonSquared)
            {
                cleaned.pop_back();
            }
        }

        m_Vertices = std::move(cleaned);
    }

    float PolygonCollider2D::CalculateSignedArea() const
    {
        if (m_Vertices.size() < 3)
        {
            return 0.0f;
        }

        float area = 0.0f;

        for (std::size_t i = 0; i < m_Vertices.size(); ++i)
        {
            const Vector2& current = m_Vertices[i];

            const Vector2& next = m_Vertices[(i + 1) % m_Vertices.size()];

            area += current.X * next.Y - next.X * current.Y;
        }

        return area * 0.5f;
    }

    void PolygonCollider2D::EnsureConsistentWinding()
    {
        if (m_Vertices.size() < 3)
        {
            return;
        }

        const float signedArea = CalculateSignedArea();

        if (signedArea < 0.0f)
        {
            std::reverse(m_Vertices.begin(), m_Vertices.end());
        }
    }

    bool PolygonCollider2D::IsConvex() const
    {
        if (m_Vertices.size() < 3)
        {
            return false;
        }

        constexpr float epsilon = 0.000001f;

        float previousCross = 0.0f;

        for (std::size_t i = 0; i < m_Vertices.size(); ++i)
        {
            const Vector2& a = m_Vertices[i];

            const Vector2& b = m_Vertices[(i + 1) % m_Vertices.size()];

            const Vector2& c = m_Vertices[(i + 2) % m_Vertices.size()];
            
            const Vector2 ab = b - a;

            const Vector2 bc = c - b;

            const float cross = ab.X * bc.Y - ab.Y * bc.X;

            // Collinear points don't change convexity.

            if (std::abs(cross) <= epsilon)
            {
                continue;
            }

            if (std::abs(previousCross) <= epsilon)
            {
                previousCross = cross;

                continue;
            }

            if (cross * previousCross < 0.0f)
            {
                return false;
            }
        }

        return true;
    }

    bool PolygonCollider2D::IsValidPolygon() const
    {
        if (m_Vertices.size() < 3)
        {
            return false;
        }

        constexpr float areaEpsilon = 0.000001f;

        if (std::abs(CalculateSignedArea()) <= areaEpsilon)
        {
            return false;
        }

        return IsConvex();
    }

    Polygon2D PolygonCollider2D::GetWorldPolygon() const
    {
        Polygon2D polygon;

        if (!IsValidPolygon())
        {
            return polygon;
        }

        Entity* owner = GetOwner();

        if (!owner)
        {
            return polygon;
        }

        TransformComponent* transform = owner->GetComponent<TransformComponent>();

        if (!transform)
        {
            return polygon;
        }

        const Transform2D& world = transform->GetWorldTransform();

        // WORLD VERTICES

        polygon.Vertices.reserve(m_Vertices.size());

        for (const Vector2& localVertex : m_Vertices)
        {
            polygon.Vertices.push_back(world.TransformPoint(localVertex + m_Offset));
        }

        polygon.Normals.reserve(polygon.Vertices.size());

        constexpr float epsilon = 0.000001f;

        for (std::size_t i = 0; i < polygon.Vertices.size(); ++i)
        {
            const Vector2& current = polygon.Vertices[i];

            const Vector2& next = polygon.Vertices[(i + 1) % polygon.Vertices.size()];

            const Vector2 edge = next - current;

            Vector2 normal{-edge.Y, edge.X};

            const float lengthSquared = normal.LengthSquared();

            if (lengthSquared <= epsilon)
            {
                polygon.Normals.push_back({0.0f, 0.0f});

                continue;
            }

            normal *= 1.0f / std::sqrt(lengthSquared);

            polygon.Normals.push_back(normal);
        }

        return polygon;
    }

    Bounds2D PolygonCollider2D::GetWorldBounds() const
    {
        const Polygon2D polygon = GetWorldPolygon();

        Bounds2D bounds;

        if (polygon.Vertices.empty())
        {
            return bounds;
        }

        bounds.Min = polygon.Vertices[0];

        bounds.Max = polygon.Vertices[0];

        for (std::size_t i = 1; i < polygon.Vertices.size(); ++i)
        {
            const Vector2& vertex = polygon.Vertices[i];

            bounds.Min.X = std::min(bounds.Min.X, vertex.X);

            bounds.Min.Y = std::min(bounds.Min.Y, vertex.Y);

            bounds.Max.X = std::max(bounds.Max.X, vertex.X);

            bounds.Max.Y = std::max(bounds.Max.Y, vertex.Y);
        }

        return bounds;
    }
}