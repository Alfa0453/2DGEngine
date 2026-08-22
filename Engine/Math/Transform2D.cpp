#include "Transform2D.h"

namespace Engine
{
    Vector2 Transform2D::TransformPoint(const Vector2& point) const
    {
        Vector2 scaledPoint = point * Scale;

        Vector2 rotationPoint = Vector2::Rotate(scaledPoint, Rotation);

        return Position + rotationPoint;
    }

    Vector2 Transform2D::InverseTransformPoint(const Vector2& point) const
    {
        Vector2 relativePoint = point - Position;

        Vector2 unrotatedPoint = Vector2::InverseRotate(relativePoint, Rotation);

        return Vector2::SafeDivide(unrotatedPoint, Scale);
    }

    Transform2D Transform2D::Combine(const Transform2D &parent, const Transform2D &local)
    {
        Transform2D world;

        world.Position = parent.TransformPoint(local.Position);

        world.Rotation = parent.Rotation + local.Rotation;

        world.Scale = parent.Scale * local.Scale;

        return world;
    }

    Transform2D Transform2D::InverseCombine(const Transform2D &parent, const Transform2D &world)
    {
        Transform2D local;

        local.Position = parent.InverseTransformPoint(world.Position);

        local.Rotation = world.Rotation - parent.Rotation;

        local.Scale = Vector2::SafeDivide(world.Scale, parent.Scale);

        return local;
    }
}