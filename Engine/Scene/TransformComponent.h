#pragma once

#include "Component.h"
#include "../Math/Transform2D.h"

namespace Engine
{
    class TransformComponent : public Component
    {
    public:
        TransformComponent() = default;

        explicit TransformComponent(const Transform2D& transform);

        Transform2D& GetTransform();

        const Transform2D& GetTransform() const;

        void SetPosition(const Vector2& position);

        const Vector2& GetPosition() const;

        void SetRotation(float rotation);

        float GetRotation() const;

        void SetScale(const Vector2& scale);

        const Vector2& GetScale() const;

    private:
        Transform2D m_Transform;
    };
}