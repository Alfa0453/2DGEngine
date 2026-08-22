#pragma once

#include "Component.h"
#include "../Math/Transform2D.h"

#include <cstdint>

namespace Engine
{
    class TransformComponent : public Component
    {
    public:
        TransformComponent() = default;

        explicit TransformComponent(const Transform2D& transform);

        Transform2D& GetLocalTransform();

        const Transform2D& GetLocalTransform() const;

        const Transform2D& GetWorldTransform() const;

        void SetLocalPosition(const Vector2& position);

        const Vector2& GetLocalPosition() const;

        Vector2 GetWorldPosition() const;

        void SetLocalRotation(float rotation);

        float GetLocalRotation() const;

        float GetWorldRotation() const;

        void SetLocalScale(const Vector2& scale);

        void SetWorldPosition(const Vector2& position);

        void SetWorldRotation(float rotation);

        void SetWorldScale(const Vector2& scale);

        void SetWorldTransform(const Transform2D& transform);

        const Vector2& GetLocalScale() const;

        Vector2 GetWorldScale() const;

        void Translate(const Vector2& offset);

        void RotateBy(float degrees);

        void ScaleBy(const Vector2& multiplier);

        void MarkDirty();

        bool IsWorldTransformDirty() const;

        std::uint64_t GetWorldVersion() const;

        std::uint64_t GetRecalculationCount() const;


        // Compatibility wrappers
        void SetPosition(const Vector2& position);

        const Vector2& GetPosition() const;

        void SetRotation(float rotation);

        float GetRotation() const;

        void SetScale(const Vector2& scale);

        const Vector2& GetScale() const;

        

    private:
        Transform2D m_LocalTransform;

        mutable Transform2D m_CachedWorldTransform;

        mutable bool m_WorldTransformDirty = true;

        mutable std::uint64_t m_WorldVersion = 0;

        mutable std::uint64_t m_RecalculationCount = 0;
    };
}