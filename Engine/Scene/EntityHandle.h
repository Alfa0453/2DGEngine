#pragma once

#include "EntityID.h"

namespace Engine
{
    class Entity;
    class Scene;

    class EntityHandle
    {
    public:
        EntityHandle() = default;

        EntityHandle(Scene* scene, EntityID id, EntityGeneration generation);

        Entity* Get() const;

        bool IsValid() const;

        EntityID GetID() const;

        EntityGeneration GetGeneration() const;

        Scene* GetScene() const;

        void Reset();

        explicit operator bool() const;

        Entity* operator->() const;

        bool operator==(const EntityHandle& other) const;

        bool operator!=(const EntityHandle& other) const;

    private:
        Scene* m_Scene = nullptr;

        EntityID m_ID = InvalidEntityID;

        EntityGeneration m_Generation = InvalidEntityGeneration;
    };
}