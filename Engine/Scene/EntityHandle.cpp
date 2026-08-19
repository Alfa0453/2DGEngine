#include "EntityHandle.h"

#include "Entity.h"
#include "Scene.h"

namespace Engine
{
    EntityHandle::EntityHandle(Scene* scene, EntityID id, EntityGeneration generation)
        : m_Scene(scene), m_ID(id), m_Generation(generation)
    {
    }

    Entity* EntityHandle::Get() const
    {
        if (!m_Scene)
        {
            return nullptr;
        }

        if (m_ID == InvalidEntityID)
        {
            return nullptr;
        }

        if (m_Generation == InvalidEntityGeneration)
        {
            return nullptr;
        }

        Entity* entity = m_Scene->FindEntityByID(m_ID);

        if (!entity)
        {
            return nullptr;
        }

        if (entity->GetGeneration() != m_Generation)
        {
            return nullptr;
        }

        return entity;
    }

    bool EntityHandle::IsValid() const
    {
        return Get() != nullptr;
    }

    EntityID EntityHandle::GetID() const
    {
        return m_ID;
    }

    EntityGeneration EntityHandle::GetGeneration() const
    {
        return m_Generation;
    }

    Scene* EntityHandle::GetScene() const
    {
        return m_Scene;
    }

    void EntityHandle::Reset()
    {
        m_Scene = nullptr;

        m_ID = InvalidEntityID;

        m_Generation = InvalidEntityGeneration;
    }

    EntityHandle::operator bool() const
    {
        return IsValid();
    }

    Entity* EntityHandle::operator->() const
    {
        return Get();
    }

    bool EntityHandle::operator==(const EntityHandle& other) const
    {
        return 
            m_Scene == other.m_Scene
            && m_ID == other.m_ID
            && m_Generation == other.m_Generation;
    }

    bool EntityHandle::operator!=(const EntityHandle& other) const
    {
        return !(*this == other);
    }
}