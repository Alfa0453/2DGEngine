#include "Component.h"

namespace Engine
{
    Entity* Component::GetOwner() const
    {
        return m_Owner;
    }

    ComponentTypeID Component::GetTypeID() const
    {
        return m_TypeID;
    }

    void Component::SetOwner(Entity* owner)
    {
        m_Owner = owner;
    }

    void Component::SetTypeID(ComponentTypeID typeID)
    {
        m_TypeID = typeID;
    }

    bool Component::HasStarted() const
    {
        return m_HasStarted;
    }

    void Component::StartInternal()
    {
        if (m_HasStarted || m_HasBeenDestroyed)
        {
            return;
        }

        m_HasStarted = true;

        Start();
    }

    void Component::DestroyInternal()
    {
        if (m_HasBeenDestroyed)
        {
            return;
        }

        m_HasBeenDestroyed = true;

        OnDestroy();
    }

    void Component::Destroy()
    {
        if (m_HasBeenDestroyed)
        {
            return;
        }

        m_PendingDestroy = true;
    }

    bool Component::IsPendingDestroy() const
    {
        return m_PendingDestroy;
    }
}