#include "Entity.h"
#include "Component.h"
#include <memory>

namespace Engine
{
    Entity::Entity(EntityID id, EntityGeneration generation, const std::string& name)
        : m_ID(id), m_Generation(generation), m_Name(name)
    {
    }

    Entity::~Entity()
    {
        for (const auto& component : m_Components)
        {
            component->DestroyInternal();
        }
    }

    EntityID Entity::GetID() const
    {
        return m_ID;
    }

    EntityGeneration Entity::GetGeneration() const
    {
        return m_Generation;
    }

    void Entity::Start()
    {
        if (m_HasStarted)
        {
            return;
        }

        m_HasStarted = true;

        for (const auto& component : m_Components)
        {
            component->StartInternal();
        }
    }

    bool Entity::HasStarted() const
    {
        return m_HasStarted;
    }

    void Entity::Update(float deltaTime)
    {
        if (!m_Active || m_PendingDestroy)
        {
            return;
        }

        if (!m_HasStarted)
        {
            Start();
        }

        m_IsUpdating = true;

        for (const auto& component : m_Components)
        {
            if (component->HasStarted() && !component->IsPendingDestroy())
            {
                component->Update(deltaTime);
            }
        }

        m_IsUpdating = false;

        FlushDestroyedComponents();

        FlushPendingComponents();
    }

    const std::string& Entity::GetName() const
    {
        return m_Name;
    }

    void Entity::SetName(const std::string& name)
    {
        m_Name = name;
    }

    bool Entity::IsActive() const
    {
        return m_Active;
    }

    void Entity::SetActive(bool active)
    {
        m_Active = active;
    }

    void Entity::AddTag(const std::string& tag)
    {
        if (tag.empty())
        {
            return;
        }

        m_Tags.insert(tag);
    }

    void Entity::RemoveTag(const std::string& tag)
    {
        m_Tags.erase(tag);
    }

    bool Entity::HasTag(const std::string& tag) const
    {
        return m_Tags.find(tag) != m_Tags.end();
    }

    void Entity::ClearTags()
    {
        m_Tags.clear();
    }

    bool Entity::RemoveComponent(Component* component)
    {
        if (!component)
        {
            return false;
        }

        for (const auto& current : m_Components)
        {
            if (current.get() == component)
            {
                if (current->IsPendingDestroy())
                {
                    return false;
                }

                current->Destroy();

                return true;
            }
        }

        return false;
    }

    const std::unordered_set<std::string>& Entity::GetTags() const
    {
        return m_Tags;
    }

    std::size_t Entity::GetComponentCount() const
    {
        std::size_t count = 0;

        for (const auto& component : m_Components)
        {
            if (!component->IsPendingDestroy())
            {
                ++count;
            }
        }

        return count;
    }

    const std::vector<std::unique_ptr<Component>>& Entity::GetAllComponents() const
    {
        return m_Components;
    }

    void Entity::Destroy()
    {
        m_PendingDestroy = true;

        m_Active = false;
    }

    bool Entity::IsPendingDestroy() const
    {
        return m_PendingDestroy;
    }

    void Entity::FlushDestroyedComponents()
    {
        auto iterator = std::remove_if(m_Components.begin(), m_Components.end(),
                            [](const std::unique_ptr<Component>& component)
                            {
                                if (component->IsPendingDestroy())
                                {
                                    component->DestroyInternal();

                                    return true;
                                }

                                return false;
                            }
                        );

        m_Components.erase(iterator, m_Components.end());
    }

    void Entity::FlushPendingComponents()
    {
        for (auto& component : m_PendingComponents)
        {
            if (component->IsPendingDestroy())
            {
                component->DestroyInternal();

                continue;
            }
            
            Component* pointer = component.get();

            m_Components.push_back(std::move(component));

            if (m_HasStarted)
            {
                pointer->StartInternal();
            }
        }

        m_PendingComponents.clear();
    }

    void Entity::RegisterComponent(ComponentTypeID typeID, Component* component)
    {
        if (typeID == InvalidComponentTypeID || !component)
        {
            return;
        }

        m_ComponentRegistry[typeID] = component;
    }

    void Entity::UnregisterComponent(ComponentTypeID typeID, Component* component)
    {
        if (typeID == InvalidComponentTypeID || !component)
        {
            return;
        }

        auto iterator = m_ComponentRegistry.find(typeID);

        if (iterator == m_ComponentRegistry.end())
        {
            return;
        }

        if (iterator->second == component)
        {
            m_ComponentRegistry.erase(iterator);
        }
    }
}