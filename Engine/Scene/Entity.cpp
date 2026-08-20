#include "Entity.h"
#include "Component.h"
#include "ComponentTypeID.h"
#include "Scene.h"

#include <cstddef>
#include <memory>
#include <algorithm>

namespace Engine
{
    Entity::Entity(Scene* scene, EntityID id, EntityGeneration generation, const std::string& name)
        : m_ID(id), m_Scene(scene), m_Generation(generation), m_Name(name)
    {
    }

    Entity::~Entity()
    {
        DetachFromHierarchy();

        m_ComponentRegistry.clear();

        for (const auto& component : m_PendingComponents)
        {
            if (component)
            {
                component->DestroyInternal();
            }
        }

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

    Scene* Entity::GetScene() const
    {
        return m_Scene;
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
        for (auto iterator = m_Components.begin(); iterator != m_Components.end();)
        {
            Component* component = iterator->get();

            if (component && component->IsPendingDestroy())
            {
                UnregisterComponent(component->GetTypeID(), component);

                component->DestroyInternal();

                iterator = m_Components.erase(iterator);
            }
            else 
            {
                ++iterator;
            }
        }
    }

    void Entity::FlushPendingComponents()
    {
        for (auto& component : m_PendingComponents)
        {
            if (component->IsPendingDestroy())
            {
                UnregisterComponent(component->GetTypeID(), component.get());

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

        auto iterator = m_ComponentRegistry.find(typeID);

        if (iterator == m_ComponentRegistry.end())
        {
            m_ComponentRegistry.emplace(typeID, component);
        }
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

        if (iterator->second != component)
        {
            return;
        }

        m_ComponentRegistry.erase(iterator);

        for (const auto& current : m_Components)
        {
            Component* candidate = current.get();

            if (!candidate || candidate == component || candidate->IsPendingDestroy())
            {
                continue;
            }

            if (candidate->GetTypeID() == typeID)
            {
                m_ComponentRegistry.emplace(typeID, candidate);

                return;
            }
        }

        for (const auto& current : m_PendingComponents)
        {
            Component* candidate = current.get();

            if (!candidate || candidate == component || candidate->IsPendingDestroy())
            {
                continue;
            }

            if (candidate->GetTypeID() == typeID)
            {
                m_ComponentRegistry.emplace(typeID, candidate);

                return;
            }
        }
    }

    bool Entity::ValidateComponentRegistry() const
    {
        for (const auto& entry : m_ComponentRegistry)
        {
            const ComponentTypeID typeID = entry.first;

            Component* component = entry.second;

            if (!component)
            {
                return false;
            }

            if (component->IsPendingDestroy())
            {
                return false;
            }

            if (component->GetTypeID() != typeID)
            {
                return false;
            }

            bool found = false;

            for (const auto& ownedComponent : m_Components)
            {
                if (ownedComponent.get() == component)
                {
                    found = true;

                    break;
                }
            }

            if (!found)
            {
                for (const auto& pendingComponent : m_PendingComponents)
                {
                    if (pendingComponent.get() == component)
                    {
                        found = true;

                        break;
                    }
                }
            }

            if (!found)
            {
                return false;
            }
        }

        return true;
    }

    std::size_t Entity::GetRegisteredComponentTypeCount() const
    {
        return m_ComponentRegistry.size();
    }

    EntityHandle Entity::GetParent() const
    {
        return m_Parent;
    }

    bool Entity::HasParent() const
    {
        return m_Parent.IsValid();
    }

    const std::vector<EntityHandle>& Entity::GetChildren() const
    {
        return m_Children;
    }

    std::size_t Entity::GetChildCount() const
    {
        std::size_t count = 0;

        for (const EntityHandle& child : m_Children)
        {
            if (child.IsValid())
            {
                ++count;
            }
        }

        return count;
    }

    void Entity::AddChildInternal(const EntityHandle& child)
    {
        if (!child)
        {
            return;
        }

        for (const EntityHandle& existing : m_Children)
        {
            if (existing == child)
            {
                return;
            }
        }

        m_Children.push_back(child);
    }

    void Entity::RemoveChildInternal(EntityID childID)
    {
        auto iterator = std::remove_if(m_Children.begin(), m_Children.end(),
                            [childID](const EntityHandle& handle)
                            {
                                return !handle.IsValid() || handle->GetID() == childID;
                            }
                        );

        m_Children.erase(iterator, m_Children.end());
    }

    bool Entity::IsChildOf(const Entity* entity) const
    {
        if (!entity)
        {
            return false;
        }

        for (const EntityHandle& child : entity->m_Children)
        {
            Entity* childEntity = child.Get();

            if (childEntity == this)
            {
                return true;
            }
        }

        return false;
    }

    bool Entity::IsDescendantOf(const Entity* entity) const
    {
        if (!entity)
        {
            return false;
        }

        EntityHandle current = m_Parent;

        while (current)
        {
            Entity* currentEntity = current.Get();

            if (!currentEntity)
            {
                return false;
            }

            if (currentEntity == entity)
            {
                return true;
            }

            current = currentEntity->GetParent();
        }

        return false;
    }

    bool Entity::SetParent(Entity* parent)
    {
        if (parent == this)
        {
            return false;
        }

        if (parent && parent->GetScene() != m_Scene)
        {
            return false;
        }

        if (parent && parent->IsDescendantOf(this))
        {
            return false;
        }

        if (m_Parent)
        {
            Entity* oldParent = m_Parent.Get();

            if (oldParent)
            {
                oldParent->RemoveChildInternal(m_ID);
            }
        }

        m_Parent.Reset();

        if (!parent)
        {
            return true;
        }

        m_Parent = m_Scene->CreateHandle(parent);

        parent->AddChildInternal(m_Scene->CreateHandle(this));

        return true;
    }

    void Entity::ClearParent()
    {
        SetParent(nullptr);
    }

    void Entity::DetachFromHierarchy()
    {
        if (m_Parent)
        {
            Entity* parent = m_Parent.Get();

            if (parent)
            {
                parent->RemoveChildInternal(m_ID);
            }

            m_Parent.Reset();
        }

        for (EntityHandle& childHandle: m_Children)
        {
            Entity* child = childHandle.Get();

            if (child)
            {
                child->m_Parent.Reset();
            }
        }

        m_Children.clear();
    }
}