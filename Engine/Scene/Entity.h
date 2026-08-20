#pragma once

#include "ComponentTypeID.h"
#include "EntityID.h"
#include "EntityHandle.h"
#include "Component.h"
#include "ComponentType.h"

#include <memory>
#include <string>
#include <algorithm>
#include <type_traits>
#include <utility>
#include <vector>
#include <unordered_set>
#include <unordered_map>

namespace Engine
{
    class Scene;

    class Entity
    {
    public:
        Entity(Scene* scene, EntityID id, EntityGeneration generation, const std::string& name);

        ~Entity();

        EntityID GetID() const;

        EntityGeneration GetGeneration() const;

        Scene* GetScene() const;

        void Start();

        void Update(float deltaTime);

        const std::string& GetName() const;

        void SetName(const std::string& name);

        bool IsActive() const;

        void SetActive(bool active);

        void AddTag(const std::string& tag);

        void RemoveTag(const std::string& tag);

        bool HasTag(const std::string& tag) const;

        void ClearTags();

        const std::unordered_set<std::string>& GetTags() const;

        bool HasStarted() const;

        bool RemoveComponent(Component* component);

        std::size_t GetComponentCount() const;

        const std::vector<std::unique_ptr<Component>>& GetAllComponents() const;

        void Destroy();

        bool IsPendingDestroy() const;

        bool ValidateComponentRegistry() const;

        std::size_t GetRegisteredComponentTypeCount() const;

        bool SetParent(Entity* parent);

        void ClearParent();

        EntityHandle GetParent() const;

        bool HasParent() const;

        const std::vector<EntityHandle>& GetChildren() const;

        std::size_t GetChildCount() const;

        bool IsChildOf(const Entity* entity) const;

        bool IsDescendantOf(const Entity* entity) const;

        template<typename T, typename... Args>
        T* AddComponent(Args&&... args)
        {
            static_assert(std::is_base_of_v<Component, T>, "T must inherit from Component.");

            auto component = std::make_unique<T>(std::forward<Args>(args)...);

            T* componentPointer = component.get();

            componentPointer->SetOwner(this);

            const ComponentTypeID typeID = GetComponentTypeID<T>();

            componentPointer->SetTypeID(typeID);

            RegisterComponent(typeID,  componentPointer);

            if (m_IsUpdating)
            {
                m_PendingComponents.push_back(std::move(component));
            }
            else
            {
                m_Components.push_back(std::move(component));

                if (m_HasStarted)
                {
                    componentPointer->StartInternal();
                }
            }

            return componentPointer;
        }

        template<typename T>
        bool RemoveComponent()
        {
            static_assert(std::is_base_of_v<Component, T>, "T must inherit from Component.");

            for (const auto& component : m_Components)
            {
                T* typedComponent = dynamic_cast<T*>(component.get());

                if (typedComponent && !typedComponent->IsPendingDestroy())
                {
                    typedComponent->Destroy();

                    return true;
                }
            }

            for (const auto& component : m_PendingComponents)
            {
                T* typedComponent = dynamic_cast<T*>(component.get());

                if (typedComponent && !typedComponent->IsPendingDestroy())
                {
                    typedComponent->Destroy();

                    return true;
                }
            }

            return false;
        }

        template<typename T>
        std::size_t RemoveAllComponents()
        {
            static_assert(std::is_base_of_v<Component, T>, "T must inherit from Component.");

            std::size_t markedCount = 0;

            for (const auto& component : m_Components)
            {
                T* typedComponent = dynamic_cast<T*>(component.get());

                if (typedComponent && !typedComponent->IsPendingDestroy())
                {
                    typedComponent->Destroy();

                    ++markedCount;
                }
            }

            return markedCount;
        }

        template<typename T>
        T* GetComponent() const
        {
            static_assert(std::is_base_of_v<Component, T>, "T must inherit from Component.");

            const ComponentTypeID typeID = GetComponentTypeID<T>();

            auto iterator = m_ComponentRegistry.find(typeID);

            if (iterator == m_ComponentRegistry.end())
            {
                return nullptr;
            }

            Component* component = iterator->second;

            if (!component || component->IsPendingDestroy())
            {
                return nullptr;
            }

            return static_cast<T*>(component);
        }

        template<typename T>
        std::vector<T*> GetComponents() const
        {
            static_assert(std::is_base_of_v<Component, T>, "T must inherit from Component.");

            std::vector<T*> result;

            for (const auto& component : m_Components)
            {
                if (component->IsPendingDestroy())
                {
                    continue;
                }

                T* typedComponent = dynamic_cast<T*>(component.get());

                if (typedComponent)
                {
                    result.push_back(typedComponent);
                }
            }

            for (const auto& component : m_PendingComponents)
            {
                if (component->IsPendingDestroy())
                {
                    continue;
                }

                T* typedComponent = dynamic_cast<T*>(component.get());

                if (typedComponent)
                {
                    result.push_back(typedComponent);
                }
            }

            return result;
        }

        template<typename T>
        bool HasComponent() const
        {
            static_assert(std::is_base_of_v<Component, T>, "T must inherit from Component.");

            return GetComponent<T>() != nullptr;
        }
    private:
        void RegisterComponent(ComponentTypeID typeID, Component* component);

        void UnregisterComponent(ComponentTypeID typeID, Component* component);

        void FlushDestroyedComponents();

        void FlushPendingComponents();

        void AddChildInternal(const EntityHandle& child);

        void RemoveChildInternal(EntityID childID);

        void DetachFromHierarchy();

    private:
        EntityID m_ID = InvalidEntityID;

        EntityGeneration m_Generation = InvalidEntityGeneration;

        std::string m_Name = "Entity";

        Scene* m_Scene = nullptr;

        bool m_Active = true;

        bool m_IsUpdating = false;

        bool m_HasStarted = false;

        bool m_PendingDestroy = false;

        std::vector<std::unique_ptr<Component>> m_Components;

        std::vector<std::unique_ptr<Component>> m_PendingComponents;

        std::unordered_map<ComponentTypeID, Component*> m_ComponentRegistry;

        std::unordered_set<std::string> m_Tags;

        EntityHandle m_Parent;

        std::vector<EntityHandle> m_Children;
    };
}