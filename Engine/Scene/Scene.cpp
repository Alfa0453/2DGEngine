#include "Scene.h"

#include "../Graphics/Renderer2D.h"
#include "../Graphics/SortingLayer.h"
#include "../Graphics/Camera2D.h"
#include "Entity.h"
#include "EntityID.h"
#include "SpriteRendererComponent.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

namespace
{
    struct RenderQueueEntry
    {
        Engine::SpriteRendererComponent* SpriteRenderer = nullptr;

        Engine::EntityID Entity = Engine::InvalidEntityID;
    };
}

namespace Engine
{
    Scene::Scene()
        : Scene("Scene")
    {
    }

    Scene::Scene(const std::string& name)
        : m_Name(name), m_PhysicsWorld(this)
    {
    }

    Entity* Scene::CreateEntity(const std::string& name)
    {
        const EntityID id = GenerateEntityID();

        const EntityGeneration generation = GetNextGeneration(id);

        auto entity = std::make_unique<Entity>(this, id, generation, name);

        Entity* entityPointer = entity.get();

        RegisterEntity(entityPointer);

        if (m_IsUpdating)
        {
            m_PendingEntities.push_back(std::move(entity));
        }
        else
        {
            m_Entities.push_back(std::move(entity));

            if (m_HasStarted)
            {
                entityPointer->Start();
            }
        }

        return entityPointer;
    }

    EntityID Scene::GenerateEntityID()
    {
        return m_NextEntityID++;
    }

    EntityGeneration Scene::GetNextGeneration(EntityID id)
    {
        EntityGeneration& generation = m_EntityGenerations[id];

        if (generation == InvalidEntityGeneration)
        {
            generation = 1;
        }
        else 
        {
            ++generation;
        }

        return generation;
    }

    void Scene::DestroyEntity(Entity* entity)
    {
        if (!entity)
        {
            return;
        }

        entity->Destroy();
    }

    void Scene::DestroyEntity(EntityID id)
    {
        Entity* entity = FindEntityByID(id);

        if (!entity)
        {
            return;
        }

        entity->Destroy();
    }

    EntityHandle Scene::CreateHandle(Entity* entity)
    {
        if (!entity || entity->IsPendingDestroy())
        {
            return {};
        }

        return EntityHandle(this, entity->GetID(), entity->GetGeneration());
    }

    EntityHandle Scene::CreateHandle(EntityID id)
    {
        Entity* entity = FindEntityByID(id);

        if (!entity)
        {
            return {};
        }

        return CreateHandle(entity);
    }

    Entity* Scene::FindEntityByName(const std::string& name) const
    {
        for (const auto& entity : m_Entities)
        {
            if (!entity->IsPendingDestroy() && entity->GetName() == name)
            {
                return entity.get();
            }
        }

        for (const auto& entity : m_PendingEntities)
        {
            if (!entity->IsPendingDestroy() && entity->GetName() == name)
            {
                return entity.get();
            }
        }

        return nullptr;
    }

    Entity* Scene::FindEntityByID(EntityID id) const
    {
        if (id == InvalidEntityID)
        {
            return nullptr;
        }

        auto iterator = m_EntityRegistry.find(id);

        if (iterator == m_EntityRegistry.end())
        {
            return nullptr;
        }

        Entity* entity = iterator->second;

        if (!entity)
        {
            return nullptr;
        }

        if (entity->IsPendingDestroy())
        {
            return nullptr;
        }

        return entity;
    }

    std::vector<Entity*> Scene::FindEntitiesWithTag(const std::string& tag) const
    {
        std::vector<Entity*> result;

        for (const auto& entity : m_Entities)
        {
            if (entity->HasTag(tag))
            {
                result.push_back(entity.get());
            }
        }

        return result;
    }

    void Scene::RegisterEntity(Entity* entity)
    {
        if (!entity)
        {
            return;
        }

        const EntityID id = entity->GetID();

        if (id == InvalidEntityID)
        {
            return;
        }

        auto iterator = m_EntityRegistry.find(id);

        if (iterator != m_EntityRegistry.end())
        {
            return;
        }

        m_EntityRegistry.emplace(id, entity);
    }

    void Scene::UnregisterEntity(EntityID id)
    {
        if (id == InvalidEntityID)
        {
            return;
        }

        m_EntityRegistry.erase(id);
    }

    void Scene::Start()
    {
        if (m_HasStarted)
        {
            return;
        }

        m_HasStarted = true;

        for (const auto& entity : m_Entities)
        {
            if (entity->IsActive())
            {
                entity->Start();
            }
        }
    }

    void Scene::Update(float deltaTime)
    {
        m_IsUpdating = true;

        for (const auto& entity : m_Entities)
        {
            if (entity->IsActive() && !entity->IsPendingDestroy())
            {
                entity->Update(deltaTime);
            }
        }

        m_PhysicsWorld.Update(deltaTime);

        m_IsUpdating = false;

        FlushDestroyedEntities();

        FlushPendingEntities();

        m_EventBus.Compact();
    }

    void Scene::Render(Renderer2D& renderer)
    {
        std::vector<RenderQueueEntry> renderQueue;

        renderQueue.reserve(m_Entities.size());

        Camera2D* camera = renderer.GetCamera();

        for (const auto& entity : m_Entities)
        {
            if (!entity || !entity->IsActive() || entity->IsPendingDestroy())
            {
                continue;
            }

            SpriteRendererComponent* spriteRenderer = entity->GetComponent<SpriteRendererComponent>();

            if (!spriteRenderer)
            {
                continue;
            }

            renderer.NotifySpriteSubmitted();

            if (camera)
            {
                const Bounds2D bounds = spriteRenderer->GetWorldBounds();

                if (!camera->IsBoundsVisible(bounds))
                {
                    renderer.NotifySpriteCulled();

                    continue;
                }
            }

            renderQueue.push_back( {spriteRenderer, entity->GetID()} );
        }

        std::sort(renderQueue.begin(), renderQueue.end(),
            [](const RenderQueueEntry& a, const RenderQueueEntry& b)
            {
                const auto aLayer = static_cast<std::int32_t>(a.SpriteRenderer->GetSortingLayer());

                const auto bLayer = static_cast<std::int32_t>(b.SpriteRenderer->GetSortingLayer());

                if (aLayer != bLayer)
                {
                    return aLayer < bLayer;
                }

                const std::int32_t aOrder = a.SpriteRenderer->GetOrderInLayer();

                const std::int32_t bOrder = b.SpriteRenderer->GetOrderInLayer();

                if (aOrder != bOrder)
                {
                    return aOrder < bOrder;
                }

                return a.Entity < b.Entity;
            }
        );

        for (const RenderQueueEntry& entry : renderQueue)
        {
            if (!entry.SpriteRenderer)
            {
                continue;
            }

            entry.SpriteRenderer->Render(renderer);

            renderer.NotifySpriteRendered();
        }
    }

    void Scene::Clear()
    {
        m_EntityRegistry.clear();

        m_PendingEntities.clear();

        m_Entities.clear();
    }

    const std::string& Scene::GetName() const
    {
        return m_Name;
    }

    void Scene::SetName(const std::string& name)
    {
        m_Name = name;
    }

    std::size_t Scene::GetEntityCount() const
    {
        std::size_t count = 0;

        for (const auto& entity : m_Entities)
        {
            if (!entity->IsPendingDestroy())
            {
                ++count;
            }
        }

        for (const auto& entity : m_PendingEntities)
        {
            if (!entity->IsPendingDestroy())
            {
                ++count;
            }
        }

        return count;
    }

    bool Scene::HasStarted() const
    {
        return m_HasStarted;
    }

    bool Scene::IsUpdating() const
    {
        return m_IsUpdating;
    }

    void Scene::FlushDestroyedEntities()
    {
        for (auto iterator = m_Entities.begin(); iterator != m_Entities.end();)
        {
            Entity* entity = iterator->get();

            if (entity && entity->IsPendingDestroy())
            {
                UnregisterEntity(entity->GetID());

                iterator = m_Entities.erase(iterator);
            }
            else 
            {
                ++iterator;
            }
        }
    }

    void Scene::FlushPendingEntities()
    {
        for (auto& entity : m_PendingEntities)
        {
            if (entity->IsPendingDestroy())
            {
                UnregisterEntity(entity->GetID());

                continue;
            }

            Entity* pointer = entity.get();

            m_Entities.push_back(std::move(entity));

            if (m_HasStarted)
            {
                pointer->Start();
            }
        }

        m_PendingEntities.clear();
    }

    bool Scene::ValidateEntityRegistry() const
    {
        for (const auto& entity : m_Entities)
        {
            if (!entity)
            {
                return false;
            }

            if (entity->IsPendingDestroy())
            {
                return false;
            }

            auto iterator = m_EntityRegistry.find(entity->GetID());

            if (iterator == m_EntityRegistry.end())
            {
                return false;
            }

            if (iterator->second != entity.get())
            {
                return false;
            }
        }

        for (const auto& entity : m_PendingEntities)
        {
            if (!entity)
            {
                return false;
            }

            if (entity->IsPendingDestroy())
            {
                continue;
            }

            auto iterator = m_EntityRegistry.find(entity->GetID());

            if (iterator == m_EntityRegistry.end())
            {
                return false;
            }

            if (iterator->second != entity.get())
            {
                return false;
            }
        }

        return true;
    }

    std::size_t Scene::GetRegisteredEntityCount() const
    {
        return m_EntityRegistry.size();
    }

    std::vector<Entity*> Scene::GetRootEntities() const
    {
        std::vector<Entity*> result;

        for (const auto& entity : m_Entities)
        {
            if (!entity || entity->IsPendingDestroy())
            {
                continue;
            }

            if (entity->IsRoot())
            {
                result.push_back(entity.get());
            }
        }

        return result;
    }

    EventBus& Scene::GetEventBus()
    {
        return m_EventBus;
    }

    const EventBus& Scene::GetEventBus() const
    {
        return m_EventBus;
    }

    PhysicsWorld2D& Scene::GetPhysicsWorld()
    {
        return m_PhysicsWorld;
    }

    const PhysicsWorld2D& Scene::GetPhysicsWorld() const
    {
        return m_PhysicsWorld;
    }
}