#pragma once

#include "Entity.h"
#include "EntityHandle.h"
#include "EntityID.h"

#include "../Events/EventBus.h"
#include "../Physics/PhysicsWorld2D.h"

#include <memory>
#include <string>
#include <functional>
#include <vector>
#include <unordered_map>

namespace Engine
{
    class Renderer2D;

    class Scene
    {
    public:
        Scene();

        explicit Scene(const std::string& name);

        ~Scene() = default;

        Entity* CreateEntity(const std::string& name);

        void DestroyEntity(Entity* entity);

        void DestroyEntity(EntityID id);

        EntityHandle CreateHandle(Entity* entity);

        EntityHandle CreateHandle(EntityID id);

        Entity* FindEntityByName(const std::string& name) const;

        Entity* FindEntityByID(EntityID id) const;

        std::vector<Entity*> FindEntitiesWithTag(const std::string& tag) const;

        void Start();

        void Update(float deltaTime);

        void Render(Renderer2D& renderer);

        void Clear();

        const std::string& GetName() const;

        void SetName(const std::string& name);

        std::size_t GetEntityCount() const;

        bool HasStarted() const;

        bool IsUpdating() const;

        bool ValidateEntityRegistry() const;

        std::size_t GetRegisteredEntityCount() const;

        std::vector<Entity*> GetRootEntities() const;

        EventBus& GetEventBus();

        const EventBus& GetEventBus() const;

        PhysicsWorld2D& GetPhysicsWorld();

        const PhysicsWorld2D& GetPhysicsWorld() const;

    private:
        EntityID GenerateEntityID();

        EntityGeneration GetNextGeneration(EntityID id);

        void RegisterEntity(Entity* entity);

        void UnregisterEntity(EntityID id);

        void FlushDestroyedEntities();

        void FlushPendingEntities();

    private:
        std::string m_Name = "Scene";

        bool m_HasStarted = false;

        bool m_IsUpdating = false;

        EventBus m_EventBus;

        EntityID m_NextEntityID = 1;

        std::vector<std::unique_ptr<Entity>> m_Entities;

        std::vector<std::unique_ptr<Entity>> m_PendingEntities;

        std::unordered_map<EntityID, Entity*> m_EntityRegistry;

        std::unordered_map<EntityID, EntityGeneration> m_EntityGenerations;

        PhysicsWorld2D m_PhysicsWorld;
    };
}