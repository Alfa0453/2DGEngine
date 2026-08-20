#pragma once

#include "ComponentTypeID.h"

namespace Engine
{
    class Entity;

    class Component
    {
    public:
        Component() = default;

        virtual ~Component() = default;

        virtual void Start()
        {
        }

        virtual void Update(float deltaTime)
        {
        }

        virtual void OnDestroy()
        {
        }

        Entity* GetOwner() const;

        ComponentTypeID GetTypeID() const;

        bool HasStarted() const;

        void Destroy();

        bool IsPendingDestroy() const;

    protected:
        friend class Entity;

        void SetOwner(Entity* owner);

        void SetTypeID(ComponentTypeID typeID);

        void StartInternal();

        void DestroyInternal();

    private:
        Entity* m_Owner = nullptr;

        ComponentTypeID m_TypeID = InvalidComponentTypeID;

        bool m_HasStarted = false;

        bool m_HasBeenDestroyed = false;

        bool m_PendingDestroy = false;
    };
}