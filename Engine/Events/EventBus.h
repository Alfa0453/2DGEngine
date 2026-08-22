#pragma once

#include "EventSubscription.h"

#include <cstddef>
#include <functional>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace Engine
{
    class EventBus
    {
    public:
        EventBus() = default;

        EventBus(const EventBus&) = default;

        EventBus& operator=(const EventBus&) = delete;

        template<typename TEvent>
        EventSubscriptionID Subscribe(std::function<void(const TEvent&)> callback);

        template<typename TEvent>
        bool Unsubscribe(EventSubscriptionID id);

        template<typename TEvent>
        void Publish(const TEvent& event);

        void Compact();

        void Clear();

        std::size_t GetSubscriberCount() const;

    private:

        struct Subscriber
        {
            EventSubscriptionID ID = InvalidEventSubscriptionID;

            std::function<void(const void*)> Callback;

            bool Active = true;
        };

        using SubscriberList = std::vector<Subscriber>;

        std::unordered_map<std::type_index, SubscriberList> m_Subscribers;

        EventSubscriptionID m_NextSubscriptionID = 1;
    };
}

template<typename TEvent>
Engine::EventSubscriptionID Engine::EventBus::Subscribe(std::function<void(const TEvent&)> callback)
{
    if (!callback)
    {
        return InvalidEventSubscriptionID;
    }

    const EventSubscriptionID id =m_NextSubscriptionID++;

    Subscriber subscriber;

    subscriber.ID = id;

    subscriber.Callback =
        [callback = std::move(callback)](const void* eventData)
        {
            const TEvent* typedEvent = static_cast<const TEvent*>(eventData);

            callback(*typedEvent);
        };

    const std::type_index eventType = std::type_index(typeid(TEvent));

    m_Subscribers[eventType].push_back(std::move(subscriber));

    return id;
}

template<typename TEvent>
void Engine::EventBus::Publish(const TEvent& event)
{
    const std::type_index eventType = std::type_index(typeid(TEvent));

    auto iterator = m_Subscribers.find(eventType);

    if (iterator == m_Subscribers.end())
    {
        return;
    }

    SubscriberList& subscribers = iterator->second;

    for (Subscriber& subscriber : subscribers)
    {
        if (!subscriber.Active || !subscriber.Callback)
        {
            continue;
        }

        subscriber.Callback(&event);
    }
}

template<typename TEvent>
bool Engine::EventBus::Unsubscribe(EventSubscriptionID id)
{
    if (id == InvalidEventSubscriptionID)
    {
        return false;
    }

    const std::type_index eventType = std::type_index(typeid(TEvent));

    auto iterator = m_Subscribers.find(eventType);

    if (iterator == m_Subscribers.end())
    {
        return false;
    }

    for (Subscriber& subscriber : iterator->second)
    {
        if (subscriber.ID == id && subscriber.Active)
        {
            subscriber.Active = false;

            subscriber.Callback = {};

            return true;
        }
    }

    return false;
}