#include "EventBus.h"

#include <algorithm>

namespace Engine
{
    void EventBus::Compact()
    {
        for (auto& pair : m_Subscribers)
        {
            SubscriberList& subscribers = pair.second;

            auto iterator = std::remove_if(subscribers.begin(), subscribers.end(),
                [](const Subscriber& subscriber)
                {
                    return !subscriber.Active;
                }
            );

            subscribers.erase(iterator, subscribers.end());
        }
    }

    void EventBus::Clear()
    {
        m_Subscribers.clear();
    }

    std::size_t EventBus::GetSubscriberCount() const
    {
        std::size_t count = 0;

        for (const auto& pair : m_Subscribers)
        {
            for (const Subscriber& subscriber : pair.second)
            {
                if (subscriber.Active)
                {
                    ++count;
                }
            }
        }

        return count;
    }
}