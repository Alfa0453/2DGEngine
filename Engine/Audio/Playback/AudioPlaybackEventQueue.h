#pragma once

#include "AudioPlaybackEvent.h"

#include "../Threading/SPSCQueue.h"

#include <cstddef>

namespace Engine
{
    class AudioPlaybackEventQueue
    {
    public:
        AudioPlaybackEventQueue() = default;

        bool Initialize(std::size_t capacity);

        void Shutdown();

        bool Push(const AudioPlaybackEvent& event);

        bool TryPop(AudioPlaybackEvent& outEvent);

        bool IsEmpty() const;

        std::size_t GetPendingCount() const;

        std::size_t GetCapacity() const;

        void Reset();

    private:

        SPSCQueue<AudioPlaybackEvent> m_Queue;
    };
}