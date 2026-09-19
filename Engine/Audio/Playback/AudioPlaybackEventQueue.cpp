#include "AudioPlaybackEventQueue.h"

namespace Engine
{
    bool AudioPlaybackEventQueue::Initialize(std::size_t capacity)
    {
        return m_Queue.Initialize(capacity);
    }

    void AudioPlaybackEventQueue::Shutdown()
    {
        m_Queue.Shutdown();
    }

    bool AudioPlaybackEventQueue::Push(const AudioPlaybackEvent& event)
    {
        return m_Queue.Push(event);
    }

    bool AudioPlaybackEventQueue::TryPop(AudioPlaybackEvent& outEvent)
    {
        return m_Queue.TryPop(outEvent);
    }

    bool AudioPlaybackEventQueue::IsEmpty() const
    {
        return m_Queue.IsEmpty();
    }

    std::size_t AudioPlaybackEventQueue::GetPendingCount() const
    {
        return m_Queue.GetApproximateCount();
    }

    std::size_t AudioPlaybackEventQueue::GetCapacity() const
    {
        return m_Queue.GetCapacity();
    }

    void AudioPlaybackEventQueue::Reset()
    {
        m_Queue.Reset();
    }
}