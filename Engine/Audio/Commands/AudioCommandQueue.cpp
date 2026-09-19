#include "AudioCommandQueue.h"

namespace Engine
{
    bool AudioCommandQueue::Initialize(std::size_t capacity)
    {
        return m_Queue.Initialize(capacity);
    }

    void AudioCommandQueue::Shutdown()
    {
        m_Queue.Shutdown();
    }

    bool AudioCommandQueue::Push(const AudioCommand& command)
    {
        return m_Queue.Push(command);
    }

    bool AudioCommandQueue::TryPop(AudioCommand& outCommand)
    {
        return m_Queue.TryPop(outCommand);
    }

    bool AudioCommandQueue::IsEmpty() const
    {
        return m_Queue.IsEmpty();
    }

    std::size_t AudioCommandQueue::GetPendingCount() const
    {
        return m_Queue.GetApproximateCount();
    }

    std::size_t AudioCommandQueue::GetCapacity() const
    {
        return m_Queue.GetCapacity();
    }

    void AudioCommandQueue::Reset()
    {
        m_Queue.Reset();
    }
}