#pragma once

#include "AudioCommand.h"

#include "../Threading/SPSCQueue.h"

#include <cstddef>

namespace Engine
{
    class AudioCommandQueue
    {
    public:

        AudioCommandQueue() = default;

        bool Initialize(std::size_t capacity);

        void Shutdown();

        bool Push(const AudioCommand& command);

        bool TryPop(AudioCommand& outCommand);

        bool IsEmpty() const;

        std::size_t GetPendingCount() const;

        std::size_t GetCapacity() const;

        void Reset();

    private:

        SPSCQueue<AudioCommand> m_Queue;
    };
}