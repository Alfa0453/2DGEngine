#pragma once

#include "../Types/AudioFormat.h"

#include <atomic>
#include <cstddef>
#include <memory>

namespace Engine
{
    class AudioStreamBuffer
    {
    public:

        AudioStreamBuffer() = default;

        ~AudioStreamBuffer();

        bool Initialize(const AudioFormat& format, std::size_t capacityFrames);

        void Shutdown();

        void Reset();

        std::size_t WriteFrames(const float* samples, std::size_t frameCount);

        std::size_t ReadFrames(float* output, std::size_t frameCount);

        std::size_t GetAvailableFrames() const;

        std::size_t GetWritableFrames() const;

        std::size_t GetCapacityFrames() const;

        const AudioFormat& GetFormat() const;

        bool IsInitialized() const;

    private:

        AudioFormat m_Format;

        std::unique_ptr<float[]> m_Buffer;

        std::size_t m_InternalCapacityFrames = 0;

        std::size_t m_UsableCapacityFrames = 0;

        std::atomic<std::size_t> m_ReadIndex{0};

        std::atomic<std::size_t> m_WriteIndex{0};

        bool m_Initialized = false;
    };
}
