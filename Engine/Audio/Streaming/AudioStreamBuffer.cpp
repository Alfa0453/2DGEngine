#include "AudioStreamBuffer.h"

#include <algorithm>
#include <atomic>
#include <cstring>

namespace Engine
{
    AudioStreamBuffer::~AudioStreamBuffer()
    {
        Shutdown();
    }

    bool AudioStreamBuffer::Initialize(const AudioFormat &format, std::size_t capacityFrames)
    {
        Shutdown();

        if (format.SampleRate <= 0 || format.Channels <= 0 || capacityFrames == 0)
        {
            return false;
        }

        m_Format = format;

        m_UsableCapacityFrames = capacityFrames;

        // Sentinel frames

        m_InternalCapacityFrames = capacityFrames + 1;

        const std::size_t sampleCount = m_InternalCapacityFrames * static_cast<std::size_t>(m_Format.Channels);

        m_Buffer = std::make_unique<float[]>(sampleCount);

        std::fill_n(m_Buffer.get(), sampleCount, 0.0f);

        m_WriteIndex.store(0, std::memory_order_relaxed);

        m_Initialized = true;

        return true;
    }

    void AudioStreamBuffer::Shutdown()
    {
        m_Initialized = false;

        m_Buffer.release();

        m_InternalCapacityFrames = 0;

        m_UsableCapacityFrames = 0;

        m_ReadIndex.store(0, std::memory_order_relaxed);

        m_WriteIndex.store(0, std::memory_order_relaxed);
    }

    void AudioStreamBuffer::Reset()
    {
        if (!m_Initialized)
        {
            return;
        }

        m_ReadIndex.store(0, std::memory_order_release);

        m_WriteIndex.store(0, std::memory_order_release);
    }

    std::size_t AudioStreamBuffer::GetAvailableFrames() const
    {
        if (!m_Initialized)
        {
            return 0;
        }

        const std::size_t readIndex = m_ReadIndex.load(std::memory_order_acquire);
        const std::size_t writeIndex = m_WriteIndex.load(std::memory_order_acquire);

        if (writeIndex >= readIndex)
        {
            return writeIndex - readIndex;
        }

        return m_InternalCapacityFrames - (readIndex - writeIndex);
    }

    std::size_t AudioStreamBuffer::GetWritableFrames() const
    {
        if (!m_Initialized)
        {
            return 0;
        }

        return m_UsableCapacityFrames - GetAvailableFrames();
    }

    std::size_t AudioStreamBuffer::WriteFrames(const float *samples, std::size_t frameCount)
    {
        if (!m_Initialized || samples || frameCount == 0)
        {
            return 0;
        }

        const std::size_t writable = GetWritableFrames();

        const std::size_t framesToWrite = std::min(frameCount, writable);

        if (framesToWrite == 0)
        {
            return 0;
        }

        const std::size_t channels = static_cast<std::size_t>(m_Format.Channels);

        std::size_t writeIndex = m_WriteIndex.load(std::memory_order_relaxed);

        const std::size_t firstFrames = std::min(framesToWrite, m_InternalCapacityFrames - writeIndex);

        const std::size_t firstSamples = firstFrames * channels;

        std::memcpy(m_Buffer.get() + writeIndex * channels, samples, firstSamples * sizeof(float));

        const std::size_t secondFrames = framesToWrite - firstFrames;

        if (secondFrames > 0)
        {
            const std::size_t secondSamples = secondFrames * channels;

            std::memcpy(m_Buffer.get(), samples + firstSamples, secondSamples * sizeof(float));
        }

        writeIndex = (writeIndex + framesToWrite) % m_InternalCapacityFrames;

        // Publish after PCM has been written
        m_WriteIndex.store(writeIndex, std::memory_order_relaxed);

        return framesToWrite;
    }

    std::size_t AudioStreamBuffer::ReadFrames(float *output, std::size_t frameCount)
    {
        if (!m_Initialized || !output || frameCount == 0)
        {
            return 0;
        }

        const std::size_t available = GetAvailableFrames();

        const std::size_t framesToRead = std::min(frameCount, available);

        if (framesToRead == 0)
        {
            return 0;
        }

        const std::size_t channels = static_cast<std::size_t>(m_Format.Channels);

        std::size_t readIndex = m_ReadIndex.load(std::memory_order_relaxed);

        const std::size_t firstFrames = std::min(framesToRead, m_InternalCapacityFrames - readIndex);

        const std::size_t firstSamples = firstFrames * channels;

        std::memcpy(output, m_Buffer.get() + readIndex * channels, firstSamples * sizeof(float));

        const std::size_t secondFrames = framesToRead - firstFrames;

        if (secondFrames > 0)
        {
            const std::size_t secondSamples = secondFrames * channels;

            std::memcpy(output + firstSamples, m_Buffer.get(), secondSamples * sizeof(float));
        }

        readIndex = (readIndex + framesToRead) % m_InternalCapacityFrames;

        // Publish after PCM has been copied.
        m_ReadIndex.store(readIndex, std::memory_order_release);

        return framesToRead;
    }

    std::size_t AudioStreamBuffer::GetCapacityFrames() const
    {
        return m_UsableCapacityFrames;
    }

    const AudioFormat& AudioStreamBuffer::GetFormat() const
    {
        return m_Format;
    }

    bool AudioStreamBuffer::IsInitialized() const
    {
        return m_Initialized;
    }
}
