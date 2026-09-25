#include "AudioStream.h"

#include "AudioStreamBuffer.h"
#include "AudioStreamDecoder.h"
#include "AudioStreamDecoderFactory.h"
#include "AudioStreamState.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>

namespace Engine
{
    AudioStream::~AudioStream()
    {
        Close();
    }

    bool AudioStream::Open(const std::string &filePath, const AudioFormat &targetFormat, std::size_t bufferFrames, std::size_t decodeChunkFrames, std::size_t initialBufferedFrames)
    {
        Close();

        if (filePath.empty() || targetFormat.SampleRate <= 0 || targetFormat.Channels <= 0 || bufferFrames == 0 || decodeChunkFrames == 0)
        {
            return false;
        }

        m_Decoder = CreateAudioStreamDecoder(filePath);

        if (!m_Decoder)
        {
            m_State.store(AudioStreamState::Error, std::memory_order_release);

            return false;
        }

        if (!m_Decoder->Open(filePath, targetFormat))
        {
            m_Decoder.reset();

            m_State.store(AudioStreamState::Error, std::memory_order_release);

            return false;
        }

        m_Format = m_Decoder->GetFormat();

        if (!m_Buffer.Initialize(m_Format, bufferFrames))
        {
            m_Decoder->Close();

            m_Decoder.reset();

            m_State.store(AudioStreamState::Error, std::memory_order_release);

            return false;
        }

        m_FilePath = filePath;

        m_DecodeChunkFrames = decodeChunkFrames;

        m_InitialBufferedFrames = std::min(initialBufferedFrames, bufferFrames);

        if (m_InitialBufferedFrames == 0)
        {
            m_InitialBufferedFrames = std::min<std::size_t>(decodeChunkFrames, bufferFrames);
        }

        m_UnderflowCount.store(0, std::memory_order_relaxed);

        m_SeekRequested.store(false, std::memory_order_relaxed);

        m_ConsumerReading.store(false, std::memory_order_relaxed);

        m_ConsumerAttached.store(false, std::memory_order_relaxed);

        m_State.store(AudioStreamState::Buffering, std::memory_order_release);

        m_Running.store(true, std::memory_order_release);

        try
        {
            m_StreamingThread = std::thread(&AudioStream::StreamingThreadMain, this);
        }
        catch (...)
        {
            m_Running.store(false, std::memory_order_release);

            m_Buffer.Shutdown();

            m_Decoder->Close();

            m_Decoder.reset();

            m_State.store(AudioStreamState::Error, std::memory_order_release);

            return false;
        }

        return true;
    }

    void AudioStream::Close()
    {
        m_Running.store(false, std::memory_order_release);

        if (m_StreamingThread.joinable())
        {
            m_StreamingThread.join();
        }

        if (m_Decoder)
        {
            m_Decoder->Close();

            m_Decoder.reset();
        }

        m_Buffer.Shutdown();

        m_FilePath.clear();

        m_SeekRequested.store(false, std::memory_order_relaxed);

        m_ConsumerAttached.store(false, std::memory_order_relaxed);

        m_ConsumerReading.store(false, std::memory_order_relaxed);

        m_State.store(AudioStreamState::Closed, std::memory_order_release);
    }

    bool AudioStream::IsOpen() const
    {
        const AudioStreamState state = m_State.load(std::memory_order_acquire);

        return state != AudioStreamState::Closed && state != AudioStreamState::Error;
    }

    bool AudioStream::IsReady() const
    {
        const AudioStreamState state = GetState();

        if (state == AudioStreamState::Playing)
        {
            return true;
        }

        if (state == AudioStreamState::Ended)
        {
            return m_Buffer.GetAvailableFrames() > 0;
        }

        return false;
    }

    AudioStreamState AudioStream::GetState() const
    {
        return m_State.load(std::memory_order_acquire);
    }

    const AudioFormat& AudioStream::GetFormat() const
    {
        return m_Format;
    }

    std::uint64_t AudioStream::GetTotalFrameCount() const
    {
        if (!m_Decoder)
        {
            return 0;
        }

        return m_Decoder->GetTotalFrameCount();
    }

    float AudioStream::GetDurationSeconds() const
    {
        if (!m_Decoder || m_Format.SampleRate <= 0)
        {
            return 0.0f;
        }

        return static_cast<float>(m_Decoder->GetTotalFrameCount()) / static_cast<float>(m_Format.SampleRate);
    }

    std::size_t AudioStream::ReadFrames(float *output, std::size_t frameCount)
    {
        if (!output || frameCount == 0)
        {
            return 0;
        }

        AudioStreamState state = m_State.load(std::memory_order_acquire);

        if (state == AudioStreamState::Closed || state == AudioStreamState::Error || state == AudioStreamState::Buffering || state == AudioStreamState::Seeking)
        {
            return 0;
        }

        // Tell worker that buffer-reset must wait.
        m_ConsumerReading.store(true, std::memory_order_release);

        // Recheck after publishing consumer-active in case worker/game requested seek in between.
        state = m_State.load(std::memory_order_acquire);

        if (state == AudioStreamState::Seeking || state == AudioStreamState::Closed || state == AudioStreamState::Error)
        {
            m_ConsumerReading.store(false, std::memory_order_release);

            return 0;
        }

        const std::size_t read = m_Buffer.ReadFrames(output, frameCount);

        m_ConsumerReading.store(false, std::memory_order_release);

        return read;
    }

    std::size_t AudioStream::GetAvailableFrames() const
    {
        return m_Buffer.GetAvailableFrames();
    }

    bool AudioStream::RequestSeekSeconds(float seconds)
    {
        if (!IsOpen())
        {
            return false;
        }

        seconds = std::max(seconds, 0.0f);

        const std::uint64_t frame = static_cast<std::uint64_t>(seconds * static_cast<float>(m_Format.SampleRate));

        return RequestSeekFrame(frame);
    }

    bool AudioStream::RequestSeekFrame(std::uint64_t frame)
    {
        if (!IsOpen())
        {
            return false;
        }

        const std::uint64_t total = GetTotalFrameCount();

        frame = std::min(frame, total);

        m_RequestedSeekFrame.store(frame, std::memory_order_release);

        // Consumer immediately stops taking old PCM.
        m_State.store(AudioStreamState::Seeking, std::memory_order_release);

        m_SeekRequested.store(true, std::memory_order_release);

        return true;
    }

    void AudioStream::SetLooping(bool looping)
    {
        m_Looping.store(looping, std::memory_order_release);
    }

    bool AudioStream::IsLooping() const
    {
        return m_Looping.load(std::memory_order_acquire);
    }

    bool AudioStream::HasCompletelyEnded() const
    {
        return GetState() == AudioStreamState::Ended && m_Buffer.GetAvailableFrames() == 0;
    }

    std::uint64_t AudioStream::GetUnderflowCount() const
    {
        return m_UnderflowCount.load(std::memory_order_acquire);
    }

    void AudioStream::RecordUnderflow()
    {
        m_UnderflowCount.fetch_add(1, std::memory_order_relaxed);
    }

    bool AudioStream::TryAcquireConsumer()
    {
        bool expected = false;

        return m_ConsumerAttached.compare_exchange_strong(expected, true, std::memory_order_acq_rel, std::memory_order_acquire);
    }

    void AudioStream::ReleaseConsumer()
    {
        m_ConsumerAttached.store(false, std::memory_order_release);
    }

    bool AudioStream::HasConsumer() const
    {
        return m_ConsumerAttached.load(std::memory_order_acquire);
    }

    bool AudioStream::HandlePendingSeek()
    {
        if (!m_SeekRequested.exchange(false, std::memory_order_acq_rel))
        {
            return true;
        }

        // This wait happens ONLY on streaming worker.
        // Never on SDL callback.
        while (m_Running.load(std::memory_order_acquire) && m_ConsumerReading.load(std::memory_order_acquire))
        {
            std::this_thread::yield();
        }

        if (!m_Running.load(std::memory_order_acquire))
        {
            return false;
        }

        m_Buffer.Reset();

        const std::uint64_t frame = m_RequestedSeekFrame.load(std::memory_order_acquire);

        if (!m_Decoder->SeekFrame(frame))
        {
            m_State.store(AudioStreamState::Error, std::memory_order_release);

            return false;
        }

        m_State.store(AudioStreamState::Buffering, std::memory_order_release);

        return true;
    }

    void AudioStream::StreamingThreadMain()
    {
        const std::size_t channels = static_cast<std::size_t>(m_Format.Channels);

        std::vector<float> decodeScratch;

        decodeScratch.resize(m_DecodeChunkFrames * channels);

        while (m_Running.load(std::memory_order_acquire))
        {
            if (!HandlePendingSeek())
            {
                continue;
            }

            if (GetState() == AudioStreamState::Error)
            {
                break;
            }

            const std::size_t writable = m_Buffer.GetWritableFrames();

            if (writable == 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));

                continue;
            }

            const std::size_t framesRequested = std::min(writable, m_DecodeChunkFrames);

            const std::size_t decoded = m_Decoder->DecodeFrames(decodeScratch.data(), framesRequested);

            if (decoded > 0)
            {
                const std::size_t written = m_Buffer.WriteFrames(decodeScratch.data(), decoded);

                if (written != decoded)
                {
                    // Should not normally happen because we checked writable frames.
                    m_State.store(AudioStreamState::Error, std::memory_order_release);

                    break;
                }

                if (GetState() == AudioStreamState::Buffering)
                {
                    if (m_Buffer.GetAvailableFrames() >= m_InitialBufferedFrames)
                    {
                        m_State.store(AudioStreamState::Playing, std::memory_order_release);
                    }
                }

                continue;
            }

            // Decoder returned zero => EOF.
            if (IsLooping())
            {
                if (!m_Decoder->SeekFrame(0))
                {
                    m_State.store(AudioStreamState::Error, std::memory_order_release);

                    break;
                }

                // Continue immediately so the ring buffer can remain full across loop boundary
                continue;
            }

            // EOF reached. Final PCM remains consumable.
            m_State.store(AudioStreamState::Ended, std::memory_order_release);

            // Stay alive for Seek requests.
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}
