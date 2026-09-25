#pragma once

#include "AudioStreamBuffer.h"
#include "AudioStreamDecoder.h"
#include "AudioStreamState.h"

#include "../Types/AudioFormat.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace Engine
{
    class AudioStreamDecoder;

    class AudioStream
    {
    public:

        AudioStream() = default;

        ~AudioStream();

        AudioStream(const AudioStream&) = delete;

        AudioStream& operator=(const AudioStream&) = delete;

        bool Open(const std::string& filePath, const AudioFormat& targetFormat, std::size_t bufferFrames, std::size_t decodeChunkFrames, std::size_t initialBufferedFrames);

        void Close();

        bool IsOpen() const;

        bool IsReady() const;

        AudioStreamState GetState() const;

        const AudioFormat& GetFormat() const;

        std::uint64_t GetTotalFrameCount() const;

        float GetDurationSeconds() const;

        std::size_t ReadFrames(float* output, std::size_t frameCount);

        std::size_t GetAvailableFrames() const;

        bool RequestSeekSeconds(float seconds);

        bool RequestSeekFrame(std::uint64_t frame);

        void SetLooping(bool looping);

        bool IsLooping() const;

        bool HasCompletelyEnded() const;

        std::uint64_t GetUnderflowCount() const;

        void RecordUnderflow();

        bool TryAcquireConsumer();

        void ReleaseConsumer();

        bool HasConsumer() const;

    private:

        void StreamingThreadMain();

        bool HandlePendingSeek();

    private:

        std::unique_ptr<AudioStreamDecoder> m_Decoder;

        AudioStreamBuffer m_Buffer;

        AudioFormat m_Format;

        std::string m_FilePath;

        std::thread m_StreamingThread;

        std::size_t m_DecodeChunkFrames = 1024;

        std::size_t m_InitialBufferedFrames = 4096;

        std::atomic<AudioStreamState> m_State{AudioStreamState::Closed};

        std::atomic<bool> m_Running{false};

        std::atomic<bool> m_Looping{false};

        std::atomic<bool> m_SeekRequested{false};

        std::atomic<std::uint64_t> m_RequestedSeekFrame{0};

        // Allows worker to reset buffer only when callback isn't inside ReadFrames().
        std::atomic<bool> m_ConsumerReading{false};

        // Enforces one active consuming Voice per stream.
        std::atomic<bool> m_ConsumerAttached{false};

        std::atomic<std::uint64_t> m_UnderflowCount{0};
    };
}
