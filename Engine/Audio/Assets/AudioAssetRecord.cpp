#include "AudioAssetRecord.h"

#include "AudioClip.h"

#include "../Streaming/AudioStream.h"

#include <atomic>
#include <utility>

namespace Engine
{
    AudioAssetRecord::AudioAssetRecord(AudioAssetHandle handle, AudioAssetType type, std::string path)
        : m_Handle(handle), m_Type(type), m_Path(std::move(path))
    {
    }

    AudioAssetRecord::~AudioAssetRecord() = default;

    AudioAssetHandle AudioAssetRecord::GetHandle() const
    {
        return m_Handle;
    }

    AudioAssetType AudioAssetRecord::GetType() const
    {
        return m_Type;
    }

    const std::string& AudioAssetRecord::GetPath() const
    {
        return m_Path;
    }

    AudioClip* AudioAssetRecord::GetClip()
    {
        return m_Clip.get();
    }

    const AudioClip* AudioAssetRecord::GetClip() const
    {
        return m_Clip.get();
    }

    AudioStream* AudioAssetRecord::GetStream()
    {
        return m_Stream.get();
    }

    const AudioStream* AudioAssetRecord::GetStream() const
    {
        return m_Stream.get();
    }

    void AudioAssetRecord::SetClip(std::unique_ptr<AudioClip> clip)
    {
        m_Clip = std::move(clip);
    }

    void AudioAssetRecord::SetStream(std::unique_ptr<AudioStream> stream)
    {
        m_Stream = std::move(stream);
    }

    void AudioAssetRecord::RetainPlaybackReference()
    {
        m_PlaybackReferences.fetch_add(1, std::memory_order_relaxed);
    }

    void AudioAssetRecord::ReleasePlaybackReference()
    {
        std::uint32_t current = m_PlaybackReferences.load(std::memory_order_relaxed);

        while (current != 0)
        {
            if (m_PlaybackReferences.compare_exchange_weak(current, current - 1, std::memory_order_release, std::memory_order_relaxed))
            {
                return;
            }
        }
    }

    std::uint32_t AudioAssetRecord::GetPlaybackReferenceCount() const
    {
        return m_PlaybackReferences.load(std::memory_order_acquire);
    }

    void AudioAssetRecord::RequestUnload()
    {
        m_UnloadRequested = true;
    }

    void AudioAssetRecord::CancelUnloadRequest()
    {
        m_UnloadRequested = false;
    }

    bool AudioAssetRecord::IsUnloadRequested() const
    {
        return m_UnloadRequested;
    }
}
