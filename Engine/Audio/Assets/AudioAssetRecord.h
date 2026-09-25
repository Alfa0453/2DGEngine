#pragma once

#include "AudioAssetHandle.h"
#include "AudioAssetType.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace Engine
{
    class AudioClip;
    class AudioStream;

    class AudioAssetRecord
    {
    public:

        AudioAssetRecord(AudioAssetHandle handle, AudioAssetType type, std::string path);

        ~AudioAssetRecord();

        AudioAssetRecord(const AudioAssetRecord&) = delete;

        AudioAssetRecord& operator=(const AudioAssetRecord&) = delete;

        AudioAssetHandle GetHandle() const;

        AudioAssetType GetType() const;

        const std::string& GetPath() const;

        AudioClip* GetClip();

        const AudioClip* GetClip() const;

        AudioStream* GetStream();

        const AudioStream* GetStream() const;

        void SetClip(std::unique_ptr<AudioClip> clip);

        void SetStream(std::unique_ptr<AudioStream> stream);

        void RetainPlaybackReference();

        void ReleasePlaybackReference();

        std::uint32_t GetPlaybackReferenceCount() const;

        void RequestUnload();

        void CancelUnloadRequest();

        bool IsUnloadRequested() const;

    private:

        AudioAssetHandle m_Handle;

        AudioAssetType m_Type = AudioAssetType::None;

        std::string m_Path;

        std::unique_ptr<AudioClip> m_Clip;

        std::unique_ptr<AudioStream> m_Stream;

        std::atomic<std::uint32_t> m_PlaybackReferences{0};

        // Resource-manager/game-thread state.
        bool m_UnloadRequested = false;
    };
}
