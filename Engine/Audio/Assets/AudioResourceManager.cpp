#include "AudioResourceManager.h"

#include "AudioAssetHandle.h"
#include "AudioAssetRecord.h"
#include "AudioAssetType.h"
#include "AudioClip.h"

#include "../Streaming/AudioStream.h"

#include <cstddef>
#include <filesystem>
#include <memory>
#include <utility>

namespace Engine
{
    AudioResourceManager::~AudioResourceManager()
    {
        Shutdown();
    }

    bool AudioResourceManager::Initialize(const AudioSettings &settings)
    {
        if (m_Initialized)
        {
            return true;
        }

        m_Settings = settings;

        m_Assets.clear();

        m_Generations.clear();

        m_PathCache.clear();

        m_Initialized = true;

        return true;
    }

    void AudioResourceManager::Shutdown()
    {
        if (!m_Initialized)
        {
            return;
        }

        // AudioSystem must already have been shutdown.
        for (std::unique_ptr<AudioAssetRecord>& asset : m_Assets)
        {
            if (!asset)
            {
                continue;
            }

            AudioStream* stream = asset->GetStream();

            if (stream)
            {
                stream->Close();
            }
        }

        m_PathCache.clear();

        m_Assets.clear();

        m_Generations.clear();

        m_Initialized = false;
    }

    bool AudioResourceManager::IsInitialized() const
    {
        return m_Initialized;
    }

    std::string AudioResourceManager::NormalizePath(const std::string &filePath) const
    {
        if (filePath.empty())
        {
            return {};
        }

        const std::filesystem::path path = std::filesystem::path(filePath).lexically_normal();

        return path.generic_string();
    }

    std::string AudioResourceManager::BuildCacheKey(AudioAssetType type, const std::string &normalizedPath) const
    {
        switch (type)
        {
            case AudioAssetType::Clip:
            {
                return "clip:" + normalizedPath;
            }
            case AudioAssetType::Stream:
            {
                return "stream:" + normalizedPath;
            }

            case AudioAssetType::None:
            default:
            {
                return {};
            }
        }
    }

    AudioAssetHandle AudioResourceManager::FindCached(AudioAssetType type, const std::string &normalizedPath)
    {
        const std::string key = BuildCacheKey(type, normalizedPath);

        const auto it = m_PathCache.find(key);

        if (it == m_PathCache.end())
        {
            return {};
        }

        AudioAssetRecord* asset = Resolve(it->second);

        if (!asset)
        {
            m_PathCache.erase(it);

            return {};
        }

        // Loading it again cancels a pending unload.
        asset->CancelUnloadRequest();

        return asset->GetHandle();
    }

    AudioAssetHandle AudioResourceManager::AllocateRecord(AudioAssetType type, const std::string &normalizedPath)
    {
        std::size_t slotIndex = m_Assets.size();

        for (std::size_t i = 0; i < m_Assets.size(); ++i)
        {
            if (!m_Assets[i])
            {
                slotIndex = i;

                break;
            }
        }

        if (slotIndex == m_Assets.size())
        {
            m_Assets.push_back(nullptr);

            m_Generations.push_back(1);
        }

        AudioAssetHandle handle;

        handle.ID = static_cast<std::uint32_t>(slotIndex);

        handle.Generation = m_Generations[slotIndex];

        m_Assets[slotIndex] = std::make_unique<AudioAssetRecord>(handle, type, normalizedPath);

        return handle;
    }

    AudioAssetHandle AudioResourceManager::LoadClip(const std::string& filePath)
    {
        if (!m_Initialized)
        {
            return {};
        }

        const std::string normalizedPath = NormalizePath(filePath);

        if (normalizedPath.empty())
        {
            return {};
        }

        const AudioAssetHandle cached = FindCached(AudioAssetType::Clip, normalizedPath);

        if (cached.IsValid())
        {
            return cached;
        }

        auto clip = std::make_unique<AudioClip>();

        if (!clip->LoadFromWav(normalizedPath, m_Settings.OutputFormat))
        {
            return {};
        }

        const AudioAssetHandle handle = AllocateRecord(AudioAssetType::Clip, normalizedPath);

        AudioAssetRecord* record = Resolve(handle);

        if (!record)
        {
            return {};
        }

        record->SetClip(std::move(clip));

        m_PathCache[BuildCacheKey(AudioAssetType::Clip, normalizedPath)] = handle;

        return handle;
    }

    AudioAssetHandle AudioResourceManager::LoadStream(const std::string &filePath)
    {
        if (!m_Initialized)
        {
            return {};
        }

        const std::string normalizedPath = NormalizePath(filePath);

        if (normalizedPath.empty())
        {
            return {};
        }

        const AudioAssetHandle cached = FindCached(AudioAssetType::Stream, normalizedPath);

        if (cached.IsValid())
        {
            return cached;
        }

        auto stream = std::make_unique<AudioStream>();

        if (!stream->Open(normalizedPath, m_Settings.OutputFormat, m_Settings.StreamBufferFrames, m_Settings.StreamDecodeChunkFrames, m_Settings.InitialBufferedFrames))
        {
            return {};
        }

        const AudioAssetHandle handle = AllocateRecord(AudioAssetType::Stream, normalizedPath);

        AudioAssetRecord* record = Resolve(handle);

        if (!record)
        {
            stream->Close();

            return {};
        }

        record->SetStream(std::move(stream));

        m_PathCache[BuildCacheKey(AudioAssetType::Stream, normalizedPath)] = handle;

        return handle;
    }

    AudioAssetRecord* AudioResourceManager::Resolve(AudioAssetHandle handle)
    {
        if (!handle.IsValid())
        {
            return nullptr;
        }

        const std::size_t slotIndex = static_cast<std::size_t>(handle.ID - 1);

        if (slotIndex >= m_Assets.size() || slotIndex >= m_Generations.size())
        {
            return nullptr;
        }

        if (m_Generations[slotIndex] != handle.Generation)
        {
            return nullptr;
        }

        return m_Assets[slotIndex].get();
    }

    const AudioAssetRecord* AudioResourceManager::Resolve(AudioAssetHandle handle) const
    {
        if (!handle.IsValid())
        {
            return nullptr;
        }

        const std::size_t slotIndex = static_cast<std::size_t>(handle.ID - 1);

        if (slotIndex >= m_Assets.size() || slotIndex >= m_Generations.size())
        {
            return nullptr;
        }

        if (m_Generations[slotIndex] != handle.Generation)
        {
            return nullptr;
        }

        return m_Assets[slotIndex].get();
    }

    AudioClip* AudioResourceManager::GetClip(AudioAssetHandle handle)
    {
        AudioAssetRecord* asset = Resolve(handle);

        if (!asset || asset->GetType() != AudioAssetType::Clip)
        {
            return nullptr;
        }

        return asset->GetClip();
    }

    const AudioClip* AudioResourceManager::GetClip(AudioAssetHandle handle) const
    {
        const AudioAssetRecord* asset = Resolve(handle);

        if (!asset || asset->GetType() != AudioAssetType::Clip)
        {
            return nullptr;
        }

        return asset->GetClip();
    }

    AudioStream* AudioResourceManager::GetStream(AudioAssetHandle handle)
    {
        AudioAssetRecord* asset = Resolve(handle);

        if (!asset || asset->GetType() != AudioAssetType::Stream)
        {
            return nullptr;
        }

        return asset->GetStream();
    }

    const AudioStream* AudioResourceManager::GetStream(AudioAssetHandle handle) const
    {
        const AudioAssetRecord* asset = Resolve(handle);

        if (!asset || asset->GetType() != AudioAssetType::Stream)
        {
            return nullptr;
        }

        return asset->GetStream();
    }

    bool AudioResourceManager::IsValid(AudioAssetHandle handle) const
    {
        return Resolve(handle) != nullptr;
    }

    bool AudioResourceManager::IsLoaded(AudioAssetHandle handle) const
    {
        const AudioAssetRecord* asset = Resolve(handle);

        return asset && !asset->IsUnloadRequested();
    }

    AudioAssetType AudioResourceManager::GetType(AudioAssetHandle handle) const
    {
        const AudioAssetRecord* asset = Resolve(handle);

        if (!asset)
        {
            return AudioAssetType::None;
        }

        return asset->GetType();
    }

    bool AudioResourceManager::Unload(AudioAssetHandle handle)
    {
        AudioAssetRecord* asset = Resolve(handle);

        if (!asset)
        {
            return false;
        }

        asset->RequestUnload();

        return true;
    }

    void AudioResourceManager::DestroyRecord(std::size_t slotIndex)
    {
        if (slotIndex >= m_Assets.size() || !m_Assets[slotIndex])
        {
            return;
        }

        AudioAssetRecord* asset = m_Assets[slotIndex].get();

        const std::string key = BuildCacheKey(asset->GetType(), asset->GetPath());

        m_PathCache.erase(key);

        AudioStream* stream = asset->GetStream();

        if (stream)
        {
            stream->Close();
        }

        m_Assets[slotIndex].reset();

        m_Generations[slotIndex] = GetNextGeneration(m_Generations[slotIndex]);
    }

    void AudioResourceManager::CollectGarbage()
    {
        if (!m_Initialized)
        {
            return;
        }

        for (std::size_t i = 0; i < m_Assets.size(); ++i)
        {
            AudioAssetRecord* asset = m_Assets[i].get();

            if (!asset)
            {
                continue;
            }

            if (!asset->IsUnloadRequested())
            {
                continue;
            }

            if (asset->GetPlaybackReferenceCount() != 0)
            {
                continue;
            }

            AudioStream* stream = asset->GetStream();

            if (stream && stream->HasConsumer())
            {
                continue;
            }

            DestroyRecord(i);
        }
    }

    std::uint32_t AudioResourceManager::GetNextGeneration(std::uint32_t generation) const
    {
        ++generation;

        if (generation == 0)
        {
            generation = 1;
        }

        return generation;
    }

    std::size_t AudioResourceManager::GetLoadedAssetCount() const
    {
        std::size_t count = 0;

        for (const auto& asset : m_Assets)
        {
            if (asset)
            {
                ++count;
            }
        }

        return count;
    }
}
