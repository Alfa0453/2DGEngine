#pragma once

#include "AudioAssetHandle.h"
#include "AudioAssetRecord.h"
#include "AudioAssetType.h"

#include "../Types/AudioSettings.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace Engine
{
    class AudioAssetRecord;
    class AudioClip;
    class AudioStream;

    class AudioResourceManager
    {
    public:

        AudioResourceManager() = default;

        ~AudioResourceManager();

        bool Initialize(const AudioSettings& settings);

        void Shutdown();

        bool IsInitialized() const;

        AudioAssetHandle LoadClip(const std::string& filePath);

        AudioAssetHandle LoadStream(const std::string& filePath);

        bool Unload(AudioAssetHandle handle);

        void CollectGarbage();

        bool IsValid(AudioAssetHandle handle) const;

        bool IsLoaded(AudioAssetHandle handle) const;

        AudioAssetType GetType(AudioAssetHandle handle) const;

        AudioAssetRecord* Resolve(AudioAssetHandle handle);

        const AudioAssetRecord* Resolve(AudioAssetHandle handle) const;

        AudioClip* GetClip(AudioAssetHandle handle);

        const AudioClip* GetClip(AudioAssetHandle handle) const;

        AudioStream* GetStream(AudioAssetHandle handle);

        const AudioStream* GetStream(AudioAssetHandle handle) const;

        std::size_t GetLoadedAssetCount() const;

    private:

        std::string NormalizePath(const std::string& filePath) const;

        std::string BuildCacheKey(AudioAssetType type, const std::string& normalizedPath) const;

        AudioAssetHandle FindCached(AudioAssetType type, const std::string& normalizedPath);

        AudioAssetHandle AllocateRecord(AudioAssetType type, const std::string& normalizedPath);

        void DestroyRecord(std::size_t slotIndex);

        std::uint32_t GetNextGeneration(std::uint32_t generation) const;

    private:

        AudioSettings m_Settings;

        std::vector<std::unique_ptr<AudioAssetRecord>> m_Assets;

        std::vector<std::uint32_t> m_Generations;

        std::unordered_map<std::string, AudioAssetHandle> m_PathCache;

        bool m_Initialized = false;

    };
}
