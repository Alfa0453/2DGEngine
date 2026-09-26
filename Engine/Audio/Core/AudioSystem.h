#pragma once

#include "../Playback/AudioPlaybackHandle.h"
#include "../Playback/AudioVoice.h"
#include "../Playback/AudioVoiceSlotState.h"
#include "../Playback/AudioPlaybackEvent.h"
#include "../Playback/AudioPlaybackEventQueue.h"
#include "../Playback/AudioVoiceSlotMetadata.h"
#include "../Spatial/AudioListenerState.h"
#include "../Types/AudioSettings.h"
#include "../Mixer/AudioMixer.h"
#include "../Mixer/AudioMixCompletion.h"
#include "../Debug/AudioStats.h"
#include "../Bus/AudioBusSystem.h"
#include "../Commands/AudioCommandQueue.h"
#include "../Assets/AudioAssetHandle.h"
#include "../Debug/AudioVoiceDebugInfo.h"
#include "../Debug/AudioBusDebugInfo.h"

#include "AudioRendererSource.h"
#include "AudioDevice.h"

#include <cstddef>
#include <vector>
#include <memory>
#include <atomic>
#include <array>

namespace Engine
{
    class AudioClip;
    class AudioDevice;
    class AudioStream;
    class AudioResourceManager;
    class AudioAssetRecord;


    class AudioSystem : public AudioRendererSource
    {
    public:

        AudioSystem() = default;

        ~AudioSystem();

        // Lifetime
        bool Initialize(const AudioSettings& settings = AudioSettings{});
        void Shutdown();
        bool IsInitialized() const;

        // Playback creation
        AudioPlaybackHandle Play(const AudioClip& clip);
        AudioPlaybackHandle Play(const AudioClip& clip, const AudioPlaybackSettings& settings);
        AudioPlaybackHandle Play(const AudioClip& clip, const AudioPlaybackSettings& settings, const Vector2& sourcePosition);
        AudioPlaybackHandle Play(const AudioClip& clip, const AudioPlaybackSettings& settings, const Vector2& sourcePosition, const Vector2& sourceVelocity);
        AudioPlaybackHandle PlayStream(AudioStream& stream, const AudioPlaybackSettings& settings);
        AudioPlaybackHandle PlayAsset(AudioAssetHandle asset);
        AudioPlaybackHandle PlayAsset(AudioAssetHandle asset, const AudioPlaybackSettings& settings);
        AudioPlaybackHandle PlayAsset(AudioAssetHandle asset, const AudioPlaybackSettings& settings, const Vector2& sourcePosition, const Vector2& sourceVelocity);

        // Playback control
        bool Stop(AudioPlaybackHandle handle);
        void StopAll();

        bool Pause(AudioPlaybackHandle handle);
        bool Resume(AudioPlaybackHandle handle);

        bool SeekSeconds(AudioPlaybackHandle handle, float seconds);

        bool FadeTo(AudioPlaybackHandle handle, float targetGain, float durationSeconds);
        bool FadeIn(AudioPlaybackHandle handle, float durationSeconds);
        bool FadeOut(AudioPlaybackHandle handle, float durationSeconds);
        bool FadeOutAndStop(AudioPlaybackHandle handle, float durationSeconds);


        // Playback parameters
        bool SetVolume(AudioPlaybackHandle handle, float volume);
        bool SetPan(AudioPlaybackHandle handle, float pan);
        bool SetPitch(AudioPlaybackHandle handle, float pitch);
        bool SetLooping(AudioPlaybackHandle handle, bool looping);


        // Spatial state
        bool SetSourcePosition(AudioPlaybackHandle handle, const Vector2& position);
        bool SetSourceSpatialState(AudioPlaybackHandle handle, const Vector2& position, const Vector2& velocity);

        bool SetListenerState(const AudioListenerState& state);


        // Bus/global control
        bool SetMasterVolume(float volume);
        bool SetBusVolume(AudioBusID bus, float volume);
        bool SetBusMuted(AudioBusID bus, bool muted);


        // Game-thread state
        bool IsPlaying(AudioPlaybackHandle handle) const;
        bool IsPaused(AudioPlaybackHandle handle) const;

        float GetRequestedBusVolume(AudioBusID bus) const;
        bool GetRequestedBusMuted(AudioBusID bus) const;

        AudioListenerState GetRequestedListenerState() const;


        // Settings / diagnostics
        const AudioSettings& GetSettings() const;

        const AudioStats& GetStats() const;


        // Game update
        void UpdateAudio();


        // Audio callback
        bool RenderAudioBlock(const float*& outSamples, std::size_t& outFrameCount) override;


        float GetMasterVolume() const;

        std::size_t GetActiveVoiceCount() const;

        void SetResourceManager(AudioResourceManager* resourceManager);

        AudioResourceManager* GetResourceManager() const;

        void GetVoiceDebugSnapshot(std::vector<AudioVoiceDebugInfo>& outVoices) const;

        void GetBusDebugSnapshot(std::array<AudioBusDebugInfo, GetAudioBusCount()>& outBuses) const;

    private:

        AudioVoice* FindVoice(AudioPlaybackHandle handle);

        const AudioVoice* FindVoice(AudioPlaybackHandle handle) const;

        AudioVoice* GetVoiceForHandle(AudioPlaybackHandle handle);

        std::size_t FindFreeVoiceSlot() const;

        AudioPlaybackHandle CreatePlaybackHandleForSlot(std::size_t slotIndex);

        void ValidateSettings();

        std::size_t HandleToSlotIndex(AudioPlaybackHandle handle) const;

        bool IsHandleKnown(AudioPlaybackHandle handle) const;

        void ProcessAudioCommands();

        void ApplyAudioCommand(const AudioCommand& command);

        void ReleaseVoiceSlot(std::size_t slotIndex);

        void ProcessPlaybackEvents();

        void QueuePlaybackEvent(AudioPlaybackEventType type, AudioPlaybackHandle handle);

        std::uint64_t AcquireVoiceStartSequence();

        void ApplySlotMetadata(std::size_t slotIndex, const AudioPlaybackSettings& settings);

        std::uint32_t GetNextGeneration(std::uint32_t generation) const;

        std::size_t FindVoiceStealCandidate(const AudioPlaybackSettings& incomingSettings) const;

        AudioPlaybackHandle CreatePlaybackHandleForGeneration(std::size_t slotIndex, std::uint32_t generation) const;

        AudioPlaybackHandle PlayInternal(AudioSourceKind sourceKind, const AudioClip* clip, AudioStream* stream, AudioAssetRecord* assetRecord, const AudioPlaybackSettings& settings, const Vector2& sourcePosition, const Vector2& sourceVelocity);

        void ReleaseCommandPlaybackResources(const AudioCommand& command);

        void RecordRenderDuration(std::uint64_t nanoseconds);

        bool PushCommand(const AudioCommand& command);

    private:

        AudioResourceManager* m_ResourceManager = nullptr;

        AudioSettings m_Settings;

        AudioMixer m_Mixer;

        std::vector<AudioVoice> m_Voices;

        std::unique_ptr<AudioDevice> m_Device;

        bool m_Initialized = false;

        AudioStats m_Stats;

        std::vector<AudioVoiceSlotState> m_VoiceSlotStates;

        std::vector<std::uint32_t> m_VoiceGenerations;

        AudioCommandQueue m_CommandQueue;

        AudioPlaybackEventQueue m_PlaybackEventQueue;

        std::vector<AudioMixCompletion> m_AudioCompletionScratch;

        AudioBusSystem m_BusSystem;

        float m_RequestedMasterVolume = 1.0f;

        std::atomic<std::uint64_t> m_AudioBlocksMixed{0};

        std::atomic<std::uint64_t> m_AudioFramesMixed{0};

        std::atomic<std::uint64_t> m_AudioCommandsProcessed{0};

        std::uint64_t m_PreviousAudioBlocksMixed = 0;

        std::uint64_t m_PreviousAudioFramesMixed = 0;

        std::uint64_t m_PreviousCommandsProcessed = 0;

        std::atomic<std::uint64_t> m_PlaybackEventQueueOverflowCount{0};

        std::uint64_t m_CommandQueueFullCount = 0;

        std::vector<AudioVoiceSlotMetadata> m_VoiceSlotMetadata;

        std::uint64_t m_NextVoiceStartSequence = 1;

        std::uint64_t m_TotalVoiceSteals = 0;

        std::uint64_t m_PlayFailuresNoVoice = 0;

        std::array<float, GetAudioBusCount()> m_RequestedBusVolumes;

        std::array<bool, GetAudioBusCount()> m_RequestedBusMuted;

        AudioListenerState m_RequestedListenerState;

        AudioListenerState m_AudioListenerState;

        std::size_t m_PeakPendingCommandsObserved = 0;

        std::size_t m_PeakPendingPlaybackEventsObserved = 0;

        std::atomic<std::uint64_t> m_LastRenderNanoseconds{0};

        std::atomic<std::uint64_t> m_MaxRenderNanoseconds{0};

        std::atomic<std::uint64_t> m_TotalRenderNanoseconds{0};

        std::atomic<std::uint64_t> m_RenderCallCount{0};

        std::atomic<std::uint64_t> m_RenderFailureCount{0};
    };
}
