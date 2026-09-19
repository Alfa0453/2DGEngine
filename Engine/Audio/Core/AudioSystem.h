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
#include "../Debug/AudioStats.h"
#include "../Bus/AudioBusSystem.h"
#include "../Commands/AudioCommandQueue.h"
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


    class AudioSystem : public AudioRendererSource
    {
    public:

        AudioSystem() = default;

        ~AudioSystem();

        bool Initialize(const AudioSettings& settings = AudioSettings{});

        void Shutdown();

        bool IsInitialized() const;

        bool RenderAudioBlock(const float*& outSamples, std::size_t& outFrameCount) override;

        AudioPlaybackHandle Play(const AudioClip& clip);

        AudioPlaybackHandle Play(const AudioClip& clip, const AudioPlaybackSettings& settings);

        AudioPlaybackHandle Play(const AudioClip& clip, const AudioPlaybackSettings& settings, const Vector2& sourcePosition);

        AudioPlaybackHandle Play(const AudioClip& clip, const AudioPlaybackSettings& settings, const Vector2& sourcePosition, const Vector2& sourceVelocity);

        bool Stop(AudioPlaybackHandle handle);

        bool IsPlaying(AudioPlaybackHandle handle) const;

        bool SetVolume(AudioPlaybackHandle handle, float volume);

        bool SetPan(AudioPlaybackHandle handle, float pan);

        bool SetPitch(AudioPlaybackHandle handle, float pitch);

        bool SetLooping(AudioPlaybackHandle handle, bool looping);

        void StopAll();

        float GetMasterVolume() const;

        bool SetMasterVolume(float volume);

        std::size_t GetActiveVoiceCount() const;

        const AudioSettings& GetSettings() const;

        void UpdateAudio();

        const AudioStats& GetStats() const;

        bool GetPlaybackSeconds(AudioPlaybackHandle handle, float& outSeconds) const;

        bool GetPlaybackProgress(AudioPlaybackHandle handle, float& outProgress) const;

        AudioVoice* GetAudioVoiceForHandle(AudioPlaybackHandle handle);

        bool SetBusVolume(AudioBusID bus, float volume);

        bool SetBusMuted(AudioBusID bus, bool muted);

        float GetRequestedBusVolume(AudioBusID bus) const;

        bool GetRequestedBusMuted(AudioBusID bus) const;

        bool SetListenerState(const AudioListenerState& state);

        AudioListenerState GetRequestedListenerState() const;

        bool SetSourcePosition(AudioPlaybackHandle handle, const Vector2& position);

        bool SetSourceSpatialState(AudioPlaybackHandle handle, const Vector2& position, const Vector2& velocity);

    private:

        AudioVoice* FindVoice(AudioPlaybackHandle handle);

        const AudioVoice* FindVoice(AudioPlaybackHandle handle) const;

        std::size_t FindFreeVoiceSlot() const;

        AudioPlaybackHandle CreatePlaybackHandleForSlot(std::size_t slotIndex);

        void ValidateSettings();

        std::size_t HandleToSlotIndex(AudioPlaybackHandle handle) const;

        bool IsHandleKnown(AudioPlaybackHandle handle) const;

        void ProcessAudioCommands();

        void ApplyAudioCommand(const AudioCommand& command);

        AudioVoice* GetVoiceForHandle(AudioPlaybackHandle handle);

        void ReleaseVoiceSlot(std::size_t slotIndex);

        void ProcessPlaybackEvents();

        void QueuePlaybackEvent(AudioPlaybackEventType type, AudioPlaybackHandle handle);

        std::uint64_t AcquireVoiceStartSequence();

        void ApplySlotMetadata(std::size_t slotIndex, const AudioPlaybackSettings& settings);

        std::uint32_t GetNextGeneration(std::uint32_t generation) const;

        std::size_t FindVoiceStealCandidtae(const AudioPlaybackSettings& incomingSettings) const;

        AudioPlaybackHandle CreatePlaybackHandleForGeneration(std::size_t slotIndex, std::uint32_t generation) const;

    private:

        AudioSettings m_Settings;

        AudioMixer m_Mixer;

        std::vector<AudioVoice> m_Voices;

        std::unique_ptr<AudioDevice> m_Device;

        std::uint32_t m_NextPlaybackID = 1;

        std::uint32_t m_Generation = 1;

        bool m_Initialized = false;

        AudioStats m_Stats;

        std::vector<AudioVoiceSlotState> m_VoiceSlotStates;

        std::vector<std::uint32_t> m_VoiceGenerations;

        AudioCommandQueue m_CommandQueue;

        AudioPlaybackEventQueue m_PlaybackEventQueue;

        std::vector<AudioPlaybackHandle> m_FinishedVoiceScratch;

        AudioBusSystem m_BusSystem;

        float m_RequestedMasterVolume = 1.0f;

        std::atomic<std::uint64_t> m_AudioBlocksMixed{0};

        std::atomic<std::uint64_t> m_AudioFrameMixed{0};

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
    };
}