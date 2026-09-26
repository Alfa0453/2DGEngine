#include "AudioSystem.h"

#include "../Assets/AudioClip.h"
#include "../Backend/SDL/SDLAudioDevice.h"
#include "../Types/AudioLimits.h"
#include "../Streaming/AudioStream.h"
#include "../Assets/AudioResourceManager.h"
#include "../Assets/AudioAssetRecord.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>
#include <chrono>
#include <array>


namespace Engine
{
    AudioSystem::~AudioSystem()
    {
        Shutdown();
    }

    bool AudioSystem::Initialize(const AudioSettings& settings)
    {
        if (m_Initialized)
        {
            return true;
        }

        m_Settings = settings;

        ValidateSettings();

        m_Voices.clear();

        m_Voices.resize(m_Settings.MaxVoices);

        m_VoiceSlotStates.assign(m_Settings.MaxVoices, AudioVoiceSlotState::Free);

        m_VoiceSlotMetadata.assign(m_Settings.MaxVoices, AudioVoiceSlotMetadata{});

        m_NextVoiceStartSequence = 1;

        m_VoiceGenerations.assign(m_Settings.MaxVoices, 1);

        if (!m_CommandQueue.Initialize(m_Settings.MaxPendingCommands))
        {
            return false;
        }

        const std::size_t playbackEventCapacity = (m_Settings.MaxVoices * 2) + 16;

        if (!m_PlaybackEventQueue.Initialize(playbackEventCapacity))
        {
            m_CommandQueue.Shutdown();

            return false;
        }

        m_AudioCompletionScratch.clear();

        m_AudioCompletionScratch.reserve(m_Settings.MaxVoices);

        m_Mixer.Initialize(m_Settings.OutputFormat, m_Settings.MixFramesPerBlock);

        if (!m_Mixer.IsInitialized())
        {
            m_PlaybackEventQueue.Shutdown();

            m_CommandQueue.Shutdown();

            m_Voices.clear();

            m_VoiceSlotStates.clear();

            m_VoiceSlotMetadata.clear();

            m_VoiceGenerations.clear();

            return false;
        }

        m_TotalVoiceSteals = 0;

        m_PlayFailuresNoVoice = 0;

        m_BusSystem.Reset();

        m_BusSystem.SetVolumeImmediate(AudioBusID::Master, m_Settings.MasterVolume);

        m_RequestedMasterVolume = m_Settings.MasterVolume;

        m_RequestedBusVolumes.fill(1.0f);

        m_RequestedBusMuted.fill(false);

        m_RequestedBusVolumes[ToAudioBusIndex(AudioBusID::Master)] = m_Settings.MasterVolume;

        m_AudioBlocksMixed.store(0, std::memory_order_relaxed);

        m_AudioFramesMixed.store(0, std::memory_order_relaxed);

        m_AudioCommandsProcessed.store(0, std::memory_order_relaxed);

        m_LastRenderNanoseconds.store(0, std::memory_order_relaxed);

        m_MaxRenderNanoseconds.store(0, std::memory_order_relaxed);

        m_TotalRenderNanoseconds.store(0, std::memory_order_relaxed);

        m_RenderCallCount.store(0, std::memory_order_relaxed);

        m_RenderFailureCount.store(0, std::memory_order_relaxed);

        m_PreviousAudioFramesMixed = 0;

        m_PreviousAudioBlocksMixed = 0;

        m_PreviousCommandsProcessed = 0;

        m_PeakPendingCommandsObserved = 0;

        m_PeakPendingPlaybackEventsObserved = 0;

        m_Stats = AudioStats{};

        m_RequestedListenerState = AudioListenerState{};

        m_AudioListenerState = AudioListenerState{};

        m_Device = std::make_unique<SDLAudioDevice>();

        if (!m_Device->Initialize(m_Settings.OutputFormat, this))
        {
            m_Device.reset();

            m_Mixer.Shutdown();

            m_PlaybackEventQueue.Shutdown();

            m_CommandQueue.Shutdown();

            m_AudioCompletionScratch.clear();

            m_Voices.clear();

            m_VoiceSlotStates.clear();

            m_VoiceSlotMetadata.clear();

            m_VoiceGenerations.clear();

            m_Initialized = false;

            return false;
        }

        m_Initialized = true;

        return true;
    }

    void AudioSystem::Shutdown()
    {
        if (!m_Initialized)
        {
            return;
        }

        if (m_Device)
        {
            m_Device->Shutdown();

            m_Device.reset();
        }

        ProcessAudioCommands();

        for (AudioVoice& voice : m_Voices)
        {
            if (voice.IsActive())
            {
                voice.Stop();
            }
        }

        m_CommandQueue.Shutdown();

        m_PlaybackEventQueue.Shutdown();

        m_AudioCompletionScratch.clear();

        m_Voices.clear();

        m_VoiceSlotMetadata.clear();

        m_VoiceSlotStates.clear();

        m_VoiceGenerations.clear();

        m_Mixer.Shutdown();

        m_NextVoiceStartSequence = 1;

        m_ResourceManager = nullptr;

        m_RequestedMasterVolume = 1.0f;

        m_RequestedBusVolumes.fill(1.0f);

        m_RequestedBusMuted.fill(false);

        m_RequestedListenerState = {};

        m_AudioListenerState = {};

        m_Stats = {};

        m_Initialized = false;
    }

    bool AudioSystem::IsInitialized() const
    {
        return m_Initialized;
    }

    bool AudioSystem::RenderAudioBlock(const float*& outSamples, std::size_t& outFrameCount)
    {
        const auto renderStart = std::chrono::steady_clock::now();

        outSamples = nullptr;

        outFrameCount = 0;

        ProcessAudioCommands();

        m_BusSystem.AdvanceSmoothing(m_Mixer.GetFramesPerBlock(), m_Settings.OutputFormat.SampleRate);

        m_AudioCompletionScratch.clear();

        if (!m_Mixer.Mix(m_Voices, m_BusSystem, m_AudioListenerState, m_Settings, m_AudioCompletionScratch))
        {
            m_RenderFailureCount.fetch_add(1, std::memory_order_relaxed);

            const auto renderEnd = std::chrono::steady_clock::now();

            const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(renderEnd - renderStart);

            RecordRenderDuration(static_cast<std::uint64_t>(duration.count()));

            return false;
        }

        for (const AudioMixCompletion& completion : m_AudioCompletionScratch)
        {
            switch (completion.Type)
            {
                case AudioMixCompletionType::Finished:
                {
                    QueuePlaybackEvent(AudioPlaybackEventType::Finished, completion.Handle);

                    break;
                }

                case AudioMixCompletionType::Stopped:
                {
                    QueuePlaybackEvent(AudioPlaybackEventType::Stopped, completion.Handle);

                    break;
                }
            }
        }

        const std::vector<float>& buffer = m_Mixer.GetMixBuffer();

        if (buffer.empty())
        {
            return false;
        }

        outSamples = buffer.data();

        outFrameCount = m_Mixer.GetFramesPerBlock();

        m_AudioBlocksMixed.fetch_add(1, std::memory_order_relaxed);

        m_AudioFramesMixed.fetch_add(static_cast<std::uint64_t>(m_Mixer.GetFramesPerBlock()), std::memory_order_relaxed);

        const auto renderEnd = std::chrono::steady_clock::now();

        const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(renderEnd - renderStart);

        RecordRenderDuration(static_cast<std::uint64_t>(duration.count()));

        return true;
    }

    std::size_t AudioSystem::FindFreeVoiceSlot() const
    {
        for (std::size_t i = 0; i < m_VoiceSlotStates.size(); ++i)
        {
            if (m_VoiceSlotStates[i] == AudioVoiceSlotState::Free)
            {
                return i;
            }
        }

        return m_VoiceSlotStates.size();
    }

    AudioPlaybackHandle AudioSystem::CreatePlaybackHandleForSlot(std::size_t slotIndex)
    {
        AudioPlaybackHandle handle;

        handle.ID = static_cast<std::uint32_t>(slotIndex + 1);

        handle.Generation = m_VoiceGenerations[slotIndex];

        return handle;
    }

    AudioPlaybackHandle AudioSystem::Play(const AudioClip& clip)
    {
        return Play(clip, AudioPlaybackSettings{});
    }

    AudioPlaybackHandle AudioSystem::Play(const AudioClip& clip, const AudioPlaybackSettings& settings)
    {
        return Play(clip, settings, Vector2{0.0f, 0.0f});
    }

    AudioPlaybackHandle AudioSystem::Play(const AudioClip& clip, const AudioPlaybackSettings& settings, const Vector2& sourcePosition)
    {
        return Play(clip, settings, sourcePosition, Vector2{0.0f, 0.0f});
    }

    AudioPlaybackHandle AudioSystem::Play(const AudioClip& clip, const AudioPlaybackSettings& settings, const Vector2& sourcePosition, const Vector2& sourceVelocity)
    {
        return PlayInternal(AudioSourceKind::Clip, &clip, nullptr, nullptr, settings, sourcePosition, sourceVelocity);
    }

    AudioPlaybackHandle AudioSystem::PlayStream(AudioStream& stream, const AudioPlaybackSettings& settings)
    {
        return PlayInternal(AudioSourceKind::Stream, nullptr, &stream, nullptr, settings, Vector2{}, Vector2{});
    }

    AudioPlaybackHandle AudioSystem::PlayAsset(AudioAssetHandle asset)
    {
        return PlayAsset(asset, AudioPlaybackSettings{});
    }

    AudioPlaybackHandle AudioSystem::PlayAsset(AudioAssetHandle asset, const AudioPlaybackSettings& settings)
    {
        return PlayAsset(asset, settings, Vector2{}, Vector2{});
    }

    AudioPlaybackHandle AudioSystem::PlayAsset(AudioAssetHandle asset, const AudioPlaybackSettings& settings, const Vector2& sourcePosition, const Vector2& sourceVelocity)
    {
        if (!m_Initialized || !m_ResourceManager)
        {
            return {};
        }

        AudioAssetRecord* record = m_ResourceManager->Resolve(asset);

        if (!record || record->IsUnloadRequested())
        {
            return {};
        }

        // Protect asset immediately, including the time spent waiting in the SPSC command queue,
        record->RetainPlaybackReference();

        switch (record->GetType())
        {
            case AudioAssetType::Clip:
            {
                return PlayInternal(AudioSourceKind::Clip, record->GetClip(), nullptr, record, settings, sourcePosition, sourceVelocity);
            }

            case AudioAssetType::Stream:
            {
                return PlayInternal(AudioSourceKind::Stream, nullptr, record->GetStream(), record, settings, sourcePosition, sourceVelocity);
            }

            case AudioAssetType::None:
            default:
            {
                record->ReleasePlaybackReference();

                return {};
            }
        }
    }

    AudioPlaybackHandle AudioSystem::PlayInternal(AudioSourceKind sourceKind, const AudioClip *clip, AudioStream *stream, AudioAssetRecord *assetRecord, const AudioPlaybackSettings &settings, const Vector2 &sourcePosition, const Vector2 &sourceVelocity)
    {
        bool streamConsumerAcquired = false;

        auto rollback =
            [&]()
            {
                if (streamConsumerAcquired && stream)
                {
                    stream->ReleaseConsumer();
                }

                if (assetRecord)
                {
                    assetRecord->ReleasePlaybackReference();
                }
            };

        if (!m_Initialized)
        {
            rollback();

            return {};
        }

        if (sourceKind == AudioSourceKind::Clip)
        {
            if (!clip || !clip->IsValid())
            {
                rollback();

                return {};
            }
        }
        else if (sourceKind == AudioSourceKind::Stream)
        {
            if (!stream || !stream->IsOpen())
            {
                rollback();

                return {};
            }

            if (stream->GetFormat() != m_Settings.OutputFormat)
            {
                rollback();

                return {};
            }

            if (!stream->TryAcquireConsumer())
            {
                rollback();

                return {};
            }

            streamConsumerAcquired = true;
        }
        else
        {
            rollback();

            return {};
        }

        AudioPlaybackSettings sanitized = SanitizeAudioPlaybackSettings(settings);

        if (sourceKind == AudioSourceKind::Stream)
        {
            sanitized.Spatial = false;

            sanitized.DopplerEnabled = false;

            sanitized.Pitch = 1.0f;

            stream->SetLooping(sanitized.Looping);
        }

        const std::size_t freeSlot = FindFreeVoiceSlot();

        if (freeSlot < m_VoiceSlotStates.size())
        {
            const AudioPlaybackHandle handle = CreatePlaybackHandleForSlot(freeSlot);

            AudioCommand command;

            command.Type = AudioCommandType::Play;

            command.SourceKind = sourceKind;

            command.Handle = handle;

            command.Clip = clip;

            command.Stream = stream;

            command.AssetRecord = assetRecord;

            command.PlaybackSettings = sanitized;

            command.SourcePosition = sourcePosition;

            command.SourceVelocity = sourceVelocity;

            if (!PushCommand(command))
            {
                rollback();

                return {};
            }

            // Ownership has transferred to queued command / eventual AudioVoice.
            streamConsumerAcquired = false;

            m_VoiceSlotStates[freeSlot] = AudioVoiceSlotState::PendingStart;

            ApplySlotMetadata(freeSlot, sanitized);

            return handle;
        }

        if (!sanitized.AllowVoiceSteal)
        {
            ++m_PlayFailuresNoVoice;

            rollback();

            return {};
        }

        const std::size_t stealSlot = FindVoiceStealCandidate(sanitized);

        if (stealSlot >= m_VoiceSlotStates.size())
        {
            ++m_PlayFailuresNoVoice;

            rollback();

            return {};
        }

        const AudioPlaybackHandle previousHandle = CreatePlaybackHandleForSlot(stealSlot);

        const std::uint32_t newGeneration = GetNextGeneration(m_VoiceGenerations[stealSlot]);

        const AudioPlaybackHandle newHandle = CreatePlaybackHandleForGeneration(stealSlot, newGeneration);

        AudioCommand command;


        command.Type = AudioCommandType::ReplaceVoice;

        command.SourceKind = sourceKind;

        command.Handle = newHandle;

        command.PreviousHandle = previousHandle;

        command.Clip = clip;

        command.Stream = stream;

        command.AssetRecord = assetRecord;

        command.PlaybackSettings = sanitized;

        command.SourcePosition = sourcePosition;

        command.SourceVelocity = sourceVelocity;

        if (!PushCommand(command))
        {
            rollback();

            return {};
        }

        streamConsumerAcquired = false;

        m_VoiceGenerations[stealSlot] = newGeneration;

        m_VoiceSlotStates[stealSlot] = AudioVoiceSlotState::PendingStart;

        ApplySlotMetadata(stealSlot, sanitized);

        ++m_TotalVoiceSteals;

        return newHandle;
    }

    AudioVoice* AudioSystem::FindVoice(AudioPlaybackHandle handle)
    {
        if (!handle.IsValid())
        {
            return nullptr;
        }

        for (AudioVoice& voice : m_Voices)
        {
            if (voice.IsActive() && voice.GetHandle() == handle)
            {
                return &voice;
            }
        }

        return nullptr;
    }

    const AudioVoice* AudioSystem::FindVoice(AudioPlaybackHandle handle) const
    {
        if (!handle.IsValid())
        {
            return nullptr;
        }

        for (const AudioVoice& voice : m_Voices)
        {
            if (voice.IsActive() && voice.GetHandle() == handle)
            {
                return &voice;
            }
        }

        return nullptr;
    }

    bool AudioSystem::Stop(AudioPlaybackHandle handle)
    {
        if (!IsHandleKnown(handle))
        {
            return false;
        }

        const std::size_t slotIndex = static_cast<std::size_t>(handle.ID - 1);

        if (slotIndex >= m_VoiceSlotStates.size())
        {
            return false;
        }

        const AudioVoiceSlotState previousState = m_VoiceSlotStates[slotIndex];

        if (previousState == AudioVoiceSlotState::Free || previousState == AudioVoiceSlotState::PendingStop)
        {
            return false;
        }

        m_VoiceSlotStates[slotIndex] = AudioVoiceSlotState::PendingStop;

        AudioCommand command;

        command.Type = AudioCommandType::Stop;

        command.Handle = handle;

        if (!PushCommand(command))
        {
            m_VoiceSlotStates[slotIndex] = previousState;

            return false;
        }

        return true;
    }

    bool AudioSystem::IsPlaying(AudioPlaybackHandle handle) const
    {
        if (!IsHandleKnown(handle))
        {
            return false;
        }

        const std::size_t slotIndex = HandleToSlotIndex(handle);

        if (slotIndex >= m_VoiceSlotStates.size())
        {
            return false;
        }

        const AudioVoiceSlotState state = m_VoiceSlotStates[slotIndex];

        return state == AudioVoiceSlotState::PendingStart || state == AudioVoiceSlotState::Active;
    }

    bool AudioSystem::SetVolume(AudioPlaybackHandle handle, float volume)
    {
        if (!IsHandleKnown(handle))
        {
            return false;
        }

        const std::size_t slotIndex = HandleToSlotIndex(handle);

        if (slotIndex >= m_VoiceSlotStates.size())
        {
            return false;
        }

        if (m_VoiceSlotStates[slotIndex] == AudioVoiceSlotState::Free)
        {
            return false;
        }

        AudioCommand command;

        command.Type = AudioCommandType::SetVolume;

        command.Handle = handle;

        command.Value = std::clamp(volume, 0.0f, 1.0f);

        if (!PushCommand(command))
        {
            return false;
        }

        return true;
    }

    bool AudioSystem::SetPan(AudioPlaybackHandle handle, float pan)
    {
        if (!IsHandleKnown(handle))
        {
            return false;
        }

        const std::size_t slotIndex = HandleToSlotIndex(handle);

        if (slotIndex >= m_VoiceSlotStates.size())
        {
            return false;
        }

        if (m_VoiceSlotStates[slotIndex] == AudioVoiceSlotState::Free)
        {
            return false;
        }

        AudioCommand command;

        command.Type = AudioCommandType::SetPan;

        command.Handle = handle;

        command.PlaybackSettings.Pan = std::clamp(pan, -1.0f, 1.0f);

        if (!PushCommand(command))
        {
            return false;
        }

        return true;
    }

    bool AudioSystem::SetPitch(AudioPlaybackHandle handle, float pitch)
    {
        if (!IsHandleKnown(handle))
        {
            return false;
        }

        const std::size_t slotIndex = HandleToSlotIndex(handle);

        if (slotIndex >= m_VoiceSlotStates.size())
        {
            return false;
        }

        if (m_VoiceSlotStates[slotIndex] == AudioVoiceSlotState::Free)
        {
            return false;
        }

        AudioCommand command;

        command.Type = AudioCommandType::SetPitch;

        command.Handle = handle;

        command.PlaybackSettings.Pitch = std::clamp(pitch, 0.25f, 4.0f);

        if (!PushCommand(command))
        {
            return false;
        }

        return true;
    }

    bool AudioSystem::SetLooping(AudioPlaybackHandle handle, bool looping)
    {
        if (!IsHandleKnown(handle))
        {
            return false;
        }

        const std::size_t slotIndex = HandleToSlotIndex(handle);

        if (slotIndex >= m_VoiceSlotStates.size())
        {
            return false;
        }

        if (m_VoiceSlotStates[slotIndex] == AudioVoiceSlotState::Free)
        {
            return false;
        }

        AudioCommand command;

        command.Type = AudioCommandType::SetLooping;

        command.Handle = handle;

        command.BoolValue = looping;

        if (!PushCommand(command))
        {
            return false;
        }

        return true;
    }

    void AudioSystem::StopAll()
    {
        if (!m_Initialized)
        {
            return;
        }

        AudioCommand command;

        command.Type = AudioCommandType::StopAll;

        if (!PushCommand(command))
        {
            return;
        }

        for (AudioVoiceSlotState& state : m_VoiceSlotStates)
        {
            if (state != AudioVoiceSlotState::Free)
            {
                state = AudioVoiceSlotState::PendingStop;
            }
        }
    }

    float AudioSystem::GetMasterVolume() const
    {
        return GetRequestedBusVolume(AudioBusID::Master);
    }

    bool AudioSystem::SetMasterVolume(float volume)
    {
        return SetBusVolume(AudioBusID::Master, volume);
    }

    std::size_t AudioSystem::GetActiveVoiceCount() const
    {
        std::size_t count = 0;

        for (const AudioVoiceSlotState state : m_VoiceSlotStates)
        {
            if (state != AudioVoiceSlotState::Free)
            {
                ++count;
            }
        }

        return count;
    }

    const AudioSettings& AudioSystem::GetSettings() const
    {
        return m_Settings;
    }

    void AudioSystem::ValidateSettings()
    {
        if (m_Settings.OutputFormat.SampleRate == 0)
        {
            m_Settings.OutputFormat.SampleRate = 48000;
        }

        if (m_Settings.OutputFormat.Channels == 0)
        {
            m_Settings.OutputFormat.Channels = 2;
        }

        m_Settings.MaxVoices = std::max<std::size_t>(1, m_Settings.MaxVoices);

        m_Settings.MixFramesPerBlock = std::max<std::size_t>(64, m_Settings.MixFramesPerBlock);

        m_Settings.MaxPendingCommands = std::max<std::size_t>(64, m_Settings.MaxPendingCommands);

        m_Settings.MasterVolume = std::clamp(m_Settings.MasterVolume, 0.0f, 1.0f);

        m_Settings.SpeedOfSound = std::max(m_Settings.SpeedOfSound, 1.0f);

        m_Settings.MinDopplerFactor = std::max(m_Settings.MinDopplerFactor, 0.01f);

        m_Settings.MaxDopplerFactor = std::max(m_Settings.MaxDopplerFactor, m_Settings.MinDopplerFactor);

        m_Settings.StreamBufferFrames = std::max<std::size_t>(1024, m_Settings.StreamBufferFrames);

        m_Settings.StreamDecodeChunkFrames = std::max<std::size_t>(64, m_Settings.StreamDecodeChunkFrames);

        m_Settings.StreamInitialBufferedFrames = std::min(std::max<std::size_t>(1, m_Settings.StreamInitialBufferedFrames), m_Settings.StreamBufferFrames);
    }

    void AudioSystem::UpdateAudio()
    {
        if (!m_Initialized)
        {
            return;
        }

        // Consume audio-thread playback events.

        ProcessPlaybackEvents();

        // Atomic cumulative counters.

        const std::uint64_t totalBlocks = m_AudioBlocksMixed.load(std::memory_order_relaxed);

        const std::uint64_t totalFrames = m_AudioFramesMixed.load(std::memory_order_relaxed);

        const std::uint64_t totalCommands = m_AudioCommandsProcessed.load(std::memory_order_relaxed);

        // Pre-game-update deltas

        m_Stats.BlocksMixedThisUpdate = static_cast<std::size_t>(totalBlocks - m_PreviousAudioBlocksMixed);

        m_Stats.FramesMixedThisUpdate = static_cast<std::size_t>(totalFrames - m_PreviousAudioFramesMixed);

        m_Stats.CommandsProcessedThisUpdate = static_cast<std::size_t>(totalCommands - m_PreviousCommandsProcessed);

        m_PreviousAudioBlocksMixed = totalBlocks;

        m_PreviousAudioFramesMixed = totalFrames;

        m_PreviousCommandsProcessed = totalCommands;

        // Totals

        m_Stats.TotalBlocksMixed = totalBlocks;

        m_Stats.TotalFramesMixed = totalFrames;

        m_Stats.TotalCommandsProcessed = totalCommands;

        m_Stats.FinishedVoicesThisUpdate = 0;

        // Voice states

        m_Stats.ActiveVoices = 0;

        m_Stats.PendingStartVoices = 0;

        m_Stats.PendingStopVoices = 0;

        m_Stats.PausedVoices = 0;

        m_Stats.ActiveMusicVoices = 0;

        m_Stats.ActiveSFXVoices = 0;

        m_Stats.ActiveUIVoices = 0;

        m_Stats.ActiveAmbientVoices = 0;

        m_Stats.FailedStartsThisUpdate = 0;

        for (std::size_t i = 0; i < m_VoiceSlotStates.size(); ++i)
        {
            const AudioVoiceSlotState state = m_VoiceSlotStates[i];

            if (state == AudioVoiceSlotState::Free)
            {
                continue;
            }

            ++m_Stats.ActiveVoices;

            if (state == AudioVoiceSlotState::PendingStart)
            {
                ++m_Stats.PendingStartVoices;
            }
            else if (state == AudioVoiceSlotState::PendingStop)
            {
                ++m_Stats.PendingStopVoices;
            }

            if (i < m_VoiceSlotMetadata.size() && m_VoiceSlotMetadata[i].Paused)
            {
                ++m_Stats.PausedVoices;
            }

            if (i >= m_VoiceSlotMetadata.size())
            {
                continue;
            }

            switch (m_VoiceSlotMetadata[i].Bus)
            {
                case AudioBusID::Music:
                {
                    ++m_Stats.ActiveMusicVoices;

                    break;
                }

                case AudioBusID::SFX:
                {
                    ++m_Stats.ActiveSFXVoices;

                    break;
                }

                case AudioBusID::UI:
                {
                    ++m_Stats.ActiveUIVoices;

                    break;
                }

                case AudioBusID::Ambient:
                {
                    ++m_Stats.ActiveAmbientVoices;

                    break;
                }

                default:
                {
                    break;
                }
            }

            // Queue pressure

            m_Stats.PendingCommands = m_CommandQueue.GetPendingCount();

            m_Stats.CommandQueueCapacity = m_CommandQueue.GetCapacity();

            m_Stats.PeakPendingCommandsObserved = m_PeakPendingCommandsObserved;

            m_Stats.PendingPlaybackEvents = m_PlaybackEventQueue.GetPendingCount();

            m_Stats.PlaybackEventQueueCapacity = m_PlaybackEventQueue.GetCapacity();

            m_PeakPendingPlaybackEventsObserved = std::max(m_PeakPendingPlaybackEventsObserved, m_Stats.PendingPlaybackEvents);

            m_Stats.PeakPendingPlaybackEventsObserved = m_PeakPendingPlaybackEventsObserved;

            // Failure / allocator statistics

            m_Stats.CommandQueueFullCount = m_CommandQueueFullCount;

            m_Stats.PlaybackEventQueueOverflowCount = m_PlaybackEventQueueOverflowCount.load(std::memory_order_relaxed);

            m_Stats.TotalVoiceSteals = m_TotalVoiceSteals;

            m_Stats.PlayFailuresNoVoice = m_PlayFailuresNoVoice;

            // Streaming

            m_Stats.StreamUnderflows = m_Mixer.GetStreamUnderflowCount();

            // Audio render profiling

            const std::uint64_t lastNanoseconds = m_LastRenderNanoseconds.load(std::memory_order_relaxed);

            const std::uint64_t maxNanoseconds = m_MaxRenderNanoseconds.load(std::memory_order_relaxed);

            const std::uint64_t totalNanoseconds = m_TotalRenderNanoseconds.load(std::memory_order_relaxed);

            const std::uint64_t renderCalls = m_RenderCallCount.load(std::memory_order_relaxed);


            m_Stats.LastRenderMilliseconds = static_cast<double>(lastNanoseconds) / 1'000'000.0;

            m_Stats.MaximumRenderMilliseconds = static_cast<double>(maxNanoseconds) / 1'000'000.0;

            m_Stats.RenderCallCount = renderCalls;

            m_Stats.RenderFailureCount = m_RenderFailureCount.load(std::memory_order_relaxed);

            if (renderCalls > 0)
            {
                const double averageNanoseconds = static_cast<double>(totalNanoseconds) / static_cast<double>(renderCalls);

                m_Stats.AverageRenderMilliseconds = averageNanoseconds / 1'000'000.0;
            }
            else
            {
                m_Stats.AverageRenderMilliseconds = 0.0;
            }

            if (m_Settings.OutputFormat.SampleRate > 0)
            {
                m_Stats.RenderBudgetMilliseconds = (static_cast<double>(m_Settings.MixFramesPerBlock) / static_cast<double>(m_Settings.OutputFormat.SampleRate)) * 1000.0;
            }
            else
            {
                m_Stats.RenderBudgetMilliseconds = 0.0;
            }

            if (m_Stats.RenderBudgetMilliseconds > 0.0)
            {
                m_Stats.LastRenderBudgetUsagePercent = (m_Stats.LastRenderMilliseconds / m_Stats.RenderBudgetMilliseconds) * 100.0;

                m_Stats.MaximumRenderBudgetUsagePercent = (m_Stats.MaximumRenderMilliseconds / m_Stats.RenderBudgetMilliseconds) * 100.0;
            }
            else
            {
                m_Stats.LastRenderBudgetUsagePercent = 0.0;

                m_Stats.MaximumRenderBudgetUsagePercent = 0.0;
            }
        }
    }

    const AudioStats& AudioSystem::GetStats() const
    {
        return m_Stats;
    }


    bool AudioSystem::SetBusVolume(AudioBusID bus, float volume)
    {
        const std::size_t index = ToAudioBusIndex(bus);

        if (index >= GetAudioBusCount())
        {
            return false;
        }

        const float clamped = std::clamp(volume, AudioLimits::MinVolume, AudioLimits::MaxVolume);

        AudioCommand command;

        command.Type = AudioCommandType::SetBusVolume;

        command.Bus = bus;

        command.Value = clamped;

        if (!PushCommand(command))
        {
            return false;
        }

        m_RequestedBusVolumes[index] = clamped;

        return true;
    }

    bool AudioSystem::SetBusMuted(AudioBusID bus, bool muted)
    {
        const std::size_t index = ToAudioBusIndex(bus);

        if (index >= GetAudioBusCount())
        {
            return false;
        }

        AudioCommand command;

        command.Type = AudioCommandType::SetBusMute;

        command.Bus = bus;

        command.BoolValue = muted;

        if (!PushCommand(command))
        {
            return false;
        }

        m_RequestedBusMuted[index] = muted;

        return true;
    }

    float AudioSystem::GetRequestedBusVolume(AudioBusID bus) const
    {
        const std::size_t index = ToAudioBusIndex(bus);

        if (index >= m_RequestedBusVolumes.size())
        {
            return 1.0f;
        }

        return m_RequestedBusVolumes[index];
    }

    bool AudioSystem::GetRequestedBusMuted(AudioBusID bus) const
    {
        const std::size_t index = ToAudioBusIndex(bus);

        if (index >= m_RequestedBusMuted.size())
        {
            return false;
        }

        return m_RequestedBusMuted[index];
    }

    std::size_t AudioSystem::HandleToSlotIndex(AudioPlaybackHandle handle) const
    {
        return static_cast<std::size_t>(handle.ID - 1);
    }

    bool AudioSystem::IsHandleKnown(AudioPlaybackHandle handle) const
    {
        if (!handle.IsValid())
        {
            return false;
        }

        const std::size_t slotIndex = static_cast<std::size_t>(handle.ID - 1);

        if (slotIndex >= m_VoiceGenerations.size())
        {
            return false;
        }

        return m_VoiceGenerations[slotIndex] == handle.Generation;
    }

    void AudioSystem::ProcessAudioCommands()
    {
        std::uint64_t processedCount = 0;

        AudioCommand command;

        while (m_CommandQueue.TryPop(command))
        {
            ApplyAudioCommand(command);

            ++processedCount;
        }

        if (processedCount > 0)
        {
            m_AudioCommandsProcessed.fetch_add(processedCount, std::memory_order_relaxed);
        }
    }

    void AudioSystem::ApplyAudioCommand(const AudioCommand& command)
    {
        switch (command.Type)
        {
            case AudioCommandType::Play:
            {
                const std::size_t slotIndex = static_cast<std::size_t>(command.Handle.ID -1);

                if (slotIndex >= m_Voices.size())
                {
                    ReleaseCommandPlaybackResources(command);

                    QueuePlaybackEvent(AudioPlaybackEventType::FailedToStart, command.Handle);

                    break;
                }

                AudioVoice& voice = m_Voices[slotIndex];

                if (command.SourceKind == AudioSourceKind::Clip)
                {
                    if (!command.Clip)
                    {
                        QueuePlaybackEvent(AudioPlaybackEventType::FailedToStart, command.Handle);

                        break;
                    }

                    voice.Start(command.Clip, command.Handle, command.PlaybackSettings, command.SourcePosition, command.SourceVelocity, command.AssetRecord);
                }

                else if (command.SourceKind == AudioSourceKind::Stream)
                {
                    if (!command.Stream || !command.Stream->IsOpen())
                    {
                        if (command.Stream)
                        {
                            command.Stream->ReleaseConsumer();
                        }

                        QueuePlaybackEvent(AudioPlaybackEventType::FailedToStart, command.Handle);

                        break;
                    }

                    voice.StartStream(command.Stream, command.Handle, command.PlaybackSettings, command.AssetRecord);
                }
                else
                {
                    QueuePlaybackEvent(AudioPlaybackEventType::FailedToStart, command.Handle);

                    break;
                }

                if (command.PlaybackSettings.FadeInSeconds > 0.0f)
                {
                    const std::uint64_t frames = static_cast<std::uint64_t>(command.PlaybackSettings.FadeInSeconds * static_cast<float>(m_Settings.OutputFormat.SampleRate));

                    voice.SetFadeGainImmediate(0.0f);

                    voice.StartFade(1.0f, frames, false);
                }

                if (!voice.IsActive())
                {
                    QueuePlaybackEvent(AudioPlaybackEventType::FailedToStart, command.Handle);

                    break;
                }

                QueuePlaybackEvent(AudioPlaybackEventType::Started, command.Handle);

                break;
            }

            case AudioCommandType::Stop:
            {
                AudioVoice* voice = GetVoiceForHandle(command.Handle);

                if (voice)
                {
                    voice->Stop();
                }

                QueuePlaybackEvent(AudioPlaybackEventType::Stopped, command.Handle);

                break;
            }

            case AudioCommandType::SetVolume:
            {
                AudioVoice* voice = GetVoiceForHandle(command.Handle);

                if (voice)
                {
                    voice->SetVolume(command.Value);
                }

                break;
            }

            case AudioCommandType::SetLooping:
            {
                AudioVoice* voice = GetVoiceForHandle(command.Handle);

                if (voice)
                {
                    voice->SetLooping(command.BoolValue);

                    if (voice->GetSourceKind() == AudioSourceKind::Stream)
                    {
                        AudioStream* stream = voice->GetStream();

                        if (stream)
                        {
                            stream->SetLooping(command.BoolValue);
                        }
                    }
                }

                break;
            }

            case AudioCommandType::StopAll:
            {
                for (AudioVoice& voice : m_Voices)
                {
                    if (!voice.IsActive())
                    {
                        continue;
                    }

                    const AudioPlaybackHandle handle = voice.GetHandle();

                    voice.Stop();

                    QueuePlaybackEvent(AudioPlaybackEventType::Stopped, handle);
                }

                break;
            }

            case AudioCommandType::SetBusVolume:
            {
                m_BusSystem.SetVolume(command.Bus, command.Value);

                break;
            }

            case AudioCommandType::SetBusMute:
            {
                m_BusSystem.SetMuted(command.Bus, command.BoolValue);

                break;
            }

            case AudioCommandType::SetPan:
            {
                AudioVoice* voice = GetVoiceForHandle(command.Handle);

                if (voice)
                {
                    voice->SetPan(command.Value);
                }

                break;
            }

            case AudioCommandType::SetPitch:
            {
                AudioVoice* voice = GetVoiceForHandle(command.Handle);

                if (voice)
                {
                    voice->SetPitch(command.Value);
                }

                break;
            }

            case AudioCommandType::ReplaceVoice:
            {
                if (!command.Handle.IsValid())
                {
                    break;
                }

                const std::size_t slotIndex = static_cast<std::size_t>(command.Handle.ID - 1);

                if (slotIndex >= m_Voices.size())
                {
                    ReleaseCommandPlaybackResources(command);

                    QueuePlaybackEvent(AudioPlaybackEventType::FailedToStart, command.Handle);

                    break;
                }

                AudioVoice& voice = m_Voices[slotIndex];

                // If the old Voice is still present, terminate it.
                // AudioVoice::Stop() also releases an old stream
                // consumer when appropriate.
                if (voice.IsActive())
                {
                    voice.Stop();
                }

                bool started = false;

                if (command.SourceKind == AudioSourceKind::Clip && command.Clip)
                {
                    voice.Start(command.Clip, command.Handle, command.PlaybackSettings, command.SourcePosition, command.SourceVelocity, command.AssetRecord);

                    started = true;
                }
                else if (command.SourceKind == AudioSourceKind::Stream && command.Stream && command.Stream->IsOpen())
                {
                    voice.StartStream(command.Stream, command.Handle, command.PlaybackSettings, command.AssetRecord);

                    started = true;
                }

                if (!started)
                {
                    if (command.SourceKind == AudioSourceKind::Stream && command.Stream)
                    {
                        command.Stream->ReleaseConsumer();
                    }

                    QueuePlaybackEvent(AudioPlaybackEventType::FailedToStart, command.Handle);

                    break;
                }

                if (command.PlaybackSettings.FadeInSeconds > 0.0f)
                {
                    const std::uint64_t frames = static_cast<std::uint64_t>(command.PlaybackSettings.FadeInSeconds * static_cast<float>(m_Settings.OutputFormat.SampleRate));

                    voice.SetFadeGainImmediate(0.0f);

                    voice.StartFade(1.0f, frames, false);
                }

                QueuePlaybackEvent(AudioPlaybackEventType::Started, command.Handle);

                break;
            }

            case AudioCommandType::SetListenerState:
            {
                m_AudioListenerState = command.ListenerState;

                break;
            }

            case AudioCommandType::SetSourcePosition:
            {
                AudioVoice* voice = GetVoiceForHandle(command.Handle);

                if (voice)
                {
                    voice->SetSpatialPosition(command.SourcePosition);
                }

                break;
            }

            case AudioCommandType::SetSourceSpatialState:
            {
                AudioVoice* voice = GetVoiceForHandle(command.Handle);

                if (voice)
                {
                    voice->SetSpatialPosition(command.SourcePosition);

                    voice->SetSpatialVelocity(command.SourceVelocity);
                }

                break;
            }

            case AudioCommandType::Pause:
            {
                AudioVoice* voice = GetVoiceForHandle(command.Handle);

                if (voice)
                {
                    voice->Pause();
                }

                break;
            }

            case AudioCommandType::Resume:
            {
                AudioVoice* voice = GetVoiceForHandle(command.Handle);

                if (voice)
                {
                    voice->Resume();
                }

                break;
            }

            case AudioCommandType::SeekSeconds:
            {
                AudioVoice* voice = GetVoiceForHandle(command.Handle);

                if (!voice)
                {
                    break;
                }

                switch (voice->GetSourceKind())
                {
                    case Engine::AudioSourceKind::Clip:
                    {
                        voice->SeekSeconds(command.Value);

                        break;
                    }

                    case Engine::AudioSourceKind::Stream:
                    {
                        AudioStream* stream = voice->GetStream();

                        if (stream)
                        {
                            stream->RequestSeekSeconds(command.Value);
                        }

                        break;
                    }

                    case Engine::AudioSourceKind::None:
                    default:
                    {
                        break;
                    }
                }

                break;
            }

            case AudioCommandType::FadeTo:
            {
                AudioVoice* voice = GetVoiceForHandle(command.Handle);

                if (voice)
                {
                    const std::uint64_t frames = static_cast<std::uint64_t>(command.DurationSeconds * static_cast<float>(m_Settings.OutputFormat.SampleRate));

                    voice->StartFade(command.Value, frames, false);
                }

                break;
            }

            case AudioCommandType::FadeOutAndStop:
            {
                AudioVoice* voice = GetVoiceForHandle(command.Handle);

                if (voice)
                {
                    const std::uint64_t frames = static_cast<std::uint64_t>(command.DurationSeconds * static_cast<float>(m_Settings.OutputFormat.SampleRate));

                    voice->StartFade(0.0f, frames, true);
                }

                break;
            }
        }
    }

    void AudioSystem::SetResourceManager(AudioResourceManager* resourceManager)
    {
        m_ResourceManager = resourceManager;
    }

    AudioResourceManager* AudioSystem::GetResourceManager() const
    {
        return m_ResourceManager;
    }

    AudioVoice* AudioSystem::GetVoiceForHandle(AudioPlaybackHandle handle)
    {
        if (!handle.IsValid())
        {
            return nullptr;
        }

        const std::size_t slotIndex = static_cast<std::size_t>(handle.ID - 1);

        if (slotIndex >= m_Voices.size())
        {
            return nullptr;
        }

        AudioVoice& voice = m_Voices[slotIndex];

        if (!voice.IsActive() || voice.GetHandle() != handle)
        {
            return nullptr;
        }

        return &voice;
    }

    void AudioSystem::ReleaseVoiceSlot(std::size_t slotIndex)
    {
        if (slotIndex >= m_VoiceSlotStates.size() || slotIndex >= m_VoiceGenerations.size() || slotIndex >= m_VoiceSlotMetadata.size())
        {
            return;
        }

        m_VoiceSlotStates[slotIndex] = AudioVoiceSlotState::Free;

        m_VoiceSlotMetadata[slotIndex] = AudioVoiceSlotMetadata{};

        ++m_VoiceGenerations[slotIndex];

        if (m_VoiceGenerations[slotIndex] == 0)
        {
            m_VoiceGenerations[slotIndex] = 1;
        }
    }

    void AudioSystem::ProcessPlaybackEvents()
    {
        m_Stats.StartedVoicesThisUpdate = 0;

        m_Stats.FinishedVoicesThisUpdate = 0;

        m_Stats.StoppedVoicesThisUpdate = 0;

        m_Stats.FailedStartsThisUpdate = 0;

        AudioPlaybackEvent event;

        while (m_PlaybackEventQueue.TryPop(event))
        {
            if (!IsHandleKnown(event.Handle))
            {
                continue;
            }

            const std::size_t slotIndex = HandleToSlotIndex(event.Handle);

            switch (event.Type)
            {
                case AudioPlaybackEventType::Started:
                {
                    if (m_VoiceSlotStates[slotIndex] == AudioVoiceSlotState::PendingStart)
                    {
                        m_VoiceSlotStates[slotIndex] = AudioVoiceSlotState::Active;

                        ++m_Stats.StartedVoicesThisUpdate;
                    }

                    break;
                }

                case AudioPlaybackEventType::Stopped:
                {
                    ++m_Stats.StoppedVoicesThisUpdate;

                    ReleaseVoiceSlot(slotIndex);

                    break;
                }

                case AudioPlaybackEventType::Finished:
                {
                    ++m_Stats.FinishedVoicesThisUpdate;

                    ReleaseVoiceSlot(slotIndex);

                    break;
                }

                case AudioPlaybackEventType::FailedToStart:
                {
                    ++m_Stats.FailedStartsThisUpdate;

                    ReleaseVoiceSlot(slotIndex);

                    break;
                }
            }
        }
    }

    void AudioSystem::QueuePlaybackEvent(AudioPlaybackEventType type, AudioPlaybackHandle handle)
    {
        AudioPlaybackEvent event;

        event.Type = type;

        event.Handle = handle;

        if (!m_PlaybackEventQueue.Push(event))
        {
            m_PlaybackEventQueueOverflowCount.fetch_add(1, std::memory_order_relaxed);
        }
    }

    std::uint64_t AudioSystem::AcquireVoiceStartSequence()
    {
        const std::uint64_t sequence = m_NextVoiceStartSequence;

        ++m_NextVoiceStartSequence;

        if (m_NextVoiceStartSequence == 0)
        {
            m_NextVoiceStartSequence = 1;
        }

        return sequence;
    }

    void AudioSystem::ApplySlotMetadata(std::size_t slotIndex, const AudioPlaybackSettings& settings)
    {
        if (slotIndex >= m_VoiceSlotMetadata.size())
        {
            return;
        }

        AudioVoiceSlotMetadata& metadata = m_VoiceSlotMetadata[slotIndex];

        metadata.Priority = settings.Priority;

        metadata.Bus = settings.Bus;

        metadata.Stealable = settings.Stealable;

        metadata.Looping = settings.Looping;

        metadata.StartSequence = AcquireVoiceStartSequence();
    }

    std::uint32_t AudioSystem::GetNextGeneration(std::uint32_t generation) const
    {
        ++generation;

        if (generation == 0)
        {
            generation = 1;
        }

        return generation;
    }

    std::size_t AudioSystem::FindVoiceStealCandidate(const AudioPlaybackSettings& incomingSettings) const
    {
        const std::size_t invalidIndex = m_VoiceSlotStates.size();

        std::size_t bestIndex = invalidIndex;

        for (std::size_t i = 0; i < m_VoiceSlotStates.size(); ++i)
        {
            if (m_VoiceSlotStates[i] != AudioVoiceSlotState::Active)
            {
                continue;
            }

            const AudioVoiceSlotMetadata& candidate = m_VoiceSlotMetadata[i];

            if (!candidate.Stealable)
            {
                continue;
            }

            if (candidate.Priority > incomingSettings.Priority)
            {
                continue;
            }

            if (bestIndex == invalidIndex)
            {
                bestIndex = i;

                continue;
            }

            const AudioVoiceSlotMetadata& best = m_VoiceSlotMetadata[bestIndex];

            if (candidate.Priority < best.Priority)
            {
                bestIndex = i;

                continue;
            }

            if (candidate.Priority > best.Priority)
            {
                continue;
            }

            // Equal priority:
            // prefer stealing non-looping before looping.

            if (candidate.Looping != best.Looping)
            {
                if (!candidate.Looping)
                {
                    bestIndex = i;
                }

                continue;
            }

            // Equal priority + same loop state:
            // steal oldest.

            if (candidate.StartSequence < best.StartSequence)
            {
                bestIndex = i;
            }
        }

        return bestIndex;
    }

    AudioPlaybackHandle AudioSystem::CreatePlaybackHandleForGeneration(std::size_t slotIndex, std::uint32_t generation) const
    {
        AudioPlaybackHandle handle;

        handle.ID = static_cast<std::uint32_t>(slotIndex + 1);

        handle.Generation = generation;

        return handle;
    }

    bool AudioSystem::SetListenerState(const AudioListenerState& state)
    {
        AudioCommand command;

        command.Type = AudioCommandType::SetListenerState;

        command.ListenerState = state;

        if (!PushCommand(command))
        {
            return false;
        }

        m_RequestedListenerState = state;

        return true;
    }

    AudioListenerState AudioSystem::GetRequestedListenerState() const
    {
        return m_RequestedListenerState;
    }

    bool AudioSystem::SetSourcePosition(AudioPlaybackHandle handle, const Vector2& position)
    {
        if (!IsHandleKnown(handle))
        {
            return false;
        }

        AudioCommand command;

        command.Type = AudioCommandType::SetSourcePosition;

        command.Handle = handle;

        command.SourcePosition = position;

        if (!PushCommand(command))
        {
            return false;
        }

        return true;
    }

    bool AudioSystem::SetSourceSpatialState(AudioPlaybackHandle handle, const Vector2& position, const Vector2& velocity)
    {
        if (!IsHandleKnown(handle))
        {
            return false;
        }

        AudioCommand command;

        command.Type = AudioCommandType::SetSourceSpatialState;

        command.Handle = handle;

        command.SourcePosition = position;

        command.SourceVelocity = velocity;

        if (!PushCommand(command))
        {
            return false;
        }

        return true;
    }

    bool AudioSystem::Pause(AudioPlaybackHandle handle)
    {
        if (!IsHandleKnown(handle))
        {
            return false;
        }

        const std::size_t index = static_cast<std::size_t>(handle.ID - 1);


        if (index >= m_VoiceSlotMetadata.size())
        {
            return false;
        }

        if (m_VoiceSlotMetadata[index].Paused)
        {
            return true;
        }

        AudioCommand command;

        command.Type = AudioCommandType::Pause;

        command.Handle = handle;

        if (!PushCommand(command))
        {
            return false;
        }

        m_VoiceSlotMetadata[index].Paused = true;

        return true;
    }

    bool AudioSystem::Resume(AudioPlaybackHandle handle)
    {
        if (!IsHandleKnown(handle))
        {
            return false;
        }

        const std::size_t index = static_cast<std::size_t>(handle.ID - 1);

        if (index >= m_VoiceSlotMetadata.size())
        {
            return false;
        }

        if (!m_VoiceSlotMetadata[index].Paused)
        {
            return true;
        }

        AudioCommand command;

        command.Type = AudioCommandType::Resume;

        command.Handle = handle;

        if (!PushCommand(command))
        {
            return false;
        }

        m_VoiceSlotMetadata[index].Paused = false;

        return true;
    }

    bool AudioSystem::IsPaused(AudioPlaybackHandle handle) const
    {
        if (!IsHandleKnown(handle))
        {
            return false;
        }

        const std::size_t index = static_cast<std::size_t>(handle.ID - 1);

        return m_VoiceSlotMetadata[index].Paused;
    }

    bool AudioSystem::SeekSeconds(AudioPlaybackHandle handle, float seconds)
    {
        if (!IsHandleKnown(handle))
        {
            return false;
        }

        AudioCommand command;

        command.Type = AudioCommandType::SeekSeconds;

        command.Handle = handle;

        command.Value = std::max(seconds, 0.0f);

        if (!PushCommand(command))
        {
            return false;
        }

        return true;
    }

    bool AudioSystem::FadeTo(AudioPlaybackHandle handle, float targetGain, float durationSeconds)
    {
        if (!IsHandleKnown(handle))
        {
            return false;
        }

        AudioCommand command;

        command.Type = AudioCommandType::FadeTo;

        command.Handle = handle;

        command.Value = std::clamp(targetGain, 0.0f, 1.0f);

        command.DurationSeconds = std::max(durationSeconds, 0.0f);

        if (!PushCommand(command))
        {
            return false;
        }

        return true;
    }

    bool AudioSystem::FadeOut(AudioPlaybackHandle handle, float durationSeconds)
    {
        return FadeTo(handle, 0.0f, durationSeconds);
    }

    bool AudioSystem::FadeIn(AudioPlaybackHandle handle, float durationSeconds)
    {
        return FadeTo(handle, 1.0f, durationSeconds);
    }

    bool AudioSystem::FadeOutAndStop(AudioPlaybackHandle handle, float durationSeconds)
    {
        if (!IsHandleKnown(handle))
        {
            return false;
        }

        AudioCommand command;

        command.Type = AudioCommandType::FadeOutAndStop;

        command.Handle = handle;

        command.Value = 0.0f;

        command.DurationSeconds = std::max(durationSeconds, 0.0f);

        if (!PushCommand(command))
        {
            return false;
        }

        const std::size_t index = static_cast<std::size_t>(handle.ID - 1);

        m_VoiceSlotStates[index] = AudioVoiceSlotState::PendingStop;

        return true;
    }

    void AudioSystem::ReleaseCommandPlaybackResources(const AudioCommand& command)
    {
        if (command.SourceKind == AudioSourceKind::Stream && command.Stream)
        {
            command.Stream->ReleaseConsumer();
        }

        if (command.AssetRecord)
        {
            command.AssetRecord->ReleasePlaybackReference();
        }
    }

    void AudioSystem::RecordRenderDuration(std::uint64_t nanoseconds)
    {
        m_LastRenderNanoseconds.store(nanoseconds, std::memory_order_relaxed);

        m_TotalRenderNanoseconds.fetch_add(nanoseconds, std::memory_order_relaxed);

        m_RenderCallCount.fetch_add(1, std::memory_order_relaxed);

        std::uint64_t previousMaximum = m_MaxRenderNanoseconds.load(std::memory_order_relaxed);

        while (nanoseconds > previousMaximum && !m_MaxRenderNanoseconds.compare_exchange_weak(previousMaximum, nanoseconds, std::memory_order_relaxed, std::memory_order_relaxed))
        {
        }
    }

    void AudioSystem::GetVoiceDebugSnapshot(std::vector<AudioVoiceDebugInfo>& outVoices) const
    {
        outVoices.clear();

        outVoices.reserve(m_VoiceSlotStates.size());

        for (std::size_t i = 0; i < m_VoiceSlotStates.size(); ++i)
        {
            if (m_VoiceSlotStates[i] == AudioVoiceSlotState::Free)
            {
                continue;
            }

            AudioVoiceDebugInfo info;

            info.Handle.ID = static_cast<std::uint32_t>(i + 1);

            info.Handle.Generation = m_VoiceGenerations[i];

            info.State = m_VoiceSlotStates[i];

            if (i < m_VoiceSlotMetadata.size())
            {
                const AudioVoiceSlotMetadata& metadata = m_VoiceSlotMetadata[i];

                info.Bus = metadata.Bus;

                info.Priority = metadata.Priority;

                info.Stealable = metadata.Stealable;

                info.Looping = metadata.Looping;

                info.Paused = metadata.Paused;

                info.StartSequence = metadata.StartSequence;
            }

            outVoices.push_back(info);
        }
    }

    void AudioSystem::GetBusDebugSnapshot(std::array<AudioBusDebugInfo, GetAudioBusCount()>& outBuses) const
    {
        for (std::size_t i = 0; i < GetAudioBusCount(); ++i)
        {
            AudioBusDebugInfo info;

            info.Bus = static_cast<AudioBusID>(i);

            info.RequestedVolume = m_RequestedBusVolumes[i];

            info.RequestedMuted = m_RequestedBusMuted[i];

            outBuses[i] = info;
        }
    }

    bool AudioSystem::PushCommand(const AudioCommand& command)
    {
        if (!m_CommandQueue.Push(command))
        {
            ++m_CommandQueueFullCount;

            return false;
        }

        return true;
    }
}
