#include "AudioSystem.h"

#include "../Assets/AudioClip.h"
#include "../Backend/SDL/SDLAudioDevice.h"
#include "../Types/AudioLimits.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

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

        m_FinishedVoiceScratch.clear();

        m_FinishedVoiceScratch.reserve(m_Settings.MaxVoices);

        m_Mixer.Initialize(m_Settings.OutputFormat, m_Settings.MixFramesPerBlock);

        if (!m_Mixer.IsInitialized())
        {
            return false;
        }

        m_TotalVoiceSteals = 0;

        m_PlayFailuresNoVoice = 0;

        m_BusSystem.Reset();

        m_BusSystem.SetVolumeImeadiate(AudioBusID::Master, m_Settings.MasterVolume);

        m_RequestedMasterVolume = m_Settings.MasterVolume;

        m_RequestedBusVolumes.fill(1.0f);

        m_RequestedBusMuted.fill(false);

        m_RequestedBusVolumes[ToAudioBusIndex(AudioBusID::Master)] = m_Settings.MasterVolume;

        m_AudioBlocksMixed.store(0, std::memory_order_relaxed);

        m_AudioFrameMixed.store(0, std::memory_order_relaxed);

        m_AudioCommandsProcessed.store(0, std::memory_order_relaxed);

        m_RequestedListenerState = AudioListenerState{};

        m_AudioListenerState = AudioListenerState{};

        m_Device = std::make_unique<SDLAudioDevice>();

        m_Initialized = true;

        if (!m_Device->Initialize(m_Settings.OutputFormat, this))
        {
            m_Initialized = false;

            m_Device.reset();

            m_Mixer.Shutdown();

            return false;
        }

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

        m_VoiceSlotMetadata.clear();

        m_NextVoiceStartSequence = 1;

        m_CommandQueue.Shutdown();

        m_PlaybackEventQueue.Shutdown();

        m_FinishedVoiceScratch.clear();

        for (AudioVoice& voice : m_Voices)
        {
            voice.Stop();
        }

        m_Voices.clear();

        m_VoiceSlotStates.clear();

        m_VoiceGenerations.clear();

        m_Mixer.Shutdown();

        m_Initialized = false;
    }

    bool AudioSystem::IsInitialized() const
    {
        return m_Initialized;
    }

    bool AudioSystem::RenderAudioBlock(const float*& outSamples, std::size_t& outFrameCount)
    {
        outSamples = nullptr;

        outFrameCount = 0;

        ProcessAudioCommands();

        m_BusSystem.AdvanceSmoothing(m_Mixer.GetFramesPerBlock());

        m_FinishedVoiceScratch.clear();

        if (!m_Mixer.Mix(m_Voices, m_BusSystem, m_AudioListenerState, m_FinishedVoiceScratch))
        {
            return false;
        }

        for (const AudioPlaybackHandle handle : m_FinishedVoiceScratch)
        {
            QueuePlaybackEvent(AudioPlaybackEventType::Finished, handle);
        }

        const std::vector<float>& buffer = m_Mixer.GetMixBuffer();

        if (buffer.empty())
        {
            return false;
        }

        outSamples = buffer.data();

        outFrameCount = m_Mixer.GetFramesPerBlock();

        m_AudioBlocksMixed.fetch_add(1, std::memory_order_relaxed);

        m_AudioFrameMixed.fetch_add(static_cast<std::uint64_t>(m_Mixer.GetFramesPerBlock()), std::memory_order_relaxed);

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
        return Play(clip, AudioPlaybackSettings{}, Vector2{0.0f, 0.0f});
    }

    AudioPlaybackHandle AudioSystem::Play(const AudioClip& clip, const AudioPlaybackSettings& settings)
    {
        return Play(clip, settings, Vector2{0.0f, 0.0f});
    }

    AudioPlaybackHandle AudioSystem::Play(const AudioClip& clip, const AudioPlaybackSettings& settings, const Vector2& sourcePosiion)
    {
         if (!m_Initialized || !clip.IsValid())
        {
            return {};
        }

        const AudioPlaybackSettings sanitizedSettings = SanitizeAudioPlaybackSettings(settings);

        const std::size_t freeSlot = FindFreeVoiceSlot();

        if (freeSlot < m_VoiceSlotStates.size())
        {
            const AudioPlaybackHandle handle = CreatePlaybackHandleForSlot(freeSlot);

            AudioCommand command;

            command.Type = AudioCommandType::Play;

            command.Handle = handle;

            command.Clip = &clip;

            command.PlaybackSettings = sanitizedSettings;

            command.SourcePosition = sourcePosiion;

            if (!m_CommandQueue.Push(command))
            {
                ++m_CommandQueueFullCount;

                return {};
            }

            m_VoiceSlotStates[freeSlot] = AudioVoiceSlotState::PendingStart;

            ApplySlotMetadata(freeSlot, sanitizedSettings);

            return handle;
        }

        if (!sanitizedSettings.AllowVoiceSteal)
        {
            return {};
        }

        const std::size_t stealSlot = FindVoiceStealCandidtae(sanitizedSettings);

        if (stealSlot >= m_VoiceSlotStates.size())
        {
            return {};
        }

        const AudioPlaybackHandle previousHandle = CreatePlaybackHandleForSlot(stealSlot);

        const std::uint32_t newGeneration = GetNextGeneration(m_VoiceGenerations[stealSlot]);

        const AudioPlaybackHandle newHandle = CreatePlaybackHandleForGeneration(stealSlot, newGeneration);

        AudioCommand command;

        command.Type = AudioCommandType::ReplaceVoice;

        command.Handle = newHandle;

        command.PreviousHandle = previousHandle;

        command.SourcePosition = sourcePosiion;

        command.Clip = &clip;

        command.PlaybackSettings = sanitizedSettings;

        if (!m_CommandQueue.Push(command))
        {
            ++m_CommandQueueFullCount;

            return {};
        }

        m_VoiceGenerations[stealSlot] = newGeneration;

        m_VoiceSlotStates[stealSlot] = AudioVoiceSlotState::PendingStart;

        ApplySlotMetadata(stealSlot, sanitizedSettings);

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

        if (!m_CommandQueue.Push(command))
        {
            ++m_CommandQueueFullCount;

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

        command.PlaybackSettings.Volume = std::clamp(volume, 0.0f, 1.0f);

        if (!m_CommandQueue.Push(command))
        {
            ++m_CommandQueueFullCount;

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

        if (!m_CommandQueue.Push(command))
        {
            ++m_CommandQueueFullCount;

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

        if (!m_CommandQueue.Push(command))
        {
            ++m_CommandQueueFullCount;

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

        command.PlaybackSettings.Looping = looping;

        if (!m_CommandQueue.Push(command))
        {
            ++m_CommandQueueFullCount;

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

        if (!m_CommandQueue.Push(command))
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

        m_Settings.MaxPendingCommands = std::max<std::size_t>(64, m_Settings.MaxPendingCommands);

        m_Settings.MaxVoices = std::max<std::size_t>(1, m_Settings.MaxVoices);

        m_Settings.MasterVolume = std::clamp(m_Settings.MasterVolume, 0.0f, 1.0f);

        m_Settings.MixFramesPerBlock = std::max<std::size_t>(64, m_Settings.MixFramesPerBlock);
    }

    void AudioSystem::UpdateAudio()
    {
        if (!m_Initialized)
        {
            return;
        }

        m_Stats.CommandsProcessedThisUpdate = 0;

        m_Stats.FinishedVoicesThisUpdate = 0;

        m_Stats.TotalFramesMixed = m_AudioFrameMixed.load(std::memory_order_relaxed);

        m_Stats.TotalCommandsProcessed = m_AudioCommandsProcessed.load(std::memory_order_relaxed);

        const std::uint64_t blocks = m_AudioBlocksMixed.load(std::memory_order_relaxed);

        m_Stats.BlocksMixedThisUpdate = static_cast<std::size_t>(blocks - m_PreviousAudioFramesMixed);

        m_PreviousAudioBlocksMixed = blocks;

        ProcessPlaybackEvents();

        m_Stats.ActiveVoices = GetActiveVoiceCount();

        m_Stats.PendingCommands = m_CommandQueue.GetPendingCount();

        m_Stats.PendingPlaybackEvents = m_PlaybackEventQueue.GetPendingCount();

        m_Stats.CommandQueueFullCount = m_CommandQueueFullCount;

        m_Stats.PlaybackEventQueueOverflowCount = m_PlaybackEventQueueOverflowCount.load(std::memory_order_relaxed);

        m_Stats.TotalVoiceSteals = m_TotalVoiceSteals;

        m_Stats.PlayFailuresNoVoice = m_PlayFailuresNoVoice;

        m_Stats.ActiveMusicVoices = 0;

        m_Stats.ActiveSFXVoices = 0;

        m_Stats.ActiveUIVoices = 0;

        m_Stats.ActiveAmbienVoices = 0;

        for (std::size_t i = 0; i < m_VoiceSlotStates.size(); ++i)
        {
            if (m_VoiceSlotStates[i] == AudioVoiceSlotState::Free)
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
                    ++m_Stats.ActiveAmbienVoices;

                    break;
                }

                default:
                {
                    break;
                }
            }
        }
    }

    const AudioStats& AudioSystem::GetStats() const
    {
        return m_Stats;
    }

    bool AudioSystem::GetPlaybackSeconds(AudioPlaybackHandle handle, float& outSeconds) const
    {
        const AudioVoice* voice = FindVoice(handle);

        if (!voice)
        {
            return false;
        }

        outSeconds = voice->GetPlaybackSeconds();

        return true;
    }

    bool AudioSystem::GetPlaybackProgress(AudioPlaybackHandle handle, float& outProgress) const
    {
        const AudioVoice* voice = FindVoice(handle);

        if (!voice)
        {
            return false;
        }

        outProgress = voice->GetProgress();

        return true;
    }
    
    AudioVoice* AudioSystem::GetAudioVoiceForHandle(AudioPlaybackHandle handle)
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

        if (!voice.IsActive())
        {
            return nullptr;
        }

        if (voice.GetHandle() != handle)
        {
            return nullptr;
        }

        return &voice;
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

        if (!m_CommandQueue.Push(command))
        {
            ++m_CommandQueueFullCount;

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

        if (!m_CommandQueue.Push(command))
        {
            ++m_CommandQueueFullCount;

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
                const std::size_t slotIndex = command.Handle.IsValid() ? static_cast<std::size_t>(command.Handle.ID -1) : m_Voices.size();

                if (!command.Clip || !command.Clip->IsValid() || slotIndex >= m_Voices.size())
                {
                    QueuePlaybackEvent(AudioPlaybackEventType::FailedToStart, command.Handle);

                    break;
                }
                
                AudioVoice& voice = m_Voices[slotIndex];

                if (voice.IsActive())
                {
                    QueuePlaybackEvent(AudioPlaybackEventType::FailedToStart, command.Handle);

                    break;
                }

                voice.Start(command.Clip, command.Handle, command.PlaybackSettings, command.SourcePosition);

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

                    QueuePlaybackEvent(AudioPlaybackEventType::Stopped, command.Handle);
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
            }

            case AudioCommandType::SetPan:
            {
                AudioVoice* voice = GetAudioVoiceForHandle(command.Handle);

                if (voice)
                {
                    voice->SetPan(command.Value);
                }

                break;
            }

            case AudioCommandType::SetPitch:
            {
                AudioVoice* voice = GetAudioVoiceForHandle(command.Handle);

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
                    QueuePlaybackEvent(AudioPlaybackEventType::FailedToStart, command.Handle);

                    break;
                }

                AudioVoice& voice = m_Voices[slotIndex];

                if (voice.IsActive() && voice.GetHandle() != command.PreviousHandle)
                {
                    QueuePlaybackEvent(AudioPlaybackEventType::FailedToStart, command.Handle);

                    break;
                }

                if (!command.Clip || !command.Clip->IsValid())
                {
                    if (voice.IsActive())
                    {
                        voice.Stop();
                    }

                    QueuePlaybackEvent(AudioPlaybackEventType::FailedToStart, command.Handle);

                    break;
                }

                if (voice.IsActive())
                {
                    voice.Stop();
                }

                voice.Start(command.Clip, command.Handle, command.PlaybackSettings, command.SourcePosition);

                if (!voice.IsActive())
                {
                    QueuePlaybackEvent(AudioPlaybackEventType::FailedToStart, command.Handle);

                    break;
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
                AudioVoice* voice = GetAudioVoiceForHandle(command.Handle);

                if (voice)
                {
                    voice->SetSpatialPosition(command.SourcePosition);
                }

                break;
            }
        }
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

        if (m_VoiceGenerations[slotIndex] != handle.Generation)
        {
            return nullptr;
        }

        return &m_Voices[slotIndex];
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
        m_Stats.FinishedVoicesThisUpdate = 0;

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
                    }

                    break;
                }

                case AudioPlaybackEventType::Stopped:
                {
                    ReleaseVoiceSlot(slotIndex);

                    break;
                }

                case AudioPlaybackEventType::Finished:
                {
                    ReleaseVoiceSlot(slotIndex);

                    break;
                }

                case AudioPlaybackEventType::FailedToStart:
                {
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

    std::size_t AudioSystem::FindVoiceStealCandidtae(const AudioPlaybackSettings& incomingSettings) const
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

        if (!m_CommandQueue.Push(command))
        {
            ++m_CommandQueueFullCount;

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

        if (!m_CommandQueue.Push(command))
        {
            ++m_CommandQueueFullCount;

            return false;
        }

        return true;
    }
}