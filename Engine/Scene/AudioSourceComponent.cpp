#include "AudioSourceComponent.h"

#include "../Audio/Assets/AudioClip.h"
#include "../Audio/Core/AudioSystem.h"
#include "../Audio/Types/AudioLimits.h"

#include "TransformComponent.h"
#include "Entity.h"

#include <algorithm>

namespace Engine
{
    AudioSourceComponent::~AudioSourceComponent()
    {
        if (m_StopOnDestroy && m_AudioSystem && m_PlaybackHandle.IsValid())
        {
            m_AudioSystem->Stop(m_PlaybackHandle);
        }
    }

    void AudioSourceComponent::Update(float deltaTime)
    {
        (void)deltaTime;

        if (!m_PlaybackSettings.Spatial || !m_AudioSystem || !m_PlaybackHandle.IsValid())
        {
            return;
        }

        if (!m_AudioSystem->IsPlaying(m_PlaybackHandle))
        {
            m_PlaybackHandle = {};

            return;
        }

        Entity* owner = GetOwner();

        if (!owner)
        {
            return;
        }

        TransformComponent* transform = owner->GetComponent<TransformComponent>();

        if (!transform)
        {
            return;
        }

        const std::uint64_t worldVersion = transform->GetWorldVersion();

        if (worldVersion == m_LastTransformWorldVersion)
        {
            return;
        }

        const Vector2 worldPosition = transform->GetWorldPosition();

        const bool transformChanged = worldVersion != m_LastTransformWorldVersion;

        if (transformChanged)
        {
            const Vector2 worldPosition = transform->GetWorldPosition();

            if (m_AutomaticVelocity && deltaTime > 0.000001f)
            {
                if (m_HasPreviousWorldPosition)
                {
                    m_SpatialVelocity = (worldPosition - m_PreviousWorldPosition) / deltaTime;
                }
                else {
                    m_SpatialVelocity = {};
                }

                m_PreviousWorldPosition = worldPosition;

                m_HasPreviousWorldPosition = true;

                m_HadAutomaticMotion = m_SpatialVelocity.LengthSquared() > 0.000001f;
            }

            if (m_AudioSystem->SetSourceSpatialState(m_PlaybackHandle, worldPosition, m_SpatialVelocity))
            {
                m_LastTransformWorldVersion = worldVersion;
            }

            return;
        }

        if (m_AutomaticVelocity && m_HadAutomaticMotion)
        {
            m_SpatialVelocity = Vector2{0.0f, 0.0f};

            const Vector2 worldPosition = transform->GetWorldPosition();

            if (m_AudioSystem->SetSourceSpatialState(m_PlaybackHandle, worldPosition, m_SpatialVelocity))
            {
                m_HadAutomaticMotion = false;
            }
        }
    }

    void AudioSourceComponent::SetAudioSystem(AudioSystem* audioSystem)
    {
        if (m_AudioSystem == audioSystem)
        {
            return;
        }

        if (m_AudioSystem && m_PlaybackHandle.IsValid())
        {
            m_AudioSystem->Stop(m_PlaybackHandle);

            m_PlaybackHandle = {};
        }

        m_AudioSystem = audioSystem;
    }

    AudioSystem* AudioSourceComponent::GetAudioSystem() const
    {
        return m_AudioSystem;
    }

    void AudioSourceComponent::SetClip(const AudioClip* clip)
    {
        if (m_Clip == clip)
        {
            return;
        }

        if (IsPlaying())
        {
            Stop();
        }

        m_Clip = clip;
    }

    const AudioClip* AudioSourceComponent::GetClip() const
    {
        return m_Clip;
    }

    void AudioSourceComponent::SetPlaybackSettings(const AudioPlaybackSettings& settings)
    {
        m_PlaybackSettings = SanitizeAudioPlaybackSettings(settings);
    }

    const AudioPlaybackSettings& AudioSourceComponent::GetPlaybackSettings() const
    {
        return m_PlaybackSettings;
    }

    bool AudioSourceComponent::Play()
    {
        if (!m_AudioSystem || !m_Clip || !m_Clip->IsValid())
        {
            return false;
        }

        if (m_PlaybackHandle.IsValid() && m_AudioSystem->IsPlaying(m_PlaybackHandle))
        {
            Stop();
        }

        Vector2 sourcePosition{0.0f, 0.0f};

        if (m_PlaybackSettings.Spatial)
        {
            Entity* owner = GetOwner();

            if (!owner)
            {
                return false;
            }

            TransformComponent* transform = owner->GetComponent<TransformComponent>();

            if (!transform)
            {
                return false;
            }

            sourcePosition = transform->GetWorldPosition();

            m_PreviousWorldPosition = sourcePosition;

            m_HasPreviousWorldPosition = true;

            m_LastTransformWorldVersion = transform->GetWorldVersion();
        }

        m_SpatialVelocity = Vector2{0.0f, 0.0f};

        m_PlaybackHandle = m_AudioSystem->Play(*m_Clip, m_PlaybackSettings, sourcePosition, m_SpatialVelocity);

        return m_PlaybackHandle.IsValid();
    }

    bool AudioSourceComponent::Stop()
    {
        if (!m_AudioSystem || !m_PlaybackHandle.IsValid())
        {
            return false;
        }

        const bool stopped = m_AudioSystem->Stop(m_PlaybackHandle);

        if (stopped)
        {
            m_PlaybackHandle = {};
        }

        return stopped;
    }

    bool AudioSourceComponent::IsPlaying() const
    {
        if (!m_AudioSystem || !m_PlaybackHandle.IsValid())
        {
            return false;
        }

        return m_AudioSystem->IsPlaying(m_PlaybackHandle);
    }

    AudioPlaybackHandle AudioSourceComponent::GetPlaybackHandle() const
    {
        return m_PlaybackHandle;
    }

    void AudioSourceComponent::ClearPlaybackHandleIfInvalid()
    {
        if (!m_PlaybackHandle.IsValid())
        {
            return;
        }

        if (!m_AudioSystem || !m_AudioSystem->IsPlaying(m_PlaybackHandle))
        {
            m_PlaybackHandle = {};
        }
    }

    void AudioSourceComponent::SetPlayOnStart(bool playOnStart)
    {
        m_PlayOnStart = playOnStart;
    }

    bool AudioSourceComponent::GetPLayOnStart() const
    {
        return m_PlayOnStart;
    }

    void AudioSourceComponent::Start()
    {
        if (m_PlayOnStart)
        {
            Play();
        }
    }

    void AudioSourceComponent::SetStopOnDestroy(bool stopOnDestroy)
    {
        m_StopOnDestroy = stopOnDestroy;
    }

    bool AudioSourceComponent::GetStopOnDestroy() const
    {
        return m_StopOnDestroy;
    }

    void AudioSourceComponent::SetVolume(float volume)
    {
        volume = std::clamp(volume, AudioLimits::MinVolume, AudioLimits::MaxVolume);

        m_PlaybackSettings.Volume = volume;

        if (m_AudioSystem && m_PlaybackHandle.IsValid() && m_AudioSystem->IsPlaying(m_PlaybackHandle))
        {
            m_AudioSystem->SetVolume(m_PlaybackHandle, volume);
        }
    }

    float AudioSourceComponent::GetVolume() const
    {
        return m_PlaybackSettings.Volume;
    }

    void AudioSourceComponent::SetPan(float pan)
    {
        pan = std::clamp(pan, AudioLimits::MinPan, AudioLimits::MaxPan);

        m_PlaybackSettings.Pan = pan;

        if (m_AudioSystem && m_PlaybackHandle.IsValid() && m_AudioSystem->IsPlaying(m_PlaybackHandle))
        {
            m_AudioSystem->SetPan(m_PlaybackHandle, pan);
        }
    }

    float AudioSourceComponent::GetPan() const
    {
        return m_PlaybackSettings.Pan;
    }

    void AudioSourceComponent::SetPitch(float pitch)
    {
        pitch = std::clamp(pitch, AudioLimits::MinPitch, AudioLimits::MaxPitch);

        m_PlaybackSettings.Pitch = pitch;

        if (m_AudioSystem && m_PlaybackHandle.IsValid() && m_AudioSystem->IsPlaying(m_PlaybackHandle))
        {
            m_AudioSystem->SetPitch(m_PlaybackHandle, pitch);
        }
    }

    float AudioSourceComponent::GetPitch() const
    {
        return m_PlaybackSettings.Pitch;
    }

    void AudioSourceComponent::SetLooping(bool looping)
    {
        m_PlaybackSettings.Looping = looping;

        if (m_AudioSystem && m_PlaybackHandle.IsValid() && m_AudioSystem->IsPlaying(m_PlaybackHandle))
        {
            m_AudioSystem->SetLooping(m_PlaybackHandle, looping);
        }
    }

    bool AudioSourceComponent::GetLooping() const
    {
        return m_PlaybackSettings.Looping;
    }

    void AudioSourceComponent::SetBus(AudioBusID bus)
    {
        m_PlaybackSettings.Bus = bus;
    }

    AudioBusID AudioSourceComponent::GetBus() const
    {
        return m_PlaybackSettings.Bus;
    }

    void AudioSourceComponent::SetPriority(std::uint8_t priority)
    {
        m_PlaybackSettings.Priority = priority;
    }

    std::uint8_t AudioSourceComponent::GetPriority() const
    {
        return m_PlaybackSettings.Priority;
    }

    void AudioSourceComponent::SetStealable(bool stealable)
    {
        m_PlaybackSettings.Stealable = stealable;
    }

    bool AudioSourceComponent::IsStealable() const
    {
        return m_PlaybackSettings.Stealable;
    }

    void AudioSourceComponent::SetAllowVoiceSteal(bool allowVoiceSteal)
    {
        m_PlaybackSettings.AllowVoiceSteal = allowVoiceSteal;
    }

    bool AudioSourceComponent::GetAllowVoiceSteal() const
    {
        return m_PlaybackSettings.AllowVoiceSteal;
    }

    void AudioSourceComponent::SetSpatial(bool spatial)
    {
        m_PlaybackSettings.Spatial = spatial;
    }

    bool AudioSourceComponent::IsSpatial() const
    {
        return m_PlaybackSettings.Spatial;
    }

    void AudioSourceComponent::SetSpatialPanDistance(float distance)
    {
        m_PlaybackSettings.SpatialPanDistance = std::max(distance, 1.0f);
    }

    float AudioSourceComponent::GetSpatialPanDistance() const
    {
        return m_PlaybackSettings.SpatialPanDistance;
    }

    void AudioSourceComponent::SetSpatialPanStrength(float strength)
    {
        m_PlaybackSettings.SpatialPanStrength = std::max(strength, 0.0f);
    }

    float AudioSourceComponent::GetSpatialPanStrength() const
    {
        return m_PlaybackSettings.SpatialPanStrength;
    }

    void AudioSourceComponent::SetMinDistance(float distance)
    {
        distance = std::max(distance, 0.0f);

        m_PlaybackSettings.MinDistance = distance;

        if (m_PlaybackSettings.MaxDistance <= distance)
        {
            m_PlaybackSettings.MaxDistance = distance + 1.0f;
        }
    }

    float AudioSourceComponent::GetMinDistance() const
    {
        return m_PlaybackSettings.MinDistance;
    }

    void AudioSourceComponent::SetMaxDistance(float distance)
    {
        m_PlaybackSettings.MaxDistance = std::max(distance, m_PlaybackSettings.MinDistance + 1.0f);
    }

    float AudioSourceComponent::GetMaxDistance() const
    {
        return m_PlaybackSettings.MaxDistance;
    }

    void AudioSourceComponent::SetAttenuationStrength(float strength)
    {
        m_PlaybackSettings.AttenuationStrength = std::max(strength, 0.0f);
    }

    float AudioSourceComponent::GetAttenuationStrength() const
    {
        return m_PlaybackSettings.AttenuationStrength;
    }

    void AudioSourceComponent::SetAttenuationModel(AudioAttenuationModel model)
    {
        m_PlaybackSettings.AttenuationModel = model;
    }

    AudioAttenuationModel AudioSourceComponent::GetAttenuationModel() const
    {
        return m_PlaybackSettings.AttenuationModel;
    }

    void AudioSourceComponent::SetVelocity(const Vector2& velocity)
    {
        m_SpatialVelocity = velocity;

        m_AutomaticVelocity = false;
    }

    const Vector2& AudioSourceComponent::GetVelocity() const
    {
        return m_SpatialVelocity;
    }

    void AudioSourceComponent::SetAutomaticVelocity(bool automatic)
    {
        if (m_AutomaticVelocity == automatic)
        {
            return;
        }

        m_AutomaticVelocity = automatic;

        if (automatic)
        {
            m_HasPreviousWorldPosition = false;

            m_SpatialVelocity = Vector2{0.0f, 0.0f};
        }
    }

    bool AudioSourceComponent::IsAutomaticVelocityEnabled() const
    {
        return m_AutomaticVelocity;
    }

    void AudioSourceComponent::SetDopplerEnabled(bool enabled)
    {
        m_PlaybackSettings.DopplerEnabled = enabled;
    }

    bool AudioSourceComponent::IsDopplerEnabled() const
    {
        return m_PlaybackSettings.DopplerEnabled;
    }

    void AudioSourceComponent::SetDopplerStrength(float strength)
    {
        m_PlaybackSettings.DopplerStrength = std::max(strength, 0.0f);
    }

    float AudioSourceComponent::GetDopplerStrength() const
    {
        return m_PlaybackSettings.DopplerStrength;
    }
}