#include "AudioVoice.h"

#include "../Assets/AudioClip.h"

#include "../Types/AudioLimits.h"

#include <algorithm>

namespace Engine
{
    void AudioVoice::Start(const AudioClip* clip, AudioPlaybackHandle handle, const AudioPlaybackSettings& settings, const Vector2& sourcePosition, const Vector2& sourceVelocity)
    {
        m_Clip = clip;

        m_Handle = handle;

        m_PlaybackFrame = 0;

        m_CurrentVolume = settings.Volume;

        m_TargetVolume = settings.Volume;

        m_CurrentPan = settings.Pan;

        m_TargetPan = settings.Pan;

        m_CurrentPitch = settings.Pitch;

        m_TargetPitch = settings.Pitch;

        m_Looping = settings.Looping;

        m_Active = (m_Clip != nullptr && m_Clip->IsValid());

        m_Bus = settings.Bus;

        m_SpatialPosition = sourcePosition;

        m_Spatial = settings.Spatial;

        m_SpatialPanDistance = settings.SpatialPanDistance;

        m_StatialPanStrength = settings.SpatialPanStrength;

        m_MinDistance = settings.MinDistance;

        m_MaxDistance = settings.MaxDistance;

        m_AttenuationStrength = settings.AttenuationStrength;

        m_AttenuationModel = settings.AttenuationModel;

        m_SpatialVelocity = sourceVelocity;

        m_DopplerEnabled = settings.DopplerEnabled;

        m_DopplerStrength = settings.DopplerStrength;
    }

    void AudioVoice::Stop()
    {
        m_Clip = nullptr;

        m_Handle = {};

        m_PlaybackFrame = 0;

        m_CurrentVolume = 1.0f;

        m_TargetVolume = 1.0f;

        m_CurrentPan = 0.0f;

        m_TargetPan = 0.0f;

        m_CurrentPitch = 1.0f;

        m_TargetPitch = 1.0f;

        m_Looping = false;

        m_Active = false;

        m_Bus = AudioBusID::SFX;

        m_SpatialPosition = Vector2{0.0f, 0.0f};

        m_Spatial = false;

        m_SpatialPanDistance = 500.0f;

        m_StatialPanStrength = 1.0f;

        m_MinDistance = 100.0f;

        m_MaxDistance = 1000.0f;

        m_AttenuationStrength = 1.0f;

        m_AttenuationModel = AudioAttenuationModel::Linear;

        m_SpatialVelocity = Vector2{0.0f, 0.0f};

        m_DopplerEnabled = false;

        m_DopplerStrength = 1.0f;
    }

    bool AudioVoice::IsActive() const
    {
        return m_Active;
    }

    const AudioClip* AudioVoice::GetClip() const
    {
        return m_Clip;
    }

    const AudioPlaybackHandle& AudioVoice::GetHandle() const
    {
        return m_Handle;
    }

    double AudioVoice::GetPlaybackFrame() const
    {
        return m_PlaybackFrame;
    }

    void AudioVoice::SetPlaybackFrame(double frame)
    {
        m_PlaybackFrame = frame;
    }

    float AudioVoice::GetVolume() const
    {
        return m_TargetVolume;
    }

    void AudioVoice::SetVolume(float volume)
    {
        m_TargetVolume = std::clamp(volume, AudioLimits::MinVolume, AudioLimits::MaxVolume);
    }

    bool AudioVoice::IsLooping() const
    {
        return m_Looping;
    }

    void AudioVoice::SetLooping(bool looping)
    {
        m_Looping = looping;
    }

    float AudioVoice::GetCurrentVolume() const
    {
        return m_CurrentVolume;
    }

    void AudioVoice::SetCurrenVolume(float volume)
    {
        m_CurrentVolume = std::clamp(volume, 0.0f, 1.0f);
    }

    float AudioVoice::GetPan() const
    {
        return m_TargetPan;
    }

    void AudioVoice::SetPan(float pan)
    {
        m_TargetPan = std::clamp(pan, AudioLimits::MinPan, AudioLimits::MaxPan);
    }

    float AudioVoice::GetCurrentPan() const
    {
        return m_CurrentPan;
    }

    void AudioVoice::SetCurrentPan(float pan)
    {
        m_CurrentPan = std::clamp(pan, -1.0f, 1.0f);
    }

    float AudioVoice::GetPitch() const
    {
        return m_TargetPitch;
    }

    void AudioVoice::SetPitch(float pitch)
    {
        m_TargetPitch = std::clamp(pitch, AudioLimits::MinPitch, AudioLimits::MaxPitch);
    }

    float AudioVoice::GetCurrentPitch() const
    {
        return m_CurrentPitch;
    }

    void AudioVoice::SetCurrentPitch(float pitch)
    {
        m_CurrentPitch = std::clamp(pitch, AudioLimits::MinPitch, AudioLimits::MaxPitch);
    }

    float AudioVoice::GetPlaybackSeconds() const
    {
        if (!m_Clip)
        {
            return 0.0f;
        }

        const AudioFormat& format = m_Clip->GetFormat();

        if (format.SampleRate == 0)
        {
            return 0.0f;
        }

        return static_cast<float>(m_PlaybackFrame) / static_cast<double>(format.SampleRate);
    }

    float AudioVoice::GetProgress() const
    {
        if (!m_Clip)
        {
            return 0.0f;
        }

        const std::size_t totalFrames = m_Clip->GetFrameCount();

        if (totalFrames == 0)
        {
            return 0.0f;
        }

        const double progress = m_PlaybackFrame / static_cast<double>(totalFrames);

        return std::clamp(static_cast<float>(progress), 0.0f, 1.0f);
    }

    std::uint32_t AudioVoice::GetGeneration() const
    {
        return m_Generation;
    }

    void AudioVoice::AdvanceGeneration()
    {
        ++m_Generation;

        if (m_Generation == 0)
        {
            m_Generation = 1;
        }
    }

    AudioBusID AudioVoice::GetBus() const
    {
        return m_Bus;
    }

    void AudioVoice::SetSpatialPosition(const Vector2& position)
    {
        m_SpatialPosition = position;
    }

    const Vector2& AudioVoice::GetSpatialPosition() const
    {
        return m_SpatialPosition;
    }

    bool AudioVoice::IsSpatial() const
    {
        return m_Spatial;
    }

    float AudioVoice::GetSpatialPanDistance() const
    {
        return m_SpatialPanDistance;
    }

    float AudioVoice::GetSpatialPanStrength() const
    {
        return m_StatialPanStrength;
    }

    float AudioVoice::GetMinDistance() const
    {
        return m_MinDistance;
    }

    float AudioVoice::GetMaxDistance() const
    {
        return m_MaxDistance;
    }

    float AudioVoice::GetAttenuationStrength() const
    {
        return m_AttenuationStrength;
    }

    AudioAttenuationModel AudioVoice::GetAttenuationModel() const
    {
        return m_AttenuationModel;
    }

    void AudioVoice::SetSpatialVelocity(const Vector2& velocity)
    {
        m_SpatialVelocity = velocity;
    }

    const Vector2& AudioVoice::GetSpatialVelocity() const
    {
        return m_SpatialVelocity;
    }

    bool AudioVoice::IsDopplerEnabled() const
    {
        return m_DopplerEnabled;
    }

    float AudioVoice::GetDopplerStrength() const
    {
        return m_DopplerStrength;
    }
}
