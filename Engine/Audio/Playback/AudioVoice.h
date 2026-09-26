#pragma once

#include "AudioPlaybackHandle.h"
#include "AudioPlaybackSettings.h"

#include "../Bus/AudioBusID.h"
#include "../Spatial/AudioAttenuationModel.h"
#include "../../Math/Vector2.h"
#include "AudioSourceKind.h"

#include <cstddef>

namespace Engine
{
    class AudioClip;
    class AudioStream;
    class AudioAssetRecord;


    class AudioVoice
    {
    public:

        AudioVoice() = default;

        void Start(const AudioClip* clip, AudioPlaybackHandle handle, const AudioPlaybackSettings& settings, const Vector2& sourcePosition, const Vector2& sourceVelocity, AudioAssetRecord* assetRecord = nullptr);

        void StartStream(AudioStream* stream, AudioPlaybackHandle handle, const AudioPlaybackSettings& settings, AudioAssetRecord* assetRecord = nullptr);

        void Stop();

        bool IsActive() const;

        // Asset manager will own the clip
        const AudioClip* GetClip() const;

        const AudioPlaybackHandle& GetHandle() const;

        double GetPlaybackFrame() const;

        void SetPlaybackFrame(double frame);

        float GetVolume() const;

        void SetVolume(float volume);

        float GetCurrentVolume() const;

        void SetCurrentVolume(float volume);

        float GetPan() const;

        void SetPan(float pan);

        float GetCurrentPan() const;

        void SetCurrentPan(float pan);

        float GetPitch() const;

        void SetPitch(float pitch);

        float GetCurrentPitch() const;

        void SetCurrentPitch(float pitch);

        bool IsLooping() const;

        void SetLooping(bool looping);

        float GetPlaybackSeconds() const;

        float GetProgress() const;

        AudioBusID GetBus() const;

        void SetSpatialPosition(const Vector2& position);

        const Vector2& GetSpatialPosition() const;

        bool IsSpatial() const;

        float GetSpatialPanDistance() const;

        float GetSpatialPanStrength() const;

        float GetMinDistance() const;

        float GetMaxDistance() const;

        float GetAttenuationStrength() const;

        AudioAttenuationModel GetAttenuationModel() const;

        void SetSpatialVelocity(const Vector2& velocity);

        const Vector2& GetSpatialVelocity() const;

        bool IsDopplerEnabled() const;

        float GetDopplerStrength() const;

        void Pause();

        void Resume();

        bool IsPaused() const;

        bool SeekSeconds(float seconds);

        void StartFade(float targetGain, std::uint64_t durationFrames, bool stopWhenComplete);

        float GetFadeGain() const;

        bool AdvanceFade();

        bool ShouldStopAfterFade() const;

        void SetFadeGainImmediate(float gain);

        AudioSourceKind GetSourceKind() const;

        AudioStream* GetStream() const;

        AudioAssetRecord* GetAssetRecord() const;

    private:

        AudioSourceKind m_SourceKind = AudioSourceKind::None;

        const AudioClip* m_Clip = nullptr;

        AudioStream* m_Stream = nullptr;

        AudioAssetRecord* m_AssetRecord = nullptr;

        AudioPlaybackHandle m_Handle;

        double m_PlaybackFrame = 0.0;

        float m_CurrentVolume = 1.0f;

        float m_TargetVolume = 1.0f;

        float m_CurrentPan = 0.0f;

        float m_TargetPan = 0.0f;

        float m_CurrentPitch = 1.0f;

        float m_TargetPitch = 1.0f;

        bool m_Looping = false;

        bool m_Active = false;

        AudioBusID m_Bus = AudioBusID::SFX;

        Vector2 m_SpatialPosition{0.0f, 0.0f};

        float m_SpatialPanDistance = 500.0f;

        float m_SpatialPanStrength = 1.0f;

        bool m_Spatial = false;

        float m_MinDistance = 100.0f;

        float m_MaxDistance = 1000.0f;

        float m_AttenuationStrength = 1.0f;

        AudioAttenuationModel m_AttenuationModel = AudioAttenuationModel::Linear;

        Vector2 m_SpatialVelocity{0.0f, 0.0f};

        float m_DopplerStrength = 1.0f;

        bool m_DopplerEnabled = false;

        float m_FadeGain = 1.0f;

        float m_FadeTargetGain = 1.0f;

        float m_FadeStepPerFrame = 0.0f;

        std::uint64_t m_FadeFramesRemaining = 0;

        bool m_StopWhenFadeComplete = false;

        bool m_Paused = false;
    };
}
