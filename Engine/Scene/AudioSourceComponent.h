#pragma once

#include "Component.h"

#include "../Audio/Playback/AudioPlaybackHandle.h"
#include "../Audio/Playback/AudioPlaybackSettings.h"

#include "../Math/Vector2.h"

namespace Engine
{
    class AudioClip;
    class AudioSystem;

    class AudioSourceComponent : public Component
    {
    public:

        AudioSourceComponent() = default;

        ~AudioSourceComponent() override;

        void Update(float deltaTime) override;

        void SetAudioSystem(AudioSystem* audioSystem);

        AudioSystem* GetAudioSystem() const;

        void SetClip(const AudioClip* clip);

        const AudioClip* GetClip() const;

        void SetPlaybackSettings(const AudioPlaybackSettings& settings);

        const AudioPlaybackSettings& GetPlaybackSettings() const;

        bool Play();

        bool Stop();

        bool IsPlaying() const;

        AudioPlaybackHandle GetPlaybackHandle() const;

        void SetPlayOnStart(bool playOnStart);

        bool GetPLayOnStart() const;

        void SetStopOnDestroy(bool stopOnDestroy);

        bool GetStopOnDestroy() const;

        void SetVolume(float volume);

        float GetVolume() const;

        void SetPan(float pan);

        float GetPan() const;

        void SetPitch(float pitch);

        float GetPitch() const;

        void SetLooping(bool looping);

        bool GetLooping() const;

        void SetBus(AudioBusID bus);

        AudioBusID GetBus() const;

        void SetPriority(std::uint8_t priority);

        std::uint8_t GetPriority() const;

        void SetStealable(bool stealable);

        bool IsStealable() const;

        void SetAllowVoiceSteal(bool allowVoiceSteal);

        bool GetAllowVoiceSteal() const;

        void SetSpatial(bool spatial);

        bool IsSpatial() const;

        void SetSpatialPanDistance(float distance);

        float GetSpatialPanDistance() const;

        void SetSpatialPanStrength(float strength);

        float GetSpatialPanStrength() const;

        void Start() override;

        void SetMinDistance(float distance);

        float GetMinDistance() const;

        void SetMaxDistance(float distance);

        float GetMaxDistance() const;

        void SetAttenuationStrength(float strength);

        float GetAttenuationStrength() const;

        void SetAttenuationModel(AudioAttenuationModel model);

        AudioAttenuationModel GetAttenuationModel() const;

        void SetVelocity(const Vector2& velocity);

        const Vector2& GetVelocity() const;

        void SetAutomaticVelocity(bool automatic);

        bool IsAutomaticVelocityEnabled() const;

        void SetDopplerEnabled(bool enabled);

        bool IsDopplerEnabled() const;

        void SetDopplerStrength(float strength);

        float GetDopplerStrength() const;

    private:

        void ClearPlaybackHandleIfInvalid();

    private:

        AudioSystem* m_AudioSystem = nullptr;

        const AudioClip* m_Clip = nullptr;

        AudioPlaybackSettings m_PlaybackSettings;

        AudioPlaybackHandle m_PlaybackHandle;

        bool m_PlayOnStart = false;

        bool m_StopOnDestroy = true;

        std::uint64_t m_LastTransformWorldVersion = 0;

        Vector2 m_PreviousWorldPosition{0.0f, 0.0f};

        Vector2 m_SpatialVelocity{0.0f, 0.0f};

        bool m_HasPreviousWorldPosition = false;

        bool m_AutomaticVelocity = true;

        bool m_HadAutomaticMotion = false;
    };
}