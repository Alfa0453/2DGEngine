#pragma once

#include "AudioPlaybackHandle.h"
#include "AudioPlaybackSettings.h"

#include "../Bus/AudioBusID.h"
#include "../Spatial/AudioAttenuationModel.h"
#include "../../Math/Vector2.h"

#include <cstddef>

namespace Engine
{
    class AudioClip;


    class AudioVoice
    {
    public:

        AudioVoice() = default;

        void Start(const AudioClip* clip, AudioPlaybackHandle handle, const AudioPlaybackSettings& settings, const Vector2& sourcePosition);

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
        
        void SetCurrenVolume(float volume);

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

        std::uint32_t GetGeneration() const;

        void AdvanceGeneration();

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

    private:

        const AudioClip* m_Clip = nullptr;

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

        std::uint32_t m_Generation = 1;

        AudioBusID m_Bus = AudioBusID::SFX;

        Vector2 m_SpatialPosition{0.0f, 0.0f};

        float m_SpatialPanDistance = 500.0f;

        float m_StatialPanStrength = 1.0f;

        bool m_Spatial = false;

        float m_MinDistance = 100.0f;

        float m_MaxDistance = 1000.0f;

        float m_AttenuationStrength = 1.0f;

        AudioAttenuationModel m_AttenuationModel = AudioAttenuationModel::Linear;
    };
}