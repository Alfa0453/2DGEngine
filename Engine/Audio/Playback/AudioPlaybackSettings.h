#pragma once

#include "../Bus/AudioBusID.h"
#include "../Spatial/AudioAttenuationModel.h"

#include <type_traits>
#include <cstdint>

namespace Engine
{
    struct AudioPlaybackSettings
    {
        float Volume = 1.0f;

        float Pan = 0.0f;

        float Pitch = 1.0f;

        float SpatialPanDistance = 500.0f;

        float SpatialPanStrength = 1.0f;

        float MinDistance = 100.0f;

        float MaxDistance = 1000.0f;

        float AttenuationStrength = 1.0f;

        std::uint8_t Priority = 128;

        AudioBusID Bus = AudioBusID::SFX;

        AudioAttenuationModel AttenuationModel = AudioAttenuationModel::Linear;

        bool Looping = false;

        bool Stealable = true;

        bool AllowVoiceSteal = true;

        bool Spatial = false;
    };

    AudioPlaybackSettings SanitizeAudioPlaybackSettings(const AudioPlaybackSettings& settings);

    static_assert(std::is_trivially_copyable_v<AudioPlaybackSettings>, "AudioPlaybackSettings must remain trivally cpyable.");
}