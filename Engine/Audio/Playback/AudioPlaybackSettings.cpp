#include "AudioPlaybackSettings.h"

#include "../Types/AudioLimits.h"

#include <algorithm>

namespace Engine
{
    AudioPlaybackSettings SanitizeAudioPlaybackSettings(const AudioPlaybackSettings &settings)
    {
        AudioPlaybackSettings sanitized = settings;

        sanitized.Volume = std::clamp(sanitized.Volume, AudioLimits::MinVolume, AudioLimits::MaxVolume);

        sanitized.Pan = std::clamp(sanitized.Pan, AudioLimits::MinPan, AudioLimits::MaxPan);

        sanitized.Pitch = std::clamp(sanitized.Pitch, AudioLimits::MinPitch, AudioLimits::MaxPitch);

        sanitized.SpatialPanDistance = std::max(sanitized.SpatialPanDistance, 1.0f);

        sanitized.SpatialPanStrength = std::max(sanitized.SpatialPanStrength, 0.0f);

        sanitized.MinDistance = std::max(sanitized.MinDistance, 0.0f);

        sanitized.MaxDistance = std::max(sanitized.MaxDistance, sanitized.MinDistance + 1.0f);

        sanitized.AttenuationStrength = std::max(sanitized.AttenuationStrength, 0.0f);

        return sanitized;
    }
}