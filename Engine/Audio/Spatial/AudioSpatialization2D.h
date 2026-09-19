#pragma once

#include "AudioAttenuationModel.h"
#include "AudioListenerState.h"

#include "../../Math/Vector2.h"

namespace Engine
{
    struct AudioSpatialResult2D
    {
        float Pan = 0.0f;

        float Distance = 0.0f;

        float DistanceGain = 1.0f;

        float DopplerFactor = 1.0f;
    };

    class AudioSpatialization2D
    {
    public:

        static AudioSpatialResult2D Calculate(const Vector2& sourcePosition, const Vector2& sourceVelocity, const AudioListenerState& listener, 
                                             float panDistance, float panStrength, float minDistance, float maxDistance, 
                                             float attenuationStrength, AudioAttenuationModel attenuationModel, 
                                             bool dopplerEnabled, float dopplerStrength, float speedOfSound, 
                                             float minDopplerFactor, float maxDopplerFactor);
    };
}