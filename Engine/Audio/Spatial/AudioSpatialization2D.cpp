#include "AudioSpatialization2D.h"
#include "AudioAttenuationModel.h"

#include <algorithm>
#include <cmath>

namespace Engine
{
    namespace
    {
        float CalculateLinearAttenuation(float distance, float minDistance, float maxDistance)
        {
            if (distance <= minDistance)
            {
                return 1.0f;
            }

            if (distance >= maxDistance)
            {
                return 0.0f;
            }

            const float range = maxDistance - minDistance;

            const float t = (distance - minDistance) / range;

            return 1.0f - t;
        }

        float CalculateInverseAttenuation(float distance, float minDistance, float maxDistance)
        {
            if (distance <= minDistance)
            {
                return 1.0f;
            }

            if (distance >= maxDistance)
            {
                return 0.0f;
            }

            const float safeMinDistance = std::max(minDistance, 1.0f);

            const float raw = safeMinDistance / distance;

            const float rawAtMax = safeMinDistance / maxDistance;

            const float denominator = 1.0f - rawAtMax;

            if (denominator <= 0.000001f)
            {
                return 0.0f;
            }

            return std::clamp((raw - rawAtMax) / denominator, 0.0f, 1.0f);
        }

        float ApplyAttenuationStrength(float gain, float strength)
        {
            gain = std::clamp(gain, 0.0f, 1.0f);

            if (strength <= 0.0f)
            {
                return 1.0f;
            }

            return std::pow(gain, strength);
        }

        float CalculateDistanceGain(float distance, float minDistance, float maxDistance, float strength, AudioAttenuationModel model)
        {
            float gain = 1.0f;

            switch (model)
            {
                case AudioAttenuationModel::Linear:
                {
                    gain = CalculateLinearAttenuation(distance, minDistance, maxDistance);

                    break;
                }

                case AudioAttenuationModel::Inverse:
                {
                    gain = CalculateInverseAttenuation(distance, minDistance, maxDistance);

                    break;
                }
            }

            return ApplyAttenuationStrength(gain, strength);
        }
    }

    AudioSpatialResult2D AudioSpatialization2D::Calculate(const Vector2 &sourcePosition, const AudioListenerState &listener, float panDistance, float panStrength, float minDistance, float maxDistance, float attenuationStrength, AudioAttenuationModel attenuationModel)
    {
        AudioSpatialResult2D result;

        const float deltaX = sourcePosition.X - listener.Position.X;

        const float deltaY = sourcePosition.Y - listener.Position.Y;

        result.Distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);

        if (!listener.Enabled)
        {
            result.Pan = 0.0f;

            result.DistanceGain = 1.0f;

            return result;
        }

        panDistance = std::max(panDistance, 1.0f);

        panStrength = std::max(panStrength, 0.0f);

        minDistance = std::max(minDistance, 0.0f);

        maxDistance = std::max(maxDistance, minDistance + 1.0f);

        attenuationStrength = std::max(attenuationStrength, 0.0f);

        const float normalizedHorizontal = deltaX / panDistance;

        result.Pan = std::clamp(normalizedHorizontal * panStrength, -1.0f, 1.0f);

        result.DistanceGain = CalculateDistanceGain(result.Distance, minDistance, maxDistance, attenuationStrength, attenuationModel);

        return result;
    }
}