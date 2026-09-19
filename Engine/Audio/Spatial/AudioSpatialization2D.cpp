#include "AudioSpatialization2D.h"
#include "AudioAttenuationModel.h"
#include "AudioListenerState.h"

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

        float CalculateDopplerFactor(const Vector2& sourcePosition, const Vector2& sourceVelocity, const AudioListenerState& listener, float distance, float strength, float speedOfSound, float minFactor, float maxFactor)
        {
            if (strength <= 0.0f || distance <= 0.0001f || speedOfSound <= 0.0001f)
            {
                return 1.0f;
            }

            const float directionX = (sourcePosition.X - listener.Position.X) / distance;

            const float directionY = (sourcePosition.Y - listener.Position.Y) / distance;

            const float listenerRadial = listener.Velocity.X * directionX + listener.Velocity.Y * directionY;

            const float sourceRadial = sourceVelocity.X * directionX + sourceVelocity.Y * directionY;

            const float velocityLimit = speedOfSound * 0.95f;

            const float safeListener = std::clamp(listenerRadial, -velocityLimit, velocityLimit);

            const float safeSource = std::clamp(sourceRadial, -velocityLimit, velocityLimit);

            const float raw = (speedOfSound + safeListener) / (speedOfSound + safeSource);

            const float scaled = 1.0f + (raw - 1.0f) * strength;

            return std::clamp(scaled, minFactor, maxFactor);
        }
    }

    AudioSpatialResult2D AudioSpatialization2D::Calculate(const Vector2 &sourcePosition, const Vector2& sourceVelocity, const AudioListenerState &listener, float panDistance, float panStrength, float minDistance, float maxDistance, float attenuationStrength, AudioAttenuationModel attenuationModel, bool dopplerEnabled, float dopplerStrength, float speedOfSound, float minDopplerFactor, float maxDopplerFactor)
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

        if (dopplerEnabled && listener.Enabled)
        {
            result.DopplerFactor = CalculateDopplerFactor(sourcePosition, sourceVelocity, listener, result.Distance, dopplerStrength, speedOfSound, minDopplerFactor, maxDopplerFactor);
        }
        else 
        {
            result.DopplerFactor = 1.0f;
        }

        return result;
    }
}