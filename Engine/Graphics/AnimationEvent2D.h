#pragma once

#include <string>

namespace Engine
{
    struct AnimationEvent2D
    {
        std::string Name;

        float NormalizedTime = 0.0f;

        AnimationEvent2D() = default;

        AnimationEvent2D(const std::string& name, float normalizedTime)
            : Name(name), NormalizedTime(normalizedTime)
        {
        }

        bool IsValid() const
        {
            return !Name.empty() &&
                   NormalizedTime >= 0.0f &&
                   NormalizedTime <= 1.0f;
        }
    };
}