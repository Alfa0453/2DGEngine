#pragma once

#include "AnimationClip2D.h"

#include <string>

namespace Engine
{
    struct AnimatorState2D
    {
        std::string Name;

        const AnimationClip2D* Clip = nullptr;

        float SpeedMultiplier = 1.0f;

        AnimatorState2D() = default;

        AnimatorState2D(const std::string& name, const AnimationClip2D* clip, float speedMultiplier = 1.0f)
            : Name(name), Clip(clip), SpeedMultiplier(speedMultiplier)
        {
        }

        bool IsValid() const
        {
            return !Name.empty() &&
                   Clip != nullptr &&
                   Clip->IsValid() && 
                   SpeedMultiplier >= 0.0f;
        }
    };
}