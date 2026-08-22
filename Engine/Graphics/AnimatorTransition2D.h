#pragma once

#include "AnimatorCondition2D.h"

#include <string>
#include <vector>

namespace Engine
{
    inline constexpr const char* AnimatorAnyState = "__AnyState__";

    struct AnimatorTransition2D
    {
        std::string FromState;

        std::string ToState;

        std::vector<AnimatorCondition2D> Conditions;

        bool HasExitTime = false;

        float ExiTime = 1.0f;

        int Priority = 0;

        bool CanTransitionToSelf = false;
    };
}