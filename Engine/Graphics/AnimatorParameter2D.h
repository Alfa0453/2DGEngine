#pragma once

#include "AnimatorParameterType.h"

#include <string>

namespace Engine
{
    struct AnimatorParameter2D
    {
        std::string Name;

        AnimatorParameterType Type = AnimatorParameterType::Float;

        bool BoolValue = false;

        int IntValue = 0;

        float FloatValue = 0.0f;

        bool TriggerValue = false;
    };
}