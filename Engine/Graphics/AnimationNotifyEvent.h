#pragma once

#include "../Scene/EntityHandle.h"

#include <string>

namespace Engine
{
    struct AnimationNotifyEvent
    {
        EntityHandle Source;

        std::string StateName;

        std::string ClipName;

        std::string NotifyName;

        float NormalizedTime = 0.0f;
    };
}