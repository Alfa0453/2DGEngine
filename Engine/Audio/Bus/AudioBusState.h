#pragma once

namespace Engine
{
    struct AudioBusState
    {
        float CurrentVolume = 1.0f;

        float TargetVolume = 1.0f;

        bool Muted = false;
    };
}