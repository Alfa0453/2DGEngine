#pragma once

#include "../Bus/AudioBusID.h"

namespace Engine
{
    struct AudioBusDebugInfo
    {
        AudioBusID Bus = AudioBusID::Master;

        float RequestedVolume = 1.0f;

        bool RequestedMuted = false;
    };
}
