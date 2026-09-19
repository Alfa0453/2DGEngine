#pragma once

#include "AudioFormat.h"

#include <cstddef>

namespace Engine
{
    struct AudioSettings
    {
        AudioFormat OutputFormat;

        std::size_t MaxVoices = 64;

        float MasterVolume = 1.0f;

        std::size_t MixFramesPerBlock = 512;

        std::size_t MaxPendingCommands = 128;
    };
}