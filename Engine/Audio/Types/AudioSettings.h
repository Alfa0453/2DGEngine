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

        float SpeedOfSound = 1000.0f;

        float MinDopplerFactor = 0.5f;

        float MaxDopplerFactor = 2.0f;
    };
}