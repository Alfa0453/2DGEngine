#pragma once

#include "../Bus/AudioBusID.h"

#include <cstdint>

namespace Engine
{
    struct AudioVoiceSlotMetadata
    {
        std::uint8_t Priority = 0;

        AudioBusID Bus = AudioBusID::SFX;

        bool Stealable = true;

        bool Looping = false;

        std::uint64_t StartSequence = 0;

        bool Paused = false;
    };
}
