#pragma once

#include "../Bus/AudioBusID.h"
#include "../Playback/AudioPlaybackHandle.h"
#include "../Playback/AudioVoiceSlotState.h"

#include <cstdint>

namespace Engine
{
    struct AudioVoiceDebugInfo
    {
        AudioPlaybackHandle Handle;

        AudioVoiceSlotState State = AudioVoiceSlotState::Free;

        AudioBusID Bus = AudioBusID::SFX;

        std::uint8_t Priority = 0;

        bool Stealable = true;

        bool Looping = false;

        bool Paused = false;

        std::uint64_t StartSequence = 0;
    };
}
