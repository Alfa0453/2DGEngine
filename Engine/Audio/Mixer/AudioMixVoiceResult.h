#pragma once

#include "../Playback/AudioPlaybackHandle.h"

namespace Engine
{
    struct AudioMixVoiceResult
    {
        bool Finished = false;

        AudioPlaybackHandle FinishedHandle;
    };
}