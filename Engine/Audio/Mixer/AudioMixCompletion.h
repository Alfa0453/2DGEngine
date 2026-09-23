#pragma once

#include "../Playback/AudioPlaybackEvent.h"

namespace Engine
{
    enum class AudioMixCompletionType
    {
        Finished,
        Stopped
    };

    struct AudioMixCompletion
    {
        AudioPlaybackHandle Handle;

        AudioMixCompletionType Type = AudioMixCompletionType::Finished;
    };
}
