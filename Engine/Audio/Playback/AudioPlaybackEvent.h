#pragma once

#include "AudioPlaybackHandle.h"

namespace Engine
{
    enum class AudioPlaybackEventType
    {
        Started,

        Stopped,

        Finished,

        FailedToStart
    };

    struct AudioPlaybackEvent
    {
        AudioPlaybackEventType Type = AudioPlaybackEventType::Finished;

        AudioPlaybackHandle Handle;
    };
}