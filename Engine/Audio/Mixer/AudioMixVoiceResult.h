#pragma once

#include "../Playback/AudioPlaybackHandle.h"

namespace Engine
{
    enum class AudioMixVoiceEndReason
    {
        None,
        Finished,
        FadeStopped
    };

    struct AudioMixVoiceResult
    {
        AudioMixVoiceEndReason EndReason = AudioMixVoiceEndReason::None;

        AudioPlaybackHandle FinishedHandle;
    };
}
