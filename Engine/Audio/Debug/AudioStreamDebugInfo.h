#pragma once

#include "../Assets/AudioAssetHandle.h"
#include "../Streaming/AudioStreamState.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace Engine
{
    struct AudioStreamDebugInfo
    {
        AudioAssetHandle Asset;

        std::string Path;

        AudioStreamState State = AudioStreamState::Closed;

        std::size_t AvailableFrames = 0;

        std::uint64_t Underflows = 0;

        bool Looping = false;

        bool HasConsumer = false;

        float DurationSeconds = 0.0f;
    };
}
