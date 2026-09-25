#pragma once

#include <cstdint>

namespace Engine
{
    enum class AudioStreamState : std::uint8_t
    {
        Closed = 0,
        Buffering,
        Playing,
        Seeking,
        Ended,
        Error
    };
}
