#pragma once

#include <cstdint>

namespace Engine
{
    struct AudioPlaybackHandle
    {
        std::uint32_t ID = 0;

        std::uint32_t Generation = 0;

        bool IsValid() const
        {
            return ID != 0;
        }

        bool operator==(const AudioPlaybackHandle& other) const
        {
            return ID == other.ID && Generation == other.Generation;
        }

        bool operator!=(const AudioPlaybackHandle& other) const
        {
            return !(*this == other);
        }
    };
}