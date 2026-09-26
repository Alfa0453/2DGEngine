#pragma once

#include <cstdint>

namespace Engine
{
    struct AudioFormat
    {
        std::uint32_t SampleRate = 48000;

        std::uint16_t Channels = 2;

        bool operator==(const AudioFormat& other) const
        {
            return SampleRate == other.SampleRate && Channels == other.Channels;
        }

        bool operator!=(const AudioFormat& other) const
        {
            return !(*this == other);
        }
    };
}
