#pragma once

#include <cstdint>

namespace Engine
{
    enum class AudioBusID : std::uint8_t
    {
        Master = 0,

        Music,

        SFX,

        UI,

        Ambient,

        Count
    };
}