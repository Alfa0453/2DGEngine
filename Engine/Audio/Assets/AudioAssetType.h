#pragma once

#include <cstdint>

namespace Engine
{
    enum class AudioAssetType : std::uint8_t
    {
        None = 0,

        Clip,

        Stream
    };
}
