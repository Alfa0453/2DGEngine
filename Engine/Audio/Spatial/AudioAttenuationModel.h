#pragma once

#include <cstdint>

namespace Engine
{
    enum class AudioAttenuationModel : std::uint8_t
    {
        Linear = 0,
        Inverse
    };
}