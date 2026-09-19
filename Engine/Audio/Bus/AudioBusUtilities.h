#pragma once

#include "AudioBusID.h"

#include <cstddef>

namespace Engine
{
    constexpr std::size_t GetAudioBusCount()
    {
        return static_cast<std::size_t>(AudioBusID::Count);
    }

    constexpr std::size_t ToAudioBusIndex(AudioBusID bus)
    {
        return static_cast<std::size_t>(bus);
    }
}