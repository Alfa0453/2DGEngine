#pragma once

#include <cstdint>

namespace Engine
{
    using EntityID = std::uint64_t;

    using EntityGeneration = std::uint32_t;

    constexpr EntityID InvalidEntityID = 0;

    constexpr EntityGeneration InvalidEntityGeneration = 0;
}