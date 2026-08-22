#pragma once

#include <cstdint>

namespace Engine
{
    enum class SortingLayer : std::int32_t
    {
        Background  = 0,
        Environment = 100,
        Characters  = 200,
        Effects     = 300,
        Foreground  = 400
    };
}