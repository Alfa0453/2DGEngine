#pragma once

#include <cstdint>



namespace Engine
{
    using EventSubscriptionID = std::uint64_t;

    inline constexpr EventSubscriptionID InvalidEventSubscriptionID = 0;
}