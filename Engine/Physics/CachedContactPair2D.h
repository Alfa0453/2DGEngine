#pragma once

#include "CachedContact2D.h"

#include <cstddef>

namespace Engine
{
    struct CachedContactPair2D
    {
        static constexpr std::size_t MaxContacts = 2;

        CachedContact2D Contacts[MaxContacts];

        std::size_t ContactCount = 0;
    };
}