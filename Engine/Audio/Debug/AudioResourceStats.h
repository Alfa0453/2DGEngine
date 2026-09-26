#pragma once

#include <cstddef>
#include <cstdint>

namespace Engine
{
    struct AudioResourceStats
    {
        std::size_t LoadedAssets = 0;

        std::size_t LoadedClips = 0;

        std::size_t LoadedStreams = 0;

        std::size_t PendingUnloadAssets = 0;

        std::uint64_t TotalPlaybackReferences = 0;

        std::size_t ActiveStreamConsumers = 0;
    };
}
