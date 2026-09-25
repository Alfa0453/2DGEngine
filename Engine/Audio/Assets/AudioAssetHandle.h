#pragma once

#include <cstdint>
#include <type_traits>

namespace Engine
{
    struct AudioAssetHandle
    {
        std::uint32_t ID = 0;

        std::uint32_t Generation = 0;

        bool IsValid() const { return ID != 0; }

        explicit operator bool() const { return IsValid(); }

        bool operator==(const AudioAssetHandle& other) const { return ID == other.ID && Generation == other.Generation; }

        bool operator!=(const AudioAssetHandle& other) const { return !(*this == other); }
    };

    static_assert(std::is_trivially_copyable_v<AudioAssetHandle>, "AudioAssetHandle must be trivially copyable.");
}
