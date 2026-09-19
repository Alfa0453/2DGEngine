#pragma once

#include "../../Math/Vector2.h"

#include <type_traits>

namespace Engine
{
    struct AudioListenerState
    {
        Vector2 Position{0.0f, 0.0f};

        Vector2 Forward{1.0f, 0.0f};

        Vector2 Velocity{0.0f, 0.0f};

        bool Enabled = true;
    };

    static_assert(std::is_trivially_copyable_v<AudioListenerState>, "AudioListenerState must remain trivially copyable.");
}