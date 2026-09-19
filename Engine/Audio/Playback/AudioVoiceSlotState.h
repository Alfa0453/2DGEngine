#pragma once

namespace Engine
{
    enum class AudioVoiceSlotState
    {
        Free,
        PendingStart,
        Active,
        PendingStop
    };
}