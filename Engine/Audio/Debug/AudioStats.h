#pragma once

#include <cstddef>
#include <cstdint>

namespace Engine
{
    struct AudioStats
    {
        std::size_t ActiveVoices = 0;

        std::size_t PendingCommands = 0;

        std::size_t CommandsProcessedThisUpdate = 0;

        std::uint64_t TotalCommandsProcessed = 0;

        std::size_t FinishedVoicesThisUpdate = 0;

        std::size_t BlocksMixedThisUpdate = 0;

        std::size_t FramesMixedThisUpdate = 0;

        std::uint64_t TotalBlocksMixed = 0;

        std::uint64_t TotalFramesMixed = 0;

        std::size_t PendingPlaybackEvents = 0;

        std::uint64_t CommandQueueFullCount = 0;

        std::uint64_t PlaybackEventQueueOverflowCount = 0;

        std::uint64_t TotalVoiceSteals = 0;

        std::uint64_t PlayFailuresNoVoice = 0;

        std::size_t ActiveMusicVoices = 0;

        std::size_t ActiveSFXVoices = 0;

        std::size_t ActiveUIVoices = 0;

        std::size_t ActiveAmbienVoices = 0;
    };
}