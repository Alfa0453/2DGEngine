#pragma once

#include <cstddef>
#include <cstdint>
#include <atomic>

namespace Engine
{
    struct AudioStats
    {
        // Voice state

        std::size_t ActiveVoices = 0;

        std::size_t PendingStartVoices = 0;

        std::size_t PendingStopVoices = 0;

        std::size_t PausedVoices = 0;

        // Playback events this game update

        std::size_t StartedVoicesThisUpdate = 0;

        std::size_t FinishedVoicesThisUpdate = 0;

        std::size_t StoppedVoicesThisUpdate = 0;

        std::size_t FailedStartsThisUpdate = 0;

        // Command processing

        std::size_t PendingCommands = 0;

        std::size_t CommandQueueCapacity = 0;

        std::size_t PeakPendingCommandsObserved = 0;

        std::size_t CommandsProcessedThisUpdate = 0;

        std::uint64_t TotalCommandsProcessed = 0;

        std::size_t CommandQueueFullCount = 0;

        // Playback event queue

        std::size_t PendingPlaybackEvents = 0;

        std::size_t PlaybackEventQueueCapacity = 0;

        std::size_t PeakPendingPlaybackEventsObserved = 0;

        std::uint64_t PlaybackEventQueueOverflowCount = 0;

        // Mixer

        std::size_t BlocksMixedThisUpdate = 0;

        std::size_t FramesMixedThisUpdate = 0;

        std::uint64_t TotalBlocksMixed = 0;

        std::uint64_t TotalFramesMixed = 0;

        std::size_t StreamUnderflows = 0;

        // Voice allocation

        std::uint64_t TotalVoiceSteals = 0;

        std::uint64_t PlayFailuresNoVoice = 0;

        // Voices by bus

        std::size_t ActiveMusicVoices = 0;

        std::size_t ActiveSFXVoices = 0;

        std::size_t ActiveUIVoices = 0;

        std::size_t ActiveAmbientVoices = 0;

        // Audio-thread profiling

        double LastRenderMilliseconds = 0.0;

        double MaximumRenderMilliseconds = 0.0;

        double AverageRenderMilliseconds = 0.0;

        double RenderBudgetMilliseconds = 0.0;

        double LastRenderBudgetUsagePercent = 0.0;

        double MaximumRenderBudgetUsagePercent = 0.0;

        std::uint64_t RenderCallCount = 0;

        std::uint64_t RenderFailureCount = 0;
    };
}
