#include "AudioDebugReporter.h"

#include <iomanip>
#include <sstream>

namespace Engine
{
    std::string AudioDebugReporter::BuildSummary(const AudioStats& stats)
    {
        std::ostringstream stream;

        stream << std::fixed << std::setprecision(3);

        stream
            << "Audio Stats\n"
            << "-----------\n"

            << "Voices: "
            << stats.ActiveVoices
            << "\n"

            << "  Pending Start: "
            << stats.PendingStartVoices
            << "\n"

            << "  Pending Stop: "
            << stats.PendingStopVoices
            << "\n"

            << "  Paused: "
            << stats.PausedVoices
            << "\n\n"

            << "Buses\n"
            << "  Music: "
            << stats.ActiveMusicVoices
            << "\n"

            << "  SFX: "
            << stats.ActiveSFXVoices
            << "\n"

            << "  UI: "
            << stats.ActiveUIVoices
            << "\n"

            << "  Ambient: "
            << stats.ActiveAmbientVoices
            << "\n\n"

            << "Commands\n"
            << "  Pending: "
            << stats.PendingCommands
            << " / "
            << stats.CommandQueueCapacity
            << "\n"

            << "  Peak observed: "
            << stats.PeakPendingCommandsObserved
            << "\n"

            << "  Queue full: "
            << stats.CommandQueueFullCount
            << "\n\n"

            << "Playback Events\n"
            << "  Pending: "
            << stats.PendingPlaybackEvents
            << " / "
            << stats.PlaybackEventQueueCapacity
            << "\n"

            << "  Overflow: "
            << stats
                .PlaybackEventQueueOverflowCount
            << "\n\n"

            << "Streaming\n"
            << "  Underflows: "
            << stats.StreamUnderflows
            << "\n\n"

            << "Mixer\n"
            << "  Blocks total: "
            << stats.TotalBlocksMixed
            << "\n"

            << "  Frames total: "
            << stats.TotalFramesMixed
            << "\n"

            << "  Last render: "
            << stats.LastRenderMilliseconds
            << " ms\n"

            << "  Average render: "
            << stats.AverageRenderMilliseconds
            << " ms\n"

            << "  Maximum render: "
            << stats.MaximumRenderMilliseconds
            << " ms\n"

            << "  Budget: "
            << stats.RenderBudgetMilliseconds
            << " ms\n"

            << "  Last budget use: "
            << stats
                .LastRenderBudgetUsagePercent
            << "%\n"

            << "  Max budget use: "
            << stats
                .MaximumRenderBudgetUsagePercent
            << "%\n\n"

            << "Allocation\n"
            << "  Voice steals: "
            << stats.TotalVoiceSteals
            << "\n"

            << "  Play failures: "
            << stats.PlayFailuresNoVoice
            << "\n";

        return stream.str();
    }
}
