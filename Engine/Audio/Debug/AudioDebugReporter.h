#pragma once

#include "AudioStats.h"

#include <string>

namespace Engine
{
    class AudioDebugReporter
    {
    public:
        static std::string BuildSummary(const AudioStats& stats);
    };
}
