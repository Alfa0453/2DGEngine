#pragma once

#include <cstddef>

namespace Engine
{
    class AudioRendererSource
    {
    public:

        virtual ~AudioRendererSource() = default;

        virtual bool RenderAudioBlock(const float*& outSamples, std::size_t& outFrameCount) = 0;
    };
}