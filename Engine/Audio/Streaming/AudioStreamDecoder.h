#pragma once

#include "../Types/AudioFormat.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace Engine
{
    class AudioStreamDecoder
    {
    public:

        virtual ~AudioStreamDecoder() = default;

        virtual bool Open(const std::string& filePath, const AudioFormat& targetFormat) = 0;

        virtual void Close() = 0;

        virtual bool IsOpen() const = 0;

        virtual const AudioFormat& GetFormat() const = 0;

        virtual std::uint64_t GetTotalFrameCount() const = 0;

        virtual std::uint64_t GetCurrentFrame() const = 0;

        virtual std::uint64_t DecodeFrames(float* output, std::size_t maxFrames) = 0;

        virtual bool SeekFrame(std::uint64_t frame) = 0;
    };
}
