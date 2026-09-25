#pragma once

#include <memory>
#include <string>

namespace Engine
{
    class AudioStreamDecoder;

    std::unique_ptr<AudioStreamDecoder> CreateAudioStreamDecoder(const std::string& filePath);
}
