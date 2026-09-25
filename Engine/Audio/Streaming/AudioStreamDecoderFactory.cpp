#include "AudioStreamDecoderFactory.h"

#include "WavStreamDecoder.h"

#include <algorithm>
#include <filesystem>
#include <cctype>

namespace Engine
{
    std::unique_ptr<AudioStreamDecoder> CreateAudioStreamDecoder(const std::string& filePath)
    {
        std::string extension = std::filesystem::path(filePath).extension().string();

        std::transform(extension.begin(), extension.end(), extension.begin(),
            [](unsigned char c) { return std::tolower(c); }
        );

        if (extension == ".wav")
        {
            return std::make_unique<WavStreamDecoder>();
        }

        return nullptr;
    }
}
