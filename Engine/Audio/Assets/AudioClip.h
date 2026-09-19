#pragma once

#include "../Types/AudioFormat.h"

#include <cstddef>
#include <string>
#include <vector>

namespace Engine
{
    class AudioClip
    {
    public:
        
        AudioClip() = default;

        AudioClip(std::string name, AudioFormat forma, std::vector<float> samples);

        const std::string& GetName() const;

        const AudioFormat& GetFormat() const;

        const std::vector<float>& GetSamples() const;

        std::size_t GetSampleCount() const;

        std::size_t GetFrameCount() const;

        float GetDurationSeconds() const;

        bool IsValid() const;

        bool LoadFromWav(const std::string& filepath, const AudioFormat& targetFormat);

    private:

        std::string m_Name;

        AudioFormat m_Format;

        std::vector<float> m_Samples;
    };
}