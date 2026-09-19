#include "AudioClip.h"

#include <SDL3/SDL.h>

#include <cstring>
#include <limits>
#include <utility>
#include <string>

namespace Engine
{
    AudioClip::AudioClip(std::string name, AudioFormat format, std::vector<float> samples)
        : m_Name(std::move(name)), m_Format(format), m_Samples(std::move(samples))
    {
    }

    const std::string& AudioClip::GetName() const
    {
        return m_Name;
    }

    const AudioFormat& AudioClip::GetFormat() const
    {
        return m_Format;
    }

    const std::vector<float>& AudioClip::GetSamples() const
    {
        return m_Samples;
    }

    std::size_t AudioClip::GetSampleCount() const
    {
        return m_Samples.size();
    }

    std::size_t AudioClip::GetFrameCount() const
    {
        if (m_Format.Channels == 0)
        {
            return 0;
        }

        return m_Samples.size() / static_cast<std::size_t>(m_Format.Channels);
    }

    float AudioClip::GetDurationSeconds() const
    {
        if (m_Format.SampleRate == 0)
        {
            return 0.0f;
        }

        return static_cast<float>(GetFrameCount()) / static_cast<float>(m_Format.SampleRate);
    }

    bool AudioClip::IsValid() const
    {
        if (m_Format.SampleRate == 0 || m_Format.Channels == 0 || m_Samples.empty())
        {
            return false;
        }

        return (m_Samples.size() % m_Format.Channels) == 0;
    }

    bool AudioClip::LoadFromWav(const std::string& filepath, const AudioFormat& targetFormat)
    {
        if (filepath.empty() || targetFormat.SampleRate == 0 || targetFormat.Channels == 0)
        {
            return false;
        }

        SDL_AudioSpec sourceSpec{};

        Uint8* sourceBuffer = nullptr;

        Uint32 sourceLength = 0;

        if (!SDL_LoadWAV(filepath.c_str(), &sourceSpec, &sourceBuffer, &sourceLength))
        {
            return false;
        }

        if (sourceLength > std::numeric_limits<Uint32>::max())
        {
            SDL_free(sourceBuffer);

            return false;
        }

        SDL_AudioSpec destinationSpec{};

        destinationSpec.format = SDL_AUDIO_F32;

        destinationSpec.channels = static_cast<int>(targetFormat.Channels);

        destinationSpec.freq = static_cast<int>(targetFormat.SampleRate);

        Uint8* convertedBuffer = nullptr;

        int convertedLength = 0;

        const bool converted = 
            SDL_ConvertAudioSamples(
                &sourceSpec,
                sourceBuffer,
                static_cast<int>(sourceLength),
                &destinationSpec,
                &convertedBuffer,
                &convertedLength
            );

        SDL_free(sourceBuffer);

        sourceBuffer = nullptr;

        if (!converted || !convertedBuffer || convertedLength <= 0)
        {
            if (convertedBuffer)
            {
                SDL_free(convertedBuffer);
            }

            return false;
        }

        if (convertedLength % static_cast<int>(sizeof(float)) != 0)
        {
            SDL_free(convertedBuffer);

            return false;
        }

        const std::size_t floatCount = static_cast<std::size_t>(convertedLength) / sizeof(float);

        m_Samples.resize(floatCount);

        std::memcpy(m_Samples.data(), convertedBuffer, static_cast<std::size_t>(convertedLength));

        SDL_free(convertedBuffer);

        convertedBuffer = nullptr;

        m_Format = targetFormat;

        m_Name = filepath;

        if (!IsValid())
        {
            m_Samples.clear();

            m_Name.clear();

            return false;
        }

        return true;
    }
}