#include "SDLAudioDevice.h"

#include <SDL3/SDL.h>

#include <limits.h>
#include <limits>

namespace Engine
{
    namespace
    {
        std::size_t GetBytesPerFrame(const AudioFormat& format)
        {
            return static_cast<std::size_t>(format.Channels) * sizeof(float);
        }
    }
    SDLAudioDevice::~SDLAudioDevice()
    {
        Shutdown();
    }

    bool SDLAudioDevice::Initialize(const AudioFormat& format, AudioRendererSource* renderSource)
    {
        if (m_Initialized)
        {
            return true;
        }

        if (!renderSource || format.SampleRate == 0 || format.Channels == 0)
        {
            return false;
        }

        if (format.SampleRate == 0 || format.Channels == 0)
        {
            return false;
        }

        SDL_AudioSpec spec{};

        spec.format = SDL_AUDIO_F32;

        spec.channels = static_cast<int>(format.Channels);

        spec.freq = static_cast<int>(format.SampleRate);

        m_RenderSource = renderSource;

        m_Format = format;

        m_Stream = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &spec, &SDLAudioDevice::AudioStreamCallback, this);

        if (!m_Stream)
        {
            m_RenderSource = nullptr;

            return false;
        }

        if (!SDL_ResumeAudioStreamDevice(m_Stream))
        {
            SDL_DestroyAudioStream(m_Stream);

            m_Stream = nullptr;

            m_RenderSource = nullptr;

            return false;
        }

        m_Initialized = true;

        return true;
    }

    void SDLAudioDevice::Shutdown()
    {
        if (!m_Stream)
        {
            m_RenderSource = nullptr;

            m_Initialized = false;

            return;
        }

        SDL_SetAudioStreamGetCallback(m_Stream, nullptr, nullptr);

        m_RenderSource = nullptr;

        SDL_DestroyAudioStream(m_Stream);

        m_Stream = nullptr;

        m_Initialized = false;
    }

    bool SDLAudioDevice::IsInitialized() const
    {
        return m_Initialized;
    }

    const AudioFormat& SDLAudioDevice::GetFormat() const
    {
        return m_Format;
    }

    void SDLCALL SDLAudioDevice::AudioStreamCallback(void* userdata, SDL_AudioStream* stream, int additionalAmount, int totalAmount)
    {
        if (!userdata)
        {
            return;
        }

        SDLAudioDevice* device = static_cast<SDLAudioDevice*>(userdata);

        device->HandleAudioRequest(stream, additionalAmount, totalAmount);
    }

    void SDLAudioDevice::HandleAudioRequest(SDL_AudioStream* stream, int additionalAmount, int totalAmount)
    {
        (void)totalAmount;

        if (!m_Initialized || !stream || !m_RenderSource || additionalAmount <= 0)
        {
            return;
        }

        const std::size_t bytesNeeded = static_cast<std::size_t>(additionalAmount);

        std::size_t bytesProvided = 0;

        while(bytesProvided < bytesNeeded)
        {
            const float* samples = nullptr;

            std::size_t frameCount = 0;

            if (!m_RenderSource->RenderAudioBlock(samples, frameCount))
            {
                break;
            }

            if (!samples || frameCount == 0)
            {
                break;
            }

            const std::size_t byteCount = frameCount * GetBytesPerFrame(m_Format);

            if (byteCount > static_cast<std::size_t>(std::numeric_limits<int>::max()))
            {
                break;
            }

            if (!SDL_PutAudioStreamData(stream, samples, static_cast<int>(byteCount)))
            {
                break;
            }

            bytesProvided += byteCount;
        }
    }
}