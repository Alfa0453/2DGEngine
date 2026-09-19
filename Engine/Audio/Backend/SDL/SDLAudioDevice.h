#pragma once

#include "../../Core/AudioDevice.h"

#include <SDL3/SDL_audio.h>

struct SDL_AudioStream;

namespace Engine
{
    class SDLAudioDevice final : public AudioDevice
    {
    public:
        SDLAudioDevice() = default;

        ~SDLAudioDevice() override;

        bool Initialize(const AudioFormat& format, AudioRendererSource* renderSource) override;

        void Shutdown() override;

        bool IsInitialized() const override;

        const AudioFormat& GetFormat() const override;

    private:

        static void SDLCALL AudioStreamCallback(void* userdata, SDL_AudioStream* stream, int additionalAmount, int totalAmount);

        void HandleAudioRequest(SDL_AudioStream* stream, int additionalAmount, int totalAmount);

    private:

        SDL_AudioStream* m_Stream = nullptr;

        AudioRendererSource* m_RenderSource = nullptr;

        AudioFormat m_Format;

        bool m_Initialized = false;
    };
}