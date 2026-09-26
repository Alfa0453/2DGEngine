#pragma once

#include "AudioBusID.h"
#include "AudioBusState.h"
#include "AudioBusUtilities.h"

#include <array>

namespace Engine
{
    class AudioBusSystem
    {
    public:

        AudioBusSystem();

        void Reset();

        void SetVolume(AudioBusID bus, float volume);

        float GetTargetVolume(AudioBusID bus) const;

        float GetCurrentVolume(AudioBusID bus) const;

        void SetMuted(AudioBusID bus, bool muted);

        bool IsMuted(AudioBusID bus) const;

        float GetEffectiveVolume(AudioBusID bus) const;

        void AdvanceSmoothing(std::size_t frameCount, std::uint32_t sampleRate);

        void SetVolumeImmediate(AudioBusID bus, float volume);

    private:

        std::array<AudioBusState, GetAudioBusCount()> m_Buses;
    };
}
