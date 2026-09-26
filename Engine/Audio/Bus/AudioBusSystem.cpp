#include "AudioBusSystem.h"

#include "../Types/AudioLimits.h"
#include "AudioBusState.h"

#include <algorithm>
#include <cmath>


namespace Engine
{
    AudioBusSystem::AudioBusSystem()
    {
        Reset();
    }

    void AudioBusSystem::Reset()
    {
        for (AudioBusState& bus : m_Buses)
        {
            bus.CurrentVolume = 1.0f;

            bus.TargetVolume = 1.0f;

            bus.Muted = false;
        }
    }

    void AudioBusSystem::SetVolume(AudioBusID bus, float volume)
    {
        const std::size_t index = ToAudioBusIndex(bus);

        if (index >= m_Buses.size())
        {
            return;
        }

        m_Buses[index].TargetVolume = std::clamp(volume, AudioLimits::MinVolume, AudioLimits::MaxVolume);
    }

    float AudioBusSystem::GetTargetVolume(AudioBusID bus) const
    {
        const std::size_t index = ToAudioBusIndex(bus);

        if (index >= m_Buses.size())
        {
            return 1.0f;
        }

        return m_Buses[index].TargetVolume;
    }

    float AudioBusSystem::GetCurrentVolume(AudioBusID bus) const
    {
        const std::size_t index = ToAudioBusIndex(bus);

        if (index >= m_Buses.size())
        {
            return 1.0f;
        }

        return m_Buses[index].CurrentVolume;
    }

    void AudioBusSystem::SetMuted(AudioBusID bus, bool muted)
    {
        const std::size_t index = ToAudioBusIndex(bus);

        if (index >= m_Buses.size())
        {
            return;
        }

        m_Buses[index].Muted = muted;
    }

    bool AudioBusSystem::IsMuted(AudioBusID bus) const
    {
        const std::size_t index = ToAudioBusIndex(bus);

        if (index >= m_Buses.size())
        {
            return false;
        }

        return m_Buses[index].Muted;
    }

    float AudioBusSystem::GetEffectiveVolume(AudioBusID bus) const
    {
        const std::size_t busIndex = ToAudioBusIndex(bus);

        if (busIndex >= m_Buses.size())
        {
            return 1.0f;
        }

        const AudioBusState& busState = m_Buses[busIndex];

        if (busState.Muted)
        {
            return 0.0f;
        }

        if (bus == AudioBusID::Master)
        {
            return busState.CurrentVolume;
        }

        const AudioBusState& master = m_Buses[ToAudioBusIndex(AudioBusID::Master)];

        if (master.Muted)
        {
            return 0.0f;
        }

        return busState.CurrentVolume * master.CurrentVolume;
    }

    void AudioBusSystem::AdvanceSmoothing(std::size_t frameCount, std::uint32_t sampleRate)
    {
        if (frameCount == 0 || sampleRate == 0)
        {
            return;
        }

        constexpr float timeConstantSeconds = 0.05f;

        const float deltaTime = static_cast<float>(frameCount) / static_cast<float>(sampleRate);

        const float alpha = 1.0f - std::exp(-deltaTime / timeConstantSeconds);

        for (AudioBusState& bus : m_Buses)
        {
            bus.CurrentVolume += (bus.TargetVolume - bus.CurrentVolume) * alpha;
        }
    }

    void AudioBusSystem::SetVolumeImmediate(AudioBusID bus, float volume)
    {
        const std::size_t index = ToAudioBusIndex(bus);

        if (index >= m_Buses.size())
        {
            return;
        }

        const float clamped = std::clamp(volume, AudioLimits::MinVolume, AudioLimits::MaxVolume);

        m_Buses[index].CurrentVolume = clamped;

        m_Buses[index].TargetVolume = clamped;
    }
}
