#pragma once

#include "../Types/AudioFormat.h"
#include "../Bus/AudioBusSystem.h"

#include "AudioMixVoiceResult.h"

#include <cstddef>
#include <vector>

namespace Engine
{
    class AudioVoice;
    class AudioPlaybackHandle;
    class AudioClip;
    struct AudioListenerState;

    class AudioMixer
    {
    public:

        AudioMixer() = default;

        void Initialize(const AudioFormat& format, std::size_t framePerBlock);

        void Shutdown();

        bool IsInitialized() const;

        const AudioFormat& GetFormat() const;

        std::size_t GetFramesPerBlock() const;

        const std::vector<float>& GetMixBuffer() const;

        bool Mix(std::vector<AudioVoice>& voices, const AudioBusSystem& busSystem, const AudioListenerState& listener, std::vector<AudioPlaybackHandle>& outFinishedVoices);

    private:

        AudioMixVoiceResult MixVoice(AudioVoice& voice, float* output, std::size_t frameCount, float busGain, float spatialPan, float distanceGain);

        float SampleChannelLinear(const AudioClip& clip, double playbackFrame, std::size_t channel, bool looping) const;

        void CalculateStereoPanGains(float pan, float& outLeftGain, float& outRightGain) const;

    private:

        AudioFormat m_Format;

        std::size_t m_FramesPerBlock = 0;

        std::vector<float> m_MixBuffer;

        bool m_Initialized = false;
    };
}