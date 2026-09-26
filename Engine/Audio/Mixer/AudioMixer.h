#pragma once

#include "../Types/AudioFormat.h"
#include "../Bus/AudioBusSystem.h"

#include "AudioMixCompletion.h"
#include "AudioMixVoiceResult.h"

#include <cstddef>
#include <vector>
#include <atomic>

namespace Engine
{
    class AudioVoice;
    class AudioPlaybackHandle;
    class AudioClip;
    class AudioStream;
    struct AudioListenerState;
    struct AudioSettings;

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

        bool Mix(std::vector<AudioVoice>& voices, const AudioBusSystem& busSystem, const AudioListenerState& listener, const AudioSettings& audioSettings, std::vector<AudioMixCompletion>& outCompletions);

        std::uint64_t GetStreamUnderflowCount() const;

    private:

        AudioMixVoiceResult MixVoice(AudioVoice& voice, float* output, std::size_t frameCount, float busGain, float spatialPan, float distanceGain, float dopplerFactor);

        AudioMixVoiceResult MixClipVoice(AudioVoice& voice, float* output, std::size_t frameCount, float busGain, float spatialPan, float distanceGain, float dopplerFactor);

        AudioMixVoiceResult MixStreamVoice(AudioVoice& voice, float* output, std::size_t frameCount, float busGain, float spatialPan, float distanceGain);

        float SampleChannelLinear(const AudioClip& clip, double playbackFrame, std::size_t channel, bool looping) const;

        void CalculateStereoPanGains(float pan, float& outLeftGain, float& outRightGain) const;

    private:

        AudioFormat m_Format;

        std::size_t m_FramesPerBlock = 0;

        std::vector<float> m_MixBuffer;

        std::vector<float> m_StreamScratch;

        bool m_Initialized = false;

        std::atomic<std::uint64_t> m_StreamUnderflowCount{0};
    };
}
