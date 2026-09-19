#include "AudioMixer.h"

#include "../Assets/AudioClip.h"
#include "../Playback/AudioVoice.h"
#include "../Playback/AudioPlaybackHandle.h"
#include "../Spatial/AudioListenerState.h"
#include "../Spatial/AudioSpatialization2D.h"

#include "AudioMixVoiceResult.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace Engine
{
    void AudioMixer::Initialize(const AudioFormat& format, std::size_t framesPerBlock)
    {
        if (format.SampleRate == 0 || format.Channels == 0)
        {
            Shutdown();

            return;
        }

        m_Format = format;

        m_FramesPerBlock = std::max<std::size_t>(1, framesPerBlock);

        const std::size_t channelCount = static_cast<std::size_t>(m_Format.Channels);

        m_MixBuffer.assign(m_FramesPerBlock * channelCount, 0.0f);

        m_Initialized = true;
    }

    void AudioMixer::Shutdown()
    {
        m_MixBuffer.clear();

        m_FramesPerBlock = 0;

        m_Initialized = false;
    }

    bool AudioMixer::IsInitialized() const
    {
        return m_Initialized;
    }

    const AudioFormat& AudioMixer::GetFormat() const
    {
        return m_Format;
    }

    std::size_t AudioMixer::GetFramesPerBlock() const
    {
        return m_FramesPerBlock;
    }

    const std::vector<float>& AudioMixer::GetMixBuffer() const
    {
        return m_MixBuffer;
    }

    bool AudioMixer::Mix(std::vector<AudioVoice>& voices, const AudioBusSystem& busSystem, const AudioListenerState& listener, std::vector<AudioPlaybackHandle>& outFinishedVoices)
    {
        if (!m_Initialized || m_MixBuffer.empty())
        {
            return false;
        }

        outFinishedVoices.clear();

        std::fill(m_MixBuffer.begin(), m_MixBuffer.end(), 0.0f);

        for (AudioVoice& voice : voices)
        {
            if (!voice.IsActive())
            {
                continue;
            }

            float spatialPan = 0.0f;

            float distanceGain = 1.0f;

            if (voice.IsSpatial())
            {
                const AudioSpatialResult2D spatialResult = AudioSpatialization2D::Calculate(voice.GetSpatialPosition(), listener, voice.GetSpatialPanDistance(), voice.GetSpatialPanDistance(), voice.GetMinDistance(), voice.GetMaxDistance(), voice.GetAttenuationStrength(), voice.GetAttenuationModel());

                spatialPan = spatialResult.Pan;

                distanceGain = spatialResult.DistanceGain;
            }

            const float busGain = busSystem.GetEffectiveVolume(voice.GetBus());

            const AudioMixVoiceResult result = MixVoice(voice, m_MixBuffer.data(), m_FramesPerBlock, busGain, spatialPan);

            if (result.Finished && result.FinishedHandle.IsValid())
            {
                outFinishedVoices.push_back(result.FinishedHandle);
            }
        }

        for (float& sample : m_MixBuffer)
        {
            sample = std::clamp(sample, -1.0f, 1.0f);
        }

        return true;
    }

    AudioMixVoiceResult AudioMixer::MixVoice(AudioVoice& voice, float* output, std::size_t frameCount, float busGain, float spatialPan, float distanceGain)
    {
        AudioMixVoiceResult result;

        if (!voice.IsActive() || !output)
        {
            return result;
        }

        const AudioClip* clip = voice.GetClip();

        if (!clip || !clip->IsValid())
        {
            result.Finished = true;

            result.FinishedHandle = voice.GetHandle();

            voice.Stop();

            return result;
        }

        if (clip->GetFormat() != m_Format)
        {
            result.Finished = true;

            result.FinishedHandle = voice.GetHandle();

            voice.Stop();

            return result;
        }

        const std::size_t totalFrames = clip->GetFrameCount();

        if (totalFrames == 0)
        {
            result.Finished = true;

            result.FinishedHandle = voice.GetHandle();

            voice.Stop();

            return result;
        }

        double playbackFrame = voice.GetPlaybackFrame();

        float currentVolume = voice.GetCurrentVolume();

        const float targetVolume = voice.GetVolume();

        float currentPan = voice.GetCurrentPan();

        const float targetPan = std::clamp(voice.GetPan() + spatialPan, -1.0f, 1.0f);

        float currentPitch = voice.GetCurrentPitch();

        float targetPitch = voice.GetPitch();

        const float inverseFrameCount = 1.0f / static_cast<float>(frameCount);

        const float volumeStep = (targetVolume - currentVolume) * inverseFrameCount;

        const float panStep = (targetPan - currentPan) * inverseFrameCount;

        const float pitchStep = (targetPitch - currentPitch) * inverseFrameCount;

        for (std::size_t outputFrame = 0; outputFrame < frameCount; ++outputFrame)
        {
            while (playbackFrame >= static_cast<double>(totalFrames))
            {
                if (voice.IsLooping())
                {
                    playbackFrame -= static_cast<double>(totalFrames);
                }
                else 
                {
                    result.Finished = true;

                    result.FinishedHandle = voice.GetHandle();

                    voice.Stop();

                    return result;
                }
            }

            float leftPanGain = 1.0f;

            float rightPanGain = 1.0f;

            if (m_Format.Channels == 2)
            {
                CalculateStereoPanGains(currentPan, leftPanGain, rightPanGain);
            }

            if (m_Format.Channels == 2)
            {
                const float leftSample = SampleChannelLinear(*clip, playbackFrame, 0, voice.IsLooping());

                const float rightSample = SampleChannelLinear(*clip, playbackFrame, 1, voice.IsLooping());

                const std::size_t outputBase = outputFrame * 2;

                output[outputBase] += leftSample * currentVolume * busGain * leftPanGain * distanceGain;

                output[outputBase + 1] += rightSample * currentVolume * busGain * rightPanGain * distanceGain;
            }
            else {
                const std::size_t channelCount = static_cast<std::size_t>(m_Format.Channels);

                const std::size_t outputBase = outputFrame * channelCount;

                for (std::size_t channel = 0; channel < channelCount; ++channel)
                {
                    const float sample = SampleChannelLinear(*clip, playbackFrame, channel, voice.IsLooping());

                    output[outputBase + channel] += sample * currentVolume * busGain * distanceGain;
                }
            }

            playbackFrame += static_cast<double>(currentPitch);

            currentVolume += volumeStep;

            currentPan += panStep;

            currentPitch += pitchStep;
        }

        if (voice.IsActive())
        {
            voice.SetPlaybackFrame(playbackFrame);

            voice.SetCurrenVolume(targetVolume);

            voice.SetCurrentPan(targetPan);

            voice.SetCurrentPitch(targetPitch);
        }

        return result;
    }

    float AudioMixer::SampleChannelLinear(const AudioClip& clip, double playbackFrame, std::size_t channel, bool looping) const
    {
        const std::size_t totalFrames = clip.GetFrameCount();

        const std::size_t channelCount = static_cast<std::size_t>(clip.GetFormat().Channels);

        if (totalFrames == 0 || channel >= channelCount)
        {
            return 0.0f;
        }

        const std::size_t frameA = static_cast<std::size_t>(playbackFrame);

        std::size_t frameB = frameA + 1;

        if (frameB >= totalFrames)
        {
            if (looping)
            {
                frameB = 0;
            }
            else
            {
                frameB = frameA;
            }
        }

        const double fraction = playbackFrame - static_cast<double>(frameA);

        const std::vector<float>& samples = clip.GetSamples();

        const float sampleA = samples[frameA * channelCount + channel];

        const float sampleB = samples[frameB * channelCount + channel];

        return sampleA + (sampleB - sampleA) * static_cast<float>(fraction);
    }

    void AudioMixer::CalculateStereoPanGains(float pan, float& outLeftGain, float& outRightGain) const
    {
        pan = std::clamp(pan, -1.0f, 1.0f);

        constexpr float HalfPi = 1.57079632679f;

        const float normalized = (pan + 1.0f) * 0.5f;

        const float angle = normalized * HalfPi;

        outLeftGain = std::cos(angle);

        outRightGain = std::sin(angle);
    }
}