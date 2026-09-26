#include "AudioMixer.h"

#include "../Assets/AudioClip.h"
#include "../Playback/AudioVoice.h"
#include "../Playback/AudioPlaybackHandle.h"
#include "../Spatial/AudioListenerState.h"
#include "../Spatial/AudioSpatialization2D.h"
#include "../Types/AudioSettings.h"
#include "../Types/AudioLimits.h"
#include "../Streaming/AudioStream.h"

#include "AudioMixCompletion.h"
#include "AudioMixVoiceResult.h"

#include <algorithm>
#include <atomic>
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

        m_StreamScratch.assign(m_FramesPerBlock * channelCount, 0.0f);

        m_StreamUnderflowCount.store(0, std::memory_order_relaxed);

        m_Initialized = true;
    }

    void AudioMixer::Shutdown()
    {
        m_MixBuffer.clear();

        m_StreamScratch.clear();

        m_FramesPerBlock = 0;

        m_StreamUnderflowCount.store(0, std::memory_order_relaxed);

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

    bool AudioMixer::Mix(std::vector<AudioVoice>& voices, const AudioBusSystem& busSystem, const AudioListenerState& listener, const AudioSettings& audioSettings, std::vector<AudioMixCompletion>& outCompletions)
    {
        if (!m_Initialized || m_MixBuffer.empty())
        {
            return false;
        }

        outCompletions.clear();

        std::fill(m_MixBuffer.begin(), m_MixBuffer.end(), 0.0f);

        for (AudioVoice& voice : voices)
        {
            if (!voice.IsActive())
            {
                continue;
            }

            float spatialPan = 0.0f;

            float distanceGain = 1.0f;

            float dopplerFactor = 1.0f;

            if (voice.IsSpatial())
            {
                const AudioSpatialResult2D spatialResult = AudioSpatialization2D::Calculate(voice.GetSpatialPosition(), voice.GetSpatialVelocity(), listener, voice.GetSpatialPanDistance(), voice.GetSpatialPanStrength(), voice.GetMinDistance(), voice.GetMaxDistance(), voice.GetAttenuationStrength(), voice.GetAttenuationModel(), voice.IsDopplerEnabled(), voice.GetDopplerStrength(), audioSettings.SpeedOfSound, audioSettings.MinDopplerFactor, audioSettings.MaxDopplerFactor);

                spatialPan = spatialResult.Pan;

                distanceGain = spatialResult.DistanceGain;

                dopplerFactor = spatialResult.DopplerFactor;
            }

            const float busGain = busSystem.GetEffectiveVolume(voice.GetBus());

            const AudioMixVoiceResult result = MixVoice(voice, m_MixBuffer.data(), m_FramesPerBlock, busGain, spatialPan, distanceGain, dopplerFactor);

            if (result.EndReason == AudioMixVoiceEndReason::Finished)
            {
                outCompletions.push_back(AudioMixCompletion{result.FinishedHandle, AudioMixCompletionType::Finished});
            }
            else if (result.EndReason == AudioMixVoiceEndReason::FadeStopped)
            {
                outCompletions.push_back(AudioMixCompletion{result.FinishedHandle, AudioMixCompletionType::Stopped});
            }
        }

        for (float& sample : m_MixBuffer)
        {
            sample = std::clamp(sample, -1.0f, 1.0f);
        }

        return true;
    }

    AudioMixVoiceResult AudioMixer::MixVoice(AudioVoice& voice, float* output, std::size_t frameCount, float busGain, float spatialPan, float distanceGain, float dopplerFactor)
    {
        switch (voice.GetSourceKind())
        {
            case AudioSourceKind::Clip:
            {
                return MixClipVoice(voice, output, frameCount, busGain, spatialPan, distanceGain, dopplerFactor);
            }

            case AudioSourceKind::Stream:
            {
                return MixStreamVoice(voice, output, frameCount, busGain, spatialPan, distanceGain);
            }

            case AudioSourceKind::None:
            default:
            {
                return {};
            }
        }
    }

    AudioMixVoiceResult AudioMixer::MixClipVoice(AudioVoice &voice, float *output, std::size_t frameCount, float busGain, float spatialPan, float distanceGain, float dopplerFactor)
    {
        AudioMixVoiceResult result;

        if (!voice.IsActive() || !output)
        {
            return result;
        }

        const AudioClip* clip = voice.GetClip();

        if (!clip || !clip->IsValid())
        {
            result.EndReason = AudioMixVoiceEndReason::Finished;

            result.FinishedHandle = voice.GetHandle();

            voice.Stop();

            return result;
        }

        if (clip->GetFormat() != m_Format)
        {
            result.EndReason = AudioMixVoiceEndReason::Finished;

            result.FinishedHandle = voice.GetHandle();

            voice.Stop();

            return result;
        }

        const std::size_t totalFrames = clip->GetFrameCount();

        if (totalFrames == 0)
        {
            result.EndReason = AudioMixVoiceEndReason::Finished;

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

        float targetPitch = std::clamp(voice.GetPitch() * dopplerFactor , AudioLimits::MinPitch, AudioLimits::MaxPitch);

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
                    result.EndReason = AudioMixVoiceEndReason::Finished;

                    result.FinishedHandle = voice.GetHandle();

                    voice.Stop();

                    return result;
                }
            }

            float leftPanGain = 1.0f;

            float rightPanGain = 1.0f;

            const float fadeGain = voice.GetFadeGain();

            if (m_Format.Channels == 2)
            {
                CalculateStereoPanGains(currentPan, leftPanGain, rightPanGain);
            }

            if (m_Format.Channels == 2)
            {
                const float leftSample = SampleChannelLinear(*clip, playbackFrame, 0, voice.IsLooping());

                const float rightSample = SampleChannelLinear(*clip, playbackFrame, 1, voice.IsLooping());

                const std::size_t outputBase = outputFrame * 2;

                output[outputBase] += leftSample * currentVolume * fadeGain * busGain * leftPanGain * distanceGain;

                output[outputBase + 1] += rightSample * currentVolume * fadeGain * busGain * rightPanGain * distanceGain;
            }
            else {
                const std::size_t channelCount = static_cast<std::size_t>(m_Format.Channels);

                const std::size_t outputBase = outputFrame * channelCount;

                for (std::size_t channel = 0; channel < channelCount; ++channel)
                {
                    const float sample = SampleChannelLinear(*clip, playbackFrame, channel, voice.IsLooping());

                    output[outputBase + channel] += sample * currentVolume * fadeGain * busGain * distanceGain;
                }
            }

            const bool fadeCompleted = voice.AdvanceFade();

            if (fadeCompleted && voice.ShouldStopAfterFade())
            {
                result.EndReason = AudioMixVoiceEndReason::FadeStopped;

                result.FinishedHandle = voice.GetHandle();

                voice.Stop();

                return result;
            }

            playbackFrame += static_cast<double>(currentPitch);

            currentVolume += volumeStep;

            currentPan += panStep;

            currentPitch += pitchStep;
        }

        if (voice.IsActive())
        {
            voice.SetPlaybackFrame(playbackFrame);

            voice.SetCurrentVolume(targetVolume);

            voice.SetCurrentPan(targetPan);

            voice.SetCurrentPitch(targetPitch);
        }

        return result;
    }

    AudioMixVoiceResult AudioMixer::MixStreamVoice(AudioVoice &voice, float *output, std::size_t frameCount, float busGain, float spatialPan, float distanceGain)
    {
        AudioMixVoiceResult result;

        if (!voice.IsActive() || !output || frameCount == 0)
        {
            return result;
        }

        // A paused stream must not consume PCM.
        if (voice.IsPaused())
        {
            return result;
        }

        AudioStream* stream = voice.GetStream();

        if (!stream)
        {
            result.EndReason = AudioMixVoiceEndReason::Finished;

            result.FinishedHandle = voice.GetHandle();

            voice.Stop();

            return result;
        }

        const AudioStreamState state = stream->GetState();

        if (state == AudioStreamState::Error)
        {
            result.EndReason = AudioMixVoiceEndReason::Finished;

            result.FinishedHandle = voice.GetHandle();

            voice.Stop();

            return result;
        }

        // Initial buffering / seek:
        // m_MixBuffer was already cleared by Mix(), so returning here produces silence.
        if (state == AudioStreamState::Buffering || state == AudioStreamState::Seeking)
        {
            return result;
        }

        const std::size_t channelCount = static_cast<std::size_t>(m_Format.Channels);

        const std::size_t requiredSamples = frameCount * channelCount;

        // Never resize this from audio callback.
        if (m_StreamScratch.size() < requiredSamples)
        {
            return result;
        }

        const std::size_t framesRead = stream->ReadFrames(m_StreamScratch.data(), frameCount);

        // No frames available.
        if (framesRead == 0)
        {
            if (stream->HasCompletelyEnded())
            {
                result.EndReason = AudioMixVoiceEndReason::Finished;

                result.FinishedHandle = voice.GetHandle();

                voice.Stop();

                return result;
            }

            // Not EOF => underflow/starvation.
            if (state == AudioStreamState::Playing)
            {
                stream->RecordUnderflow();

                m_StreamUnderflowCount.fetch_add(1, std::memory_order_relaxed);
            }

            return result;
        }

        if (framesRead < frameCount && !stream->HasCompletelyEnded())
        {
            stream->RecordUnderflow();

            m_StreamUnderflowCount.fetch_add(1, std::memory_order_relaxed);
        }

        float currentVolume = voice.GetCurrentVolume();

        const float targetVolume = voice.GetVolume();

        float currentPan = voice.GetCurrentPan();

        const float targetPan = std::clamp(voice.GetPan() + spatialPan, -1.0f, 1.0f);

        const float inverseFramesRead = 1.0f / static_cast<float>(framesRead);

        const float volumeStep = (targetVolume - currentVolume) * inverseFramesRead;

        const float panStep = (targetPan - currentPan) * inverseFramesRead;

        for (std::size_t frame = 0; frame < framesRead; ++frame)
        {
            float leftPanGain = 1.0f;

            float rightPanGain = 1.0f;

            const float fadeGain = voice.GetFadeGain();

            if (m_Format.Channels == 2)
            {
                CalculateStereoPanGains(currentPan, leftPanGain, rightPanGain);
            }

            const std::size_t outputBase = frame * channelCount;

            if (m_Format.Channels == 2)
            {
                const float leftSample = m_StreamScratch[outputBase];

                const float rightSample = m_StreamScratch[outputBase + 1];

                output[outputBase] += leftSample * currentVolume * fadeGain * busGain * leftPanGain * distanceGain;

                output[outputBase + 1] += rightSample * currentVolume * fadeGain * busGain * rightPanGain * distanceGain;
            }
            else
            {
                for (std::size_t channel = 0; channel < channelCount; ++channel)
                {
                    const float sample = m_StreamScratch[outputBase + channel];

                    output[outputBase + channel] += sample * currentVolume * fadeGain * busGain * distanceGain;
                }
            }

            // Fade progresses only when actual stream PCM was consumed.
            const bool fadeCompleted = voice.AdvanceFade();

            if (fadeCompleted && voice.ShouldStopAfterFade())
            {
                result.EndReason = AudioMixVoiceEndReason::FadeStopped;

                result.FinishedHandle = voice.GetHandle();

                voice.Stop();

                return result;
            }

            currentVolume += volumeStep;

            currentPan += panStep;
        }

        if (voice.IsActive())
        {
            voice.SetCurrentVolume(targetVolume);

            voice.SetCurrentPan(targetPan);
        }

        // Decoder may have reached EOF before this block, while final buffered frames were just consumed.
        if (stream->HasCompletelyEnded())
        {
            result.EndReason = AudioMixVoiceEndReason::Finished;

            result.FinishedHandle = voice.GetHandle();

            voice.Stop();
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

    std::uint64_t AudioMixer::GetStreamUnderflowCount() const
    {
        return m_StreamUnderflowCount.load(std::memory_order_relaxed);
    }
}
