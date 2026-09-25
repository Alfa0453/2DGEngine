#pragma once

#include "../Playback/AudioPlaybackHandle.h"
#include "../Playback/AudioPlaybackSettings.h"
#include "../Spatial/AudioListenerState.h"
#include "../Playback/AudioSourceKind.h"

#include <type_traits>

namespace Engine
{
    class AudioClip;
    class AudioStream;
    class AudioAssetRecord;

    enum class AudioCommandType
    {
        Play,
        ReplaceVoice,

        Stop,
        StopAll,

        Pause,
        Resume,

        SeekSeconds,

        SetVolume,
        SetPan,
        SetPitch,
        SetLooping,

        SetSourcePosition,
        SetSourceSpatialState,

        FadeTo,
        FadeOutAndStop,

        SetBusVolume,
        SetBusMute,

        SetListenerState
    };

    struct AudioCommand
    {
        AudioCommandType Type = AudioCommandType::StopAll;

        AudioPlaybackHandle Handle;

        AudioPlaybackHandle PreviousHandle;

        const AudioClip* Clip = nullptr;

        AudioSourceKind SourceKind = AudioSourceKind::None;

        AudioStream* Stream = nullptr;

        AudioAssetRecord* AssetRecord = nullptr;

        AudioPlaybackSettings PlaybackSettings;

        AudioBusID Bus = AudioBusID::SFX;

        AudioListenerState ListenerState;

        Vector2 SourcePosition{0.0f, 0.0f};

        Vector2 SourceVelocity{0.0f, 0.0f};

        float DurationSeconds = 0.0f;

        float Value = 0.0f;

        bool BoolValue = false;
    };

    static_assert(std::is_trivially_copyable_v<AudioCommand>, "AudioCommand must remain trivially copyable.");
}
