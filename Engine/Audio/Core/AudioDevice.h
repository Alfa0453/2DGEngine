#pragma once

#include "../Types/AudioFormat.h"

#include "AudioRendererSource.h"

#include <cstddef>

namespace Engine
{
    class AudioDevice
    {
    public:

        virtual~AudioDevice() = default;

        virtual bool Initialize(const AudioFormat& requestedFormat, AudioRendererSource* renderSource) = 0;

        virtual void Shutdown() = 0;

        virtual bool IsInitialized() const = 0;

        virtual const AudioFormat& GetFormat() const = 0;
    };
}