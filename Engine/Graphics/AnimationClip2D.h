#pragma once

#include "AnimationFrame2D.h"
#include "AnimationEvent2D.h"

#include <cstddef>
#include <string>
#include <vector>

namespace Engine
{
    class AnimationClip2D
    {
    public:
        AnimationClip2D() = default;

        explicit AnimationClip2D(const std::string& name);

        void SetName(const std::string& name);

        const std::string& GetName() const;

        void SetFramesPerSecond(float fps);

        float GetFramesPerSecond() const;

        float GetFrameDuration() const;

        float GetDuration() const;

        void SetLooping(bool looping);

        bool IsLooping() const;

        void AddFrame(const AnimationFrame2D& frame);

        void AddFrame(Texture2D* texture, const SpriteRegion& region);

        void ClearFrames();

        std::size_t GetFrameCount() const;

        const AnimationFrame2D* GetFrame(std::size_t index) const;

        bool IsValid() const;

        void AddEvent(const AnimationEvent2D& event);

        bool AddEvent(const std::string& name, float normalizedTime);

        void ClearEvents();

        const std::vector<AnimationEvent2D>& GetEvents() const;

    private:
        std::string m_Name;

        std::vector<AnimationFrame2D> m_Frames;

        std::vector<AnimationEvent2D> m_Events;

        float m_FramesPerSecond = 12.0f;

        bool m_Looping = true;
    };
}