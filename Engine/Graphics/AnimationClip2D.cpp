#include "AnimationClip2D.h"

#include <algorithm>
#include <vector>

namespace Engine
{
    AnimationClip2D::AnimationClip2D(const std::string& name)
        : m_Name(name)
    {
    }

    void AnimationClip2D::SetName(const std::string& name)
    {
        m_Name = name;
    }

    const std::string& AnimationClip2D::GetName() const
    {
        return m_Name;
    }

    void AnimationClip2D::SetFramesPerSecond(float fps)
    {
        constexpr float MinFPS = 0.01f;

        m_FramesPerSecond = fps > MinFPS ? fps : MinFPS;
    }

    float AnimationClip2D::GetFramesPerSecond() const
    {
        return m_FramesPerSecond;
    }

    float AnimationClip2D::GetFrameDuration() const
    {
        return 1.0f / m_FramesPerSecond;
    }

    float AnimationClip2D::GetDuration() const
    {
        return static_cast<float>(m_Frames.size()) * GetFrameDuration();
    }

    void AnimationClip2D::SetLooping(bool looping)
    {
        m_Looping = looping;
    }

    bool AnimationClip2D::IsLooping() const
    {
        return m_Looping;
    }

    void AnimationClip2D::AddFrame(const AnimationFrame2D& frame)
    {
        m_Frames.push_back(frame);
    }

    void AnimationClip2D::AddFrame(Texture2D* texture, const SpriteRegion& region)
    {
        m_Frames.emplace_back(texture, region);
    }

    void AnimationClip2D::ClearFrames()
    {
        m_Frames.clear();
    }

    std::size_t AnimationClip2D::GetFrameCount() const
    {
        return m_Frames.size();
    }

    const AnimationFrame2D* AnimationClip2D::GetFrame(std::size_t index) const
    {
        if (index >= m_Frames.size())
        {
            return nullptr;
        }

        return &m_Frames[index];
    }

    bool AnimationClip2D::IsValid() const
    {
        return !m_Frames.empty() && m_FramesPerSecond > 0.0f;
    }

    void AnimationClip2D::AddEvent(const AnimationEvent2D& event)
    {
        if (!event.IsValid())
        {
            return;
        }

        m_Events.push_back(event);

        std::sort(m_Events.begin(), m_Events.end(),
            [](const AnimationEvent2D& a, const AnimationEvent2D& b)
            {
                return a.NormalizedTime < b.NormalizedTime;
            }
        );
    }

    bool AnimationClip2D::AddEvent(const std::string& name, float normalizedTime)
    {
        AnimationEvent2D event(name, normalizedTime);

        if (!event.IsValid())
        {
            return false;
        }

        AddEvent(event);

        return true;
    }

    void AnimationClip2D::ClearEvents()
    {
        m_Events.clear();
    }

    const std::vector<AnimationEvent2D>& AnimationClip2D::GetEvents() const
    {
        return m_Events;
    }
}