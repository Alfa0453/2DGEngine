#include "EngineTime.h"

#include <SDL3/SDL.h>

namespace Engine
{
    void Time::Initialize()
    {
        m_PerformanceFrequency = static_cast<double>(SDL_GetPerformanceFrequency());

        m_PrevuiousCounter = SDL_GetPerformanceCounter();

        m_UnscaledDeltaTime = 0.0f;

        m_DeltaTime = 0.0f;

        m_ElapsedTime = 0.0f;

        m_FPS = 0.0f;

        m_TimeScale = 1.0f;
    }

    void Time::Update()
    {
        const std::uint64_t currentCounter = SDL_GetPerformanceCounter();

        const std::uint64_t counterDifference = currentCounter - m_PrevuiousCounter;

        m_PrevuiousCounter = currentCounter;

        m_UnscaledDeltaTime = static_cast<float>(static_cast<double>(counterDifference) / m_PerformanceFrequency);

        constexpr float maxDeltaTime = 0.1f;

        if (m_UnscaledDeltaTime > maxDeltaTime)
        {
            m_UnscaledDeltaTime = maxDeltaTime;
        }

        m_DeltaTime = m_UnscaledDeltaTime * m_TimeScale;

        m_ElapsedTime += m_DeltaTime;

        if (m_UnscaledDeltaTime > 0.0f)
        {
            m_FPS = 1.0f / m_UnscaledDeltaTime;
        }
    }

    float Time::GetDeltaTime() const
    {
        return m_DeltaTime;
    }

    float Time::GetUnscaledDeltaTime() const
    {
        return m_UnscaledDeltaTime;
    }

    float Time::GetElapsedTime() const
    {
        return m_ElapsedTime;
    }

    float Time::GetFPS() const
    {
        return m_FPS;
    }

    void Time::SetTimeScale(float scale)
    {
        if (scale < 0.0f)
        {
            scale = 0.0f;
        }

        m_TimeScale = scale;
    }

    float Time::GetTimeScale() const
    {
        return m_TimeScale;
    }
}