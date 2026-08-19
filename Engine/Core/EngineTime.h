#pragma once

#include <cstdint>

namespace Engine
{
    class Time
    {
    public:
        void Initialize();

        void Update();

        float GetDeltaTime() const;

        float GetUnscaledDeltaTime() const;

        float GetElapsedTime() const;

        float GetFPS() const;

        void SetTimeScale(float scale);

        float GetTimeScale() const;

    private:
        std::uint64_t m_PrevuiousCounter = 0;

        double m_PerformanceFrequency = 0.0;

        float m_UnscaledDeltaTime = 0.0f;

        float m_DeltaTime = 0.0f;

        float m_ElapsedTime = 0.0f;

        float m_FPS = 0.0f;

        float m_TimeScale = 1.0f;
    };
}