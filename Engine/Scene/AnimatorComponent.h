#pragma once

#include "Component.h"

#include "../Graphics/AnimationClip2D.h"
#include "../Graphics/AnimatorState2D.h"
#include "../Graphics/AnimatorParameter2D.h"
#include "../Graphics/AnimatorTransition2D.h"
#include "../Graphics/AnimationEvent2D.h"

#include <cstddef>
#include <string>
#include <vector>
#include <functional>
#include <unordered_map>


using AnimationEventCallback = std::function<void(const Engine::AnimationEvent2D&)>;

using StateCallback = std::function<void(const std::string&)>;

namespace Engine
{
    class SpriteRendererComponent;

    class AnimatorComponent : public Component
    {
    public:
        AnimatorComponent() = default;

        void Start() override;

        void Update(float deltaTime) override;

        void Play(const AnimationClip2D* clip, bool restartIfSame);

        void Pause();

        void Resume();

        void Stop();

        bool IsPlaying() const;

        bool IsPaused() const;

        const AnimationClip2D* GetCurrentClip() const;

        std::size_t GetCurrentFrameIndex() const;

        float GetPlaybackTime() const;

        void SetPlaybackSpeed(float speed);

        float GetPlaybackSpeed() const;

        bool AddState(const std::string& name, const AnimationClip2D* clip, float speedMultiplier = 1.0f);

        bool AddState(const AnimatorState2D& state);

        bool RemoveState(const std::string& name);

        bool HasState(const std::string& name) const;

        const AnimatorState2D* GetState(const std::string& name) const;

        bool Play(const std::string& stateName, bool restartIfSame = false);

        const std::string& GetCurrentStateName() const;

        bool AddBoolParameter(const std::string& name, bool defaultValue = false);

        bool AddIntParameter(const std::string& name, int defaultValue = 0);

        bool AddFloatParameter(const std::string&name, float defaultValue = 0.0f);

        bool AddTriggerParameter(const std::string& name);

        bool SetBool(const std::string& name, bool value);

        bool SetInt(const std::string& name, int value);

        bool SetFloat(const std::string& name, float value);

        bool SetTrigger(const std::string& name);

        bool ResetTrigger(const std::string& name);

        bool GetBool(const std::string& name, bool fallback = false) const;

        int GetInt(const std::string& name, int fallback = 0) const;

        float GetFloat(const std::string& name, float fallback = 0.0f) const;

        bool GetTrigger(const std::string& name) const;

        void AddTransition(AnimatorTransition2D transition);

        void ClearTransitions();

        float GetNormalizedTime() const;

        void SetStateEnterCallback(StateCallback callback);

        void SetStateExitCallback(StateCallback callback);

        const std::string& GetLastTransitionFrom() const;

        const std::string& GetLastTransitionTo() const;

        int GetLastTransitionPriority() const;

        void SetAnimationEventCallback(AnimationEventCallback callback);

        float GetClipNormalizedTime() const;

    private:
        void ApplyCurrentFrame();

        void AdvanceFrame();

        void PlayClipInternal(const AnimationClip2D* clip, bool restartIfSame);

        bool EvaluateTransitions();

        bool EvaluateCondition(const AnimatorCondition2D& condition) const;

        bool AreTransitionConditionsMet(const AnimatorTransition2D& transition) const;

        bool IsExitTimeSatisfied(const AnimatorTransition2D& transition) const;

        void ConsumeTransitionTriggers(const AnimatorTransition2D& transition);

        void SortTransitionsIfNeeded();

        bool ChangeState(const std::string& stateName, bool restartIfSame);

        void PublishAnimationEvent(const AnimationEvent2D& event);

        void ResetEventTracking();

        void DispatchAnimationEvents(float previousNormalized, float currentNormalized);

        void DispatchLoopingAnimationEvents(float previousNormalized, float currentNormalized);

    private:
        SpriteRendererComponent* m_SpriteRenderer = nullptr;

        std::unordered_map<std::string, AnimatorState2D> m_States;

        std::unordered_map<std::string, AnimatorParameter2D> m_Parameters;

        std::vector<AnimatorTransition2D> m_Transitions;

        std::string m_CurrentStateName;

        float m_CurrentStateSpeed = 1.0f;

        const AnimationClip2D* m_CurrentClip = nullptr;

        std::size_t m_CurrentFrameIndex = 0;

        float m_FrameTimer = 0.0f;

        float m_PlaybackTime = 0.0f;

        float m_PlaybackSpeed = 1.0f;

        bool m_IsPlaying = false;

        bool m_IsPaused = false;

        bool m_TransitionsDirty = false;

        float m_ClipLocalTime = 0.0f;

        StateCallback m_OnStateEnter;

        StateCallback m_OnStateExit;

        std::string m_LastTransitionTo;

        std::string m_LastTransitionFrom;

        int m_LastTransitionPriority = 0;

        AnimationEventCallback m_OnAnimationEvent;

        std::vector<bool> m_FiredEvents;
    };
}