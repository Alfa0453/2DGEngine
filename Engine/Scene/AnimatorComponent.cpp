#include "AnimatorComponent.h"
#include "Entity.h"
#include "Scene.h"
#include "SpriteRendererComponent.h"
#include "../Graphics/AnimationNotifyEvent.h"

#include <algorithm>
#include <utility>

namespace Engine
{
    void AnimatorComponent::Start()
    {
        Entity* owner = GetOwner();

        if (!owner)
        {
            return;
        }

        m_SpriteRenderer = owner->GetComponent<SpriteRendererComponent>();

        if (m_CurrentClip && m_CurrentClip->IsValid())
        {
            ApplyCurrentFrame();
        }
    }

    void AnimatorComponent::Play(const AnimationClip2D* clip, bool restartIfSame)
    {
        m_CurrentStateName.clear();

        m_CurrentStateSpeed = 1.0f;

        PlayClipInternal(clip, restartIfSame);
    }

    void AnimatorComponent::ApplyCurrentFrame()
    {
        if (!m_SpriteRenderer || !m_CurrentClip)
        {
            return;
        }

        const AnimationFrame2D* frame = m_CurrentClip->GetFrame(m_CurrentFrameIndex);

        if (!frame)
        {
            return;
        }

        m_SpriteRenderer->SetSpriteFrame(frame->SpriteFrame);
    }

    void AnimatorComponent::AdvanceFrame()
    {
        if (!m_CurrentClip || m_CurrentClip->GetFrameCount() == 0)
        {
            Stop();
            return;
        }

        ++m_CurrentFrameIndex;

        if (m_CurrentFrameIndex >= m_CurrentClip->GetFrameCount())
        {
            if (m_CurrentClip->IsLooping())
            {
                m_CurrentFrameIndex = 0;
            }
            else 
            {
                m_CurrentFrameIndex = m_CurrentClip->GetFrameCount() - 1;

                m_IsPlaying = false;

                m_IsPaused = false;
            }
        }

        ApplyCurrentFrame();
    }

    void AnimatorComponent::Update(float deltaTime)
    {
        // Check transitions before update
        EvaluateTransitions();

        // Validate current playback state

        if (!m_IsPlaying || m_IsPaused || !m_CurrentClip || !m_CurrentClip->IsValid())
        {
            // Important for non-looping clips:
            // Attack may have just finished, so exit-time transitions can stiil fire.
            EvaluateTransitions();

            return;
        }

        if (m_PlaybackSpeed <= 0.0f)
        {
            return;
        }

        // Save previous event position
        const float previousNormalized = GetClipNormalizedTime();

        // Calculate scaled delta
        const float scaledDelta = deltaTime * m_PlaybackSpeed * m_CurrentStateSpeed;

        // Advance timers
        m_FrameTimer += scaledDelta;

        m_PlaybackTime += scaledDelta;

        m_ClipLocalTime += scaledDelta;

        // Handle looping local time
        const float clipDuration = m_CurrentClip->GetDuration();

        bool wrapped = false;

        if (m_CurrentClip->IsLooping() && clipDuration > 0.0f)
        {
            while (m_ClipLocalTime >= clipDuration) 
            {
                m_ClipLocalTime -= clipDuration;

                wrapped = true;
            }
        }

        // Advance animation frames
        const float frameDuration = m_CurrentClip->GetFrameDuration();

        while (m_FrameTimer >= frameDuration && m_IsPlaying)
        {
            m_FrameTimer -= frameDuration;

            AdvanceFrame();
        }

        // Get current normalized time
        const float currentNormalized = GetClipNormalizedTime();

        // Dispatch animation events
        if (wrapped)
        {
            DispatchLoopingAnimationEvents(previousNormalized, currentNormalized);
        }
        else 
        {
            DispatchAnimationEvents(previousNormalized, currentNormalized);
        }

        // Check transitions again
        EvaluateTransitions();
    }

    const std::string& AnimatorComponent::GetCurrentStateName() const
    {
        return m_CurrentStateName;
    }

    void AnimatorComponent::Pause()
    {
        if (m_IsPlaying)
        {
            m_IsPaused = true;
        }
    }

    void AnimatorComponent::Resume()
    {
        if (m_IsPlaying)
        {
            m_IsPaused = false;
        }
    }

    void AnimatorComponent::Stop()
    {
        if (!m_CurrentStateName.empty() && m_OnStateExit)
        {
            m_OnStateExit(m_CurrentStateName);
        }

        m_CurrentClip = nullptr;

        m_CurrentFrameIndex = 0;

        m_FrameTimer = 0.0f;

        m_PlaybackTime = 0.0f;

        m_IsPlaying = false;

        m_IsPaused = false;

        m_CurrentStateName.clear();

        m_CurrentStateSpeed = 1.0f;
    }

    void AnimatorComponent::SetPlaybackSpeed(float speed)
    {
        m_PlaybackSpeed = speed >= 0.0f ? speed : 0.0f;
    }

    float AnimatorComponent::GetPlaybackSpeed() const
    {
        return  m_PlaybackSpeed;;
    }

    bool AnimatorComponent::IsPlaying() const
    {
        return m_IsPlaying;
    }

    bool AnimatorComponent::IsPaused() const
    {
        return m_IsPaused;
    }

    const AnimationClip2D* AnimatorComponent::GetCurrentClip() const
    {
        return m_CurrentClip;
    }

    std::size_t AnimatorComponent::GetCurrentFrameIndex() const
    {
        return m_CurrentFrameIndex;
    }

    float AnimatorComponent::GetPlaybackTime() const
    {
        return m_PlaybackTime;
    }

    bool AnimatorComponent::AddState(const std::string& name, const AnimationClip2D* clip, float speedMultiplier)
    {
        return AddState(AnimatorState2D(name, clip, speedMultiplier));
    }

    bool AnimatorComponent::AddState(const AnimatorState2D& state)
    {
        if (!state.IsValid())
        {
            return false;
        }

        auto [iterator, inserted] = m_States.emplace(state.Name, state);

        return inserted;
    }

    bool AnimatorComponent::HasState(const std::string& name) const
    {
        return m_States.find(name) != m_States.end();
    }

    const AnimatorState2D* AnimatorComponent::GetState(const std::string& name) const
    {
        auto iterator = m_States.find(name);

        if (iterator == m_States.end())
        {
            return nullptr;
        }

        return &iterator->second;
    }

    bool AnimatorComponent::Play(const std::string& stateName, bool restartIfSame)
    {
        return ChangeState(stateName, restartIfSame);
    }

    bool AnimatorComponent::RemoveState(const std::string& name)
    {
        auto iterator = m_States.find(name);

        if (iterator == m_States.end())
        {
            return false;
        }

        const bool removingCurrent = m_CurrentStateName == name;

        m_States.erase(iterator);

        if (removingCurrent)
        {
            Stop();
        }

        return true;
    }

    void AnimatorComponent::PlayClipInternal(const AnimationClip2D* clip, bool restartIfSame)
    {
        if (!clip || !clip->IsValid())
        {
            Stop();
            return;
        }

        if (m_CurrentClip == clip && m_IsPlaying && !restartIfSame)
        {
            return;
        }

        m_CurrentClip = clip;

        m_CurrentFrameIndex = 0;

        m_FrameTimer = 0.0f;

        m_PlaybackTime = 0.0f;

        m_IsPlaying = true;

        m_IsPaused = false;

        m_ClipLocalTime = 0.0f;

        ResetEventTracking();

        ApplyCurrentFrame();
    }

    bool AnimatorComponent::AddIntParameter(const std::string& name, int defaultValue)
    {
        if (name.empty() || m_Parameters.find(name) != m_Parameters.end())
        {
            return false;
        }

        AnimatorParameter2D parameter;

        parameter.Name = name;

        parameter.Type = AnimatorParameterType::Int;

        parameter.IntValue = defaultValue;

        m_Parameters.emplace(name, parameter);

        return true;
    }

    bool AnimatorComponent::AddFloatParameter(const std::string& name, float defaultValue)
    {
        if (name.empty() || m_Parameters.find(name) != m_Parameters.end())
        {
            return false;
        }

        AnimatorParameter2D parameter;

        parameter.Name = name;

        parameter.Type = AnimatorParameterType::Float;

        parameter.FloatValue = defaultValue;

        m_Parameters.emplace(name, parameter);

        return true;
    }

    bool AnimatorComponent::AddBoolParameter(const std::string& name, bool defaultValue)
    {
        if (name.empty() || m_Parameters.find(name) != m_Parameters.end())
        {
            return false;
        }

        AnimatorParameter2D parameter;

        parameter.Name = name;

        parameter.Type = AnimatorParameterType::Bool;

        parameter.BoolValue = defaultValue;

        m_Parameters.emplace(name, parameter);

        return true;
    }

    bool AnimatorComponent::AddTriggerParameter(const std::string& name)
    {
        if (name.empty() || m_Parameters.find(name) != m_Parameters.end())
        {
            return false;
        }

        AnimatorParameter2D parameter;

        parameter.Name = name;

        parameter.Type = AnimatorParameterType::Trigger;

        parameter.FloatValue = false;

        m_Parameters.emplace(name, parameter);

        return true;
    }

    bool AnimatorComponent::SetFloat(const std::string& name, float value)
    {
        auto iterator = m_Parameters.find(name);

        if (iterator == m_Parameters.end() || iterator->second.Type != AnimatorParameterType::Float)
        {
            return false;
        }

        iterator->second.FloatValue = value;

        return true;
    }

    bool AnimatorComponent::SetBool(const std::string& name, bool value)
    {
        auto iterator = m_Parameters.find(name);

        if (iterator == m_Parameters.end() || iterator->second.Type != AnimatorParameterType::Bool)
        {
            return false;
        }

        iterator->second.BoolValue = value;

        return true;
    }

    bool AnimatorComponent::SetInt(const std::string& name, int value)
    {
        auto iterator = m_Parameters.find(name);

        if (iterator == m_Parameters.end() || iterator->second.Type != AnimatorParameterType::Int)
        {
            return false;
        }

        iterator->second.IntValue = value;

        return true;
    }

    bool AnimatorComponent::SetTrigger(const std::string& name)
    {
        auto iterator = m_Parameters.find(name);

        if (iterator == m_Parameters.end() || iterator->second.Type != AnimatorParameterType::Trigger)
        {
            return false;
        }

        iterator->second.TriggerValue = true;

        return true;
    }

    bool AnimatorComponent::ResetTrigger(const std::string& name)
    {
        auto iterator = m_Parameters.find(name);

        if (iterator == m_Parameters.end() || iterator->second.Type != AnimatorParameterType::Trigger)
        {
            return false;
        }

        iterator->second.TriggerValue = false;

        return true;
    }

    bool AnimatorComponent::GetBool(const std::string& name, bool fallback) const
    {
        auto iterator = m_Parameters.find(name);

        if (iterator == m_Parameters.end() || iterator->second.Type != AnimatorParameterType::Bool)
        {
            return fallback;
        }

        return iterator->second.BoolValue;
    }

    float AnimatorComponent::GetFloat(const std::string& name, float fallback) const
    {
        auto iterator = m_Parameters.find(name);

        if (iterator == m_Parameters.end() || iterator->second.Type != AnimatorParameterType::Float)
        {
            return fallback;
        }

        return iterator->second.FloatValue;
    }

    int AnimatorComponent::GetInt(const std::string& name, int fallback) const
    {
        auto iterator = m_Parameters.find(name);

        if (iterator == m_Parameters.end() || iterator->second.Type != AnimatorParameterType::Int)
        {
            return fallback;
        }

        return iterator->second.IntValue;
    }

    bool AnimatorComponent::GetTrigger(const std::string& name) const
    {
        auto iterator = m_Parameters.find(name);

        if (iterator == m_Parameters.end() || iterator->second.Type != AnimatorParameterType::Trigger)
        {
            return false;
        }

        return iterator->second.TriggerValue;
    }

    void AnimatorComponent::AddTransition(AnimatorTransition2D transition)
    {
        if (transition.ExiTime < 0.0f)
        {
            transition.ExiTime = 0.0f;
        }

        if (transition.ExiTime > 1.0f)
        {
            transition.ExiTime = 1.0f;
        }

        m_Transitions.push_back(std::move(transition));

        m_TransitionsDirty = true;
    }

    void AnimatorComponent::ClearTransitions()
    {
        m_Transitions.clear();
    }

    bool AnimatorComponent::EvaluateTransitions()
    {
        SortTransitionsIfNeeded();

        for (const AnimatorTransition2D& transition : m_Transitions)
        {
            const bool fromAnyState = transition.FromState == AnimatorAnyState;

            const bool fromCurrentState = transition.FromState == m_CurrentStateName;

            if (!fromAnyState && !fromCurrentState)
            {
                continue;
            }

            if (!transition.CanTransitionToSelf && transition.ToState == m_CurrentStateName)
            {
                continue;
            }

            if (!IsExitTimeSatisfied(transition))
            {
                continue;
            }

            if (!AreTransitionConditionsMet(transition))
            {
                continue;
            }

            m_LastTransitionFrom = m_CurrentStateName;

            m_LastTransitionTo = transition.ToState;

            m_LastTransitionPriority = transition.Priority;

            if (ChangeState(transition.ToState, false))
            {
                ConsumeTransitionTriggers(transition);

                return true;
            }
        }

        return false;
    }

    bool AnimatorComponent::EvaluateCondition(const AnimatorCondition2D& condition) const
    {
        auto iterator = m_Parameters.find(condition.Parameter);

        if (iterator == m_Parameters.end())
        {
            return false;
        }

        const AnimatorParameter2D& parameter = iterator->second;

        switch (condition.Mode)
        {
            case Engine::AnimatorConditionMode::IsTrue:
            {
                return parameter.Type == AnimatorParameterType::Bool && parameter.BoolValue;
            }

            case Engine::AnimatorConditionMode::IsFalse:
            {
                return parameter.Type == AnimatorParameterType::Bool && !parameter.BoolValue;
            }

            case Engine::AnimatorConditionMode::Greater:
            {
                return parameter.Type == AnimatorParameterType::Float && parameter.FloatValue > condition.FloatValue;
            }

            case Engine::AnimatorConditionMode::GreaterOrEqual:
            {
                return parameter.Type == AnimatorParameterType::Float && parameter.FloatValue >= condition.FloatValue;
            }

            case Engine::AnimatorConditionMode::Less:
            {
                return parameter.Type == AnimatorParameterType::Float && parameter.FloatValue < condition.FloatValue;
            }

            case Engine::AnimatorConditionMode::LessOrEqual:
            {
                return parameter.Type == AnimatorParameterType::Float && parameter.FloatValue <= condition.FloatValue;
            }

            case Engine::AnimatorConditionMode::Triggered:
            {
                return parameter.Type == AnimatorParameterType::Trigger && parameter.TriggerValue;
            }

            default:
             return false;
        }
    }

    bool AnimatorComponent::AreTransitionConditionsMet(const AnimatorTransition2D& transition) const
    {
        for (const AnimatorCondition2D& condition : transition.Conditions)
        {
            if (!EvaluateCondition(condition))
            {
                return false;
            }
        }

        return true;
    }

    bool AnimatorComponent::IsExitTimeSatisfied(const AnimatorTransition2D& transition) const
    {
        if (!transition.HasExitTime)
        {
            return true;
        }

        if (!m_CurrentClip || !m_CurrentClip->IsValid())
        {
            return true;
        }

        if (m_CurrentClip->IsLooping())
        {
            return false;
        }

        return GetNormalizedTime() >= transition.ExiTime;
    }

    void AnimatorComponent::ConsumeTransitionTriggers(const AnimatorTransition2D& transition)
    {
        for (const AnimatorCondition2D& condition : transition.Conditions)
        {
            if (condition.Mode == AnimatorConditionMode::Triggered)
            {
                ResetTrigger(condition.Parameter);
            }
        }
    }

    float AnimatorComponent::GetNormalizedTime() const
    {
        if (!m_CurrentClip || !m_CurrentClip->IsValid())
        {
            return 0.0f;
        }

        const float duration = m_CurrentClip->GetDuration();

        if (duration <= 0.0f)
        {
            return 0.0f;
        }

        float normalized = m_PlaybackTime / duration;

        if (normalized < 0.0f)
        {
            normalized = 0.0f;
        }

        if (normalized > 1.0f)
        {
            normalized = 1.0f;
        }

        return normalized;
    }

    void AnimatorComponent::SortTransitionsIfNeeded()
    {
        if (!m_TransitionsDirty)
        {
            return;
        }

        std::stable_sort(m_Transitions.begin(), m_Transitions.end(),
                        [](const AnimatorTransition2D& a, const AnimatorTransition2D& b)
                        {
                            return a.Priority > b.Priority;
                        }
                    );

        m_TransitionsDirty = false;
    }

    void AnimatorComponent::SetStateEnterCallback(StateCallback callback)
    {
        m_OnStateEnter = std::move(callback);
    }

    void AnimatorComponent::SetStateExitCallback(StateCallback callback)
    {
        m_OnStateExit = std::move(callback);
    }

    bool AnimatorComponent::ChangeState(const std::string& stateName, bool restartIfSame)
    {
        const AnimatorState2D* state = GetState(stateName);

        if (!state)
        {
            return false;
        }

        const bool sameState = m_CurrentStateName == stateName;

        if (sameState && m_IsPlaying && !restartIfSame)
        {
            return true;
        }

        const std::string oldState = m_CurrentStateName;

        if (!oldState.empty() && m_OnStateExit)
        {
            m_OnStateExit(oldState);
        }

        m_CurrentStateName = stateName;

        m_CurrentStateSpeed = state->SpeedMultiplier;

        PlayClipInternal(state->Clip, restartIfSame);

        if (m_OnStateEnter)
        {
            m_OnStateEnter(m_CurrentStateName);
        }

        return true;
    }

    const std::string& AnimatorComponent::GetLastTransitionFrom() const
    {
        return m_LastTransitionFrom;
    }

    const std::string& AnimatorComponent::GetLastTransitionTo() const
    {
        return m_LastTransitionTo;
    }

    int AnimatorComponent::GetLastTransitionPriority() const
    {
        return m_LastTransitionPriority;
    }

    void AnimatorComponent::ResetEventTracking()
    {
        m_FiredEvents.clear();

        if (!m_CurrentClip)
        {
            return;
        }

        m_FiredEvents.resize(m_CurrentClip->GetEvents().size(), false);
    }

    float AnimatorComponent::GetClipNormalizedTime() const
    {
        if (!m_CurrentClip || !m_CurrentClip->IsValid())
        {
            return 0.0f;
        }

        const float duration = m_CurrentClip->GetDuration();

        if (duration <= 0.0f)
        {
            return 0.0f;
        }

        float normalized = m_ClipLocalTime / duration;

        if (normalized < 0.0f)
        {
            normalized = 0.0f;
        }

        if (normalized > 1.0f)
        {
            normalized = 1.0f;
        }

        return normalized;
    }

    void AnimatorComponent::DispatchAnimationEvents(float previousNormalized, float currentNormalized)
    {
        if (!m_CurrentClip)
        {
            return;
        }

        const auto& events = m_CurrentClip->GetEvents();

        if (m_FiredEvents.size() != events.size())
        {
            m_FiredEvents.assign(events.size(), false);
        }

        for (std::size_t i = 0; i < events.size(); ++i)
        {
            if (m_FiredEvents[i])
            {
                continue;
            }

            const bool crossed = 
                events[i].NormalizedTime > previousNormalized &&
                events[i].NormalizedTime <= currentNormalized;

            const bool atStart = previousNormalized == 0.0f && events[i].NormalizedTime == 0.0f;

            if (crossed || atStart)
            {
                m_FiredEvents[i] = true;

                PublishAnimationEvent(events[i]);

                if (m_OnAnimationEvent)
                {
                    m_OnAnimationEvent(events[i]);
                }
            }
        }
    }

    void AnimatorComponent::DispatchLoopingAnimationEvents(float previousNormalized, float currentNormalized)
    {
        if (!m_CurrentClip)
        {
            return;
        }

        const auto& events = m_CurrentClip->GetEvents();

        if (m_FiredEvents.size() != events.size())
        {
            m_FiredEvents.assign(events.size(), false);
        }

        // Finish events from prevous loop
        for (std::size_t i = 0; i < events.size(); ++i)
        {
            if (m_FiredEvents[i])
            {
                continue;
            }

            if (events[i].NormalizedTime > previousNormalized)
            {
                m_FiredEvents[i] = true;

                PublishAnimationEvent(events[i]);

                if (m_OnAnimationEvent)
                {
                    m_OnAnimationEvent(events[i]);
                }
            }
        }

        // New loop starts
        std::fill(m_FiredEvents.begin(), m_FiredEvents.end(), false);

        // Fire beginning-of-loop events
        for (std::size_t i = 0; i < events.size(); ++i)
        {
            if (events[i].NormalizedTime <= currentNormalized)
            {
                m_FiredEvents[i] = true;

                PublishAnimationEvent(events[i]);

                if (m_OnAnimationEvent)
                {
                    m_OnAnimationEvent(events[i]);
                }
            }
        }
    }

    void AnimatorComponent::PublishAnimationEvent(const AnimationEvent2D& event)
    {
        Entity* owner = GetOwner();

        if (!owner)
        {
            return;
        }

        Scene* scene = owner->GetScene();

        if (!scene)
        {
            return;
        }

        AnimationNotifyEvent notify;

        notify.Source = scene->CreateHandle(owner);

        notify.StateName = m_CurrentStateName;

        notify.NotifyName = event.Name;

        notify.NormalizedTime = event.NormalizedTime;

        if (m_CurrentClip)
        {
            notify.ClipName = m_CurrentClip->GetName();
        }

        scene->GetEventBus().Publish<AnimationNotifyEvent>(notify);
    }
}