#include "AudioListenerComponent.h"

#include "../Audio/Core/AudioSystem.h"
#include "AudioSourceComponent.h"
#include "Entity.h"
#include "TransformComponent.h"


namespace Engine
{
    void AudioListenerComponent::SetAudioSystem(AudioSystem* audioSystem)
    {
        if (m_AudioSystem == audioSystem)
        {
            return;
        }

        m_AudioSystem = audioSystem;

        m_ListenerStateDirty = true;

        if (m_AudioSystem)
        {
            SyncToAudioSystem();
        }
    }

    AudioSystem* AudioListenerComponent::GetAudioSystem() const
    {
        return m_AudioSystem;
    }

    void AudioListenerComponent::Start()
    {
        m_ListenerStateDirty = true;

        SyncToAudioSystem();
    }

    void AudioListenerComponent::Update(float deltaTime)
    {
        if (!m_AudioSystem)
        {
            return;
        }

        Entity* owner = GetOwner();

        if (!owner)
        {
            return;
        }

        TransformComponent* ownerTransform = owner->GetComponent<TransformComponent>();

        if (!ownerTransform)
        {
            return;
        }

        const std::uint64_t worldVersion = ownerTransform->GetWorldVersion();

        const bool transformChanged = worldVersion != m_LastTransformWorldVersion;

        if (!transformChanged && !m_ListenerStateDirty)
        {
            return;
        }

        const Vector2 currentPosition = ownerTransform->GetWorldPosition();

        if (m_AutomaticVelocity && deltaTime > 0.000001f)
        {
            if (m_HasPreviousWorldPosition)
            {
                m_Velocity = (currentPosition - m_PreviousWorldPosition) / deltaTime;
            }
            else 
            {
                m_Velocity = Vector2{0.0f, 0.0f};
            }

            m_PreviousWorldPosition = currentPosition;

            m_HasPreviousWorldPosition = true;

            m_HadAutomaticMotion = m_Velocity.LengthSquared() > 0.000001f;

            m_ListenerStateDirty = true;
        }

        SyncToAudioSystem();
    }

    void AudioListenerComponent::SetEnabled(bool enabled)
    {
        if (m_Enabled == enabled)
        {
            return;
        }

        m_Enabled = enabled;

        m_ListenerStateDirty = true;
    }

    bool AudioListenerComponent::IsEnabled() const
    {
        return m_Enabled;
    }

    void AudioListenerComponent::SetForward(const Vector2& forward)
    {
        if (forward.LengthSquared() > 0.000001f)
        {
            m_Forward = forward.Normalized();
        }
        else 
        {
            m_Forward = Vector2{1.0f, 0.0f};
        }

        m_ListenerStateDirty = true;
    }

    const Vector2& AudioListenerComponent::GetForward() const
    {
        return m_Forward;
    }

    void AudioListenerComponent::SetVelocity(const Vector2& velocity)
    {
        m_Velocity = velocity;

        m_AutomaticVelocity = false;

        m_ListenerStateDirty = true;
    }

    const Vector2& AudioListenerComponent::GetVelocity() const
    {
        return m_Velocity;
    }

    void AudioListenerComponent::SyncToAudioSystem()
    {
        if (!m_AudioSystem)
        {
            return;
        }

        Entity* owner = GetOwner();

        if (!owner)
        {
            return;
        }

        TransformComponent* ownerTransform = owner->GetComponent<TransformComponent>();

        if (!ownerTransform)
        {
            return;
        }

        AudioListenerState state;

        state.Enabled = m_Enabled;

        state.Forward = m_Forward;

        state.Velocity = m_Velocity;

        state.Position = ownerTransform->GetWorldPosition();

        if (m_AudioSystem->SetListenerState(state))
        {
            m_ListenerStateDirty = false;

            m_LastTransformWorldVersion = ownerTransform->GetWorldVersion();
        }
    }

    void AudioListenerComponent::OnDestroy()
    {
        if (!m_AudioSystem)
        {
            return;
        }

        if (!m_Enabled)
        {
            return;
        }

        AudioListenerState state;

        state.Enabled = false;

        state.Forward = m_Forward;

        state.Velocity = m_Velocity;

        Entity* owner = GetOwner();

        if (owner)
        {
            TransformComponent* transform = owner->GetComponent<TransformComponent>();

            if (transform)
            {
                state.Position = transform->GetWorldPosition();
            }
        }

        m_AudioSystem->SetListenerState(state);
    }

    void AudioListenerComponent::SetAutomaticVelocity(bool automatic)
    {
        if (m_AutomaticVelocity == automatic)
        {
            return;
        }

        m_AutomaticVelocity = automatic;

        if (automatic)
        {
            m_HasPreviousWorldPosition = false;

            m_HadAutomaticMotion = false;

            m_Velocity = Vector2{0.0f, 0.0f};
        }

        m_ListenerStateDirty = true;
    }
}