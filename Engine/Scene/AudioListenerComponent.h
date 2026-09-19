#pragma once

#include "Component.h"

#include "../Audio/Spatial/AudioListenerState.h"

#include <cstdint>

namespace Engine
{
    class AudioSystem;

    class AudioListenerComponent : public Component
    {
    public:

        AudioListenerComponent() = default;

        ~AudioListenerComponent() override = default;

        void Start() override;

        void Update(float deltaTime) override;

        void OnDestroy() override;

        void SetAudioSystem(AudioSystem* audioSystem);

        AudioSystem* GetAudioSystem() const;

        void SetEnabled(bool enabled);

        bool IsEnabled() const;

        void SetForward(const Vector2& forward);

        const Vector2& GetForward() const;

        void SetVelocity(const Vector2& velocity);

        const Vector2& GetVelocity() const;

        void SyncToAudioSystem();

        void SetAutomaticVelocity(bool automatic);

        bool IsAutomaticVelocityEnabled() const;

    private:
        
        AudioSystem* m_AudioSystem = nullptr;

        Vector2 m_Forward{1.0f, 0.0f};

        Vector2 m_Velocity{0.0f, 0.0f};

        bool m_Enabled = true;

        std::uint64_t m_LastTransformWorldVersion = 0;

        bool m_ListenerStateDirty = true;

        Vector2 m_PreviousWorldPosition{0.0f, 0.0f};

        bool m_HasPreviousWorldPosition = false;

        bool m_AutomaticVelocity = true;

        bool m_HadAutomaticMotion = false;
    };
}