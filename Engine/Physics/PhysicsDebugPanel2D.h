#pragma once

namespace Engine
{
    class PhysicsWorld2D;

    class PhysicsDebugPanel2D
    {
    public:

        PhysicsDebugPanel2D() = default;

        explicit PhysicsDebugPanel2D(PhysicsWorld2D* physicsWorld);

        void SetPhysicsWorld(PhysicsWorld2D* physicsWorld);

        PhysicsWorld2D* GetPhysicsWorld() const;

        bool IsOpen() const;

        void SetOpen(bool open);

        void Toggle();

        void PrintSummary() const;

    private:

        PhysicsWorld2D* m_Physicsworld = nullptr;

        bool m_IsOpen = false;
    };
}