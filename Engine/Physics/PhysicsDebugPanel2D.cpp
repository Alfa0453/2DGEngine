#include "PhysicsDebugPanel2D.h"
#include "PhysicsWorld2D.h"

#include <iostream>

namespace Engine
{
    PhysicsDebugPanel2D::PhysicsDebugPanel2D(PhysicsWorld2D* physicsWorld)
        : m_Physicsworld(physicsWorld)
    {
    }

    void PhysicsDebugPanel2D::SetPhysicsWorld(PhysicsWorld2D* physicsWorld)
    {
        m_Physicsworld = physicsWorld;
    }

    PhysicsWorld2D* PhysicsDebugPanel2D::GetPhysicsWorld() const
    {
        return m_Physicsworld;
    }

    bool PhysicsDebugPanel2D::IsOpen() const
    {
        return m_IsOpen;
    }

    void PhysicsDebugPanel2D::SetOpen(bool open)
    {
        m_IsOpen = open;
    }

    void PhysicsDebugPanel2D::Toggle()
    {
        m_IsOpen = !m_IsOpen;
    }

    void PhysicsDebugPanel2D::PrintSummary() const
    {
        if (!m_IsOpen || !m_Physicsworld)
        {
            return;
        }

        const PhysicsDebugSnapshot2D snapshot = m_Physicsworld->GetDebugSnapshot();

        std::cout
            << "\n=== Physics Debug ===\n"

            << "Step: "
            << snapshot.Stats.StepIndex
            << '\n'

            << "Dynamic: "
            << snapshot.Stats.DynamicBodies
            << '\n'

            << "Sleeping: "
            << snapshot.Stats.SleepingBodies
            << '\n'

            << "Colliders: "
            << snapshot.Stats.ActiveColliders
            << '\n'

            << "Candidate pairs: "
            << snapshot.Stats.CandidatePairs
            << '\n'

            << "Contacts: "
            << snapshot.Stats.GeneratedManifolds
            << '\n'

            << "Islands: "
            << snapshot.Stats.Islands
            << '\n'

            << "CCD hits: "
            << snapshot.Stats.CCDHits
            << '\n';
    }
}