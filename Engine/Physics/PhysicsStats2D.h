#pragma once

#include <cstddef>
#include <cstdint>

namespace Engine
{
    struct PhysicsStats2D
    {
        // =====================================
        // STEP
        // =====================================

        std::uint64_t StepIndex =
            0;


        float StepDeltaTime =
            0.0f;


        std::size_t SubStepsThisUpdate =
            0;


        // =====================================
        // WORLD CONTENT
        // =====================================

        std::size_t ActiveColliders =
            0;


        std::size_t DynamicBodies =
            0;


        std::size_t KinematicBodies =
            0;


        std::size_t StaticBodies =
            0;


        std::size_t SleepingBodies =
            0;


        // =====================================
        // BROAD PHASE
        // =====================================

        std::size_t SpatialCells =
            0;


        std::size_t BroadPhaseProxies =
            0;


        std::size_t CandidatePairs =
            0;


        // =====================================
        // NARROW PHASE
        // =====================================

        std::size_t NarrowPhaseTests =
            0;


        std::size_t GeneratedManifolds =
            0;


        std::size_t ContactPoints =
            0;


        std::size_t TriggerPairs =
            0;


        // =====================================
        // ISLANDS / JOINTS
        // =====================================

        std::size_t Islands =
            0;


        std::size_t Joints =
            0;


        std::size_t ActiveJointConstraints =
            0;


        // =====================================
        // SOLVER
        // =====================================

        std::size_t VelocityContactSolves =
            0;


        std::size_t PositionContactSolves =
            0;


        std::size_t JointVelocitySolves =
            0;


        std::size_t JointPositionSolves =
            0;


        // =====================================
        // CCD
        // =====================================

        std::size_t CCDBodies =
            0;


        std::size_t CCDCandidateTests =
            0;


        std::size_t CCDNarrowPhaseTests =
            0;


        std::size_t CCDHits =
            0;


        std::size_t CCDResolvedImpacts =
            0;


        std::size_t SweptTriggerHits =
            0;


        // =====================================
        // CACHE
        // =====================================

        std::size_t CachedContactPairs =
            0;


        std::size_t RestoredCachedContacts =
            0;
    };
}