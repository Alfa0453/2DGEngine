#pragma once

#include "../Math/Vector2.h"

#include <cstddef>

namespace Engine
{
    struct PhysicsSettings2D
    {
        // FIXED STEP

        float FixedDeltaTime = 1.0f / 60.0f; // The fixed time step for physics updates (in seconds)

        std::size_t MaxSubSteps = 8; // The maximum number of sub-steps allowed per frame

        // WORLD

        Vector2 Gravity{0.0f, 980.0f}; // The gravity vector applied to all physics objects in the world (in pixels per second squared)

        // SOLVER

        std::size_t VelocityIterations = 8; // The number of iterations for the velocity solver

        std::size_t PositionIterations = 3; // The number of iterations for the position solver

        // POSITION CORRECTION

        float PositionSlop = 0.01f; // The allowed penetration depth before position correction is applied (in pixels)

        float PositionCorrectionPercent = 0.35f; // The percentage of the penetration depth to correct per physics update (between 0.0 and 1.0)

        // RESTITUTION

        float RestitutionVelocityThreshold = 20.0f; // The velocity threshold below which restitution is not applied (in pixels per second)

        // SLEEP

        float SleepLinearSpeedThreshold = 5.0f; // The linear speed threshold below which a physics object can go to sleep (in pixels per second)

        float SleepAngularSpeedThreshold = 0.5f; // The angular speed threshold below which a physics object can go to sleep (in radians per second)

        float TimeToSleep = 0.5f; // The time duration a physics object must remain below the sleep thresholds before going to sleep (in seconds)

        float CollisionWakeSpeed = 10.0f; // The speed threshold above which a sleeping physics object will be woken up by a collision (in pixels per second)

        // BROAD PHASE

        float SpatialCellSize = 128.0f; // The size of the spatial partitioning cells used for broad-phase collision detection (in pixels)

        // CCD

        std::size_t MaxCCDImpacts = 4; // The maximum number of continuous collision detection (CCD) impacts allowed per physics update

        float CCDTimeEpsilon = 0.0001f; // The time epsilon used for continuous collision detection (CCD) to prevent tunneling issues (in seconds)

        float CCDSeparation = 0.001f; // The separation distance used for continuous collision detection (CCD) to prevent tunneling issues (in pixels)
    };
}