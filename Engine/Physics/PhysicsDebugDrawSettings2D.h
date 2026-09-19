#pragma once

namespace Engine
{
    struct PhysicsDebugDrawSettings2D
    {
        bool DrawColliders = true;

        bool DrawAABBs = false;

        bool DrawContactPoints = true;

        bool DrawContactNormals = true;

        bool DrawBroadPhaseCells = false;

        bool DrawSleepingBodies = false;

        bool DrawIslands = false;

        bool DrawJoints = true;

        bool DrawCCDSweeps = false;

        bool DrawCCDImpactPoints = false;
    };
}