#include "../TestFramework.h"

#include "PhysicsScenarioFixture2D.h"

#include "../../Engine/Scene/Scene.h"
#include "../../Engine/Scene/Entity.h"
#include "../../Engine/Scene/TransformComponent.h"

#include "../../Engine/Physics/PhysicsWorld2D.h"
#include "../../Engine/Physics/BoxCollider2D.h"
#include "../../Engine/Physics/CircleCollider2D.h"
#include "../../Engine/Physics/Rigidbody2D.h"

// -----------------------------------------------------------------------------
// Continuous collision detection (CCD) tests.
//
// A body moving fast enough to cross a thin wall within a single fixed step
// would tunnel straight through it under discrete detection (the wall is never
// overlapped at either the start or the end of the step). With CCD enabled the
// swept path is tested, so the body is stopped at the wall's surface instead.
//
// These tests set up exactly that situation and check both outcomes: the wall
// is genuinely tunnelled in Discrete mode (which is why CCD exists) and is NOT
// tunnelled in Continuous mode.
//
// Gravity is disabled so the motion stays purely horizontal and the assertions
// concern only the X axis.
// -----------------------------------------------------------------------------

namespace
{
    using namespace Engine;

    Entity* MakeEntity(Scene& scene, const char* name, const Vector2& position)
    {
        Entity* entity = scene.CreateEntity(name);

        TransformComponent* transform = entity->AddComponent<TransformComponent>();

        transform->SetWorldPosition(position);

        return entity;
    }

    // Builds a thin static wall (X[-5,5]) and a fast circle starting far to the
    // left and moving right at 1000 units/step. Returns the circle collider and
    // its rigidbody through the out-parameters so each test can set the mode.
    CircleCollider2D* BuildTunnelScenario(Tests::PhysicsScenarioFixture2D& fixture, Rigidbody2D*& outBody)
    {
        Scene& scene = fixture.GetScene();

        fixture.GetPhysics().GetSettings().Gravity = {0.0f, 0.0f};

        // Thin static wall centred on the origin: 10 wide, 400 tall.
        Entity* wallEntity = MakeEntity(scene, "wall", {0.0f, 0.0f});

        BoxCollider2D* wall = wallEntity->AddComponent<BoxCollider2D>();

        wall->SetSize({10.0f, 400.0f});

        // Fast dynamic circle far to the left, aimed straight at the wall.
        Entity* circleEntity = MakeEntity(scene, "bullet", {-500.0f, 0.0f});

        CircleCollider2D* circle = circleEntity->AddComponent<CircleCollider2D>(5.0f);

        Rigidbody2D* body = circleEntity->AddComponent<Rigidbody2D>();

        body->SetBodyType(BodyType2D::Dynamic);

        // 60000 u/s at a 1/60 s step = 1000 units of travel in one step, far
        // more than enough to jump the 10-unit-thick wall in a single frame.
        body->SetVelocity({60000.0f, 0.0f});

        outBody = body;

        return circle;
    }
}

// With CCD enabled the fast circle must be stopped at the wall's near face
// rather than passing through it.
TEST_CASE(CCD_FastCircle_DoesNotTunnelThinWall)
{
    Tests::PhysicsScenarioFixture2D fixture;

    Rigidbody2D* body = nullptr;

    CircleCollider2D* circle = BuildTunnelScenario(fixture, body);

    body->SetCollisionDetectionMode(CollisionDetectionMode2D::Continuous);

    fixture.Step(1);

    const float centerX = circle->GetWorldCenter().X;

    // Did not cross to the far side of the wall (wall left face is at X=-5,
    // circle radius 5, so a stopped circle sits near X=-10).
    TEST_ASSERT(centerX < -5.0f);

    // Was actually advanced to the wall, not left at its start position.
    TEST_ASSERT(centerX > -40.0f);
}

// The same scenario in Discrete mode tunnels straight through - this both
// documents the limitation CCD addresses and proves the scenario really would
// tunnel without CCD, so the test above is meaningful.
TEST_CASE(Discrete_FastCircle_TunnelsThinWall)
{
    Tests::PhysicsScenarioFixture2D fixture;

    Rigidbody2D* body = nullptr;

    CircleCollider2D* circle = BuildTunnelScenario(fixture, body);

    body->SetCollisionDetectionMode(CollisionDetectionMode2D::Discrete);

    fixture.Step(1);

    const float centerX = circle->GetWorldCenter().X;

    // Passed clean through to the far side of the wall.
    TEST_ASSERT(centerX > 5.0f);
}
