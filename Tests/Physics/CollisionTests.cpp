#include "../TestFramework.h"

#include "PhysicsScenarioFixture2D.h"

#include "../../Engine/Scene/Scene.h"
#include "../../Engine/Scene/Entity.h"
#include "../../Engine/Scene/TransformComponent.h"

#include "../../Engine/Physics/PhysicsWorld2D.h"
#include "../../Engine/Physics/BoxCollider2D.h"
#include "../../Engine/Physics/CapsuleCollider2D.h"
#include "../../Engine/Physics/PolygonCollider2D.h"
#include "../../Engine/Physics/CollisionManifold2D.h"

#include <vector>

// -----------------------------------------------------------------------------
// Narrow-phase collision detection tests for the capsule shape pairs that were
// previously unimplemented in GenerateManifold: Capsule vs Box and Capsule vs
// Polygon. Both colliders are left static (no Rigidbody2D), so the solver never
// moves them and GetCurrentContacts() exposes the raw detected manifold.
//
// Coordinate note: gravity is +Y (down) in this engine, so a body resting on a
// surface sits at a smaller Y and the contact normal that pushes it off the
// surface points +Y ("down" toward the surface it presses on).
// -----------------------------------------------------------------------------

namespace
{
    using namespace Engine;

    Entity* MakeEntity(Scene& scene, const char* name, const Vector2& position, float rotationDegrees = 0.0f)
    {
        Entity* entity = scene.CreateEntity(name);

        TransformComponent* transform = entity->AddComponent<TransformComponent>();

        transform->SetWorldRotation(rotationDegrees);

        transform->SetWorldPosition(position);

        return entity;
    }

    const CollisionManifold2D* FindContact(const PhysicsWorld2D& physics, const Collider2D* first, const Collider2D* second)
    {
        for (const CollisionManifold2D& manifold : physics.GetCurrentContacts())
        {
            const bool match =
                (manifold.A == first && manifold.B == second) ||
                (manifold.A == second && manifold.B == first);

            if (match)
            {
                return &manifold;
            }
        }

        return nullptr;
    }

    // Contact normal oriented so that it points away from `from` toward the
    // other collider, independent of which collider the manifold stored as A.
    Vector2 NormalFrom(const CollisionManifold2D& manifold, const Collider2D* from)
    {
        return (manifold.A == from) ? manifold.Normal : manifold.Normal * -1.0f;
    }
}

// A vertical capsule overlapping the top face of a box must be detected, with a
// vertical normal and a penetration of ~5 units.
TEST_CASE(CapsuleVsBox_Overlap_GeneratesContact)
{
    Tests::PhysicsScenarioFixture2D fixture;

    Scene& scene = fixture.GetScene();

    Entity* boxEntity = MakeEntity(scene, "box", {0.0f, 0.0f});

    BoxCollider2D* box = boxEntity->AddComponent<BoxCollider2D>();

    box->SetSize({200.0f, 40.0f}); // X[-100,100], Y[-20,20]

    // Vertical capsule: core Y[-65,-25], +radius => bottom tip at Y=-15, which
    // penetrates the box top face (Y=-20) by 5 units.
    Entity* capsuleEntity = MakeEntity(scene, "capsule", {0.0f, -45.0f});

    CapsuleCollider2D* capsule = capsuleEntity->AddComponent<CapsuleCollider2D>(10.0f, 20.0f);

    fixture.Step(1);

    const CollisionManifold2D* manifold = FindContact(fixture.GetPhysics(), box, capsule);

    TEST_ASSERT(manifold != nullptr);

    TEST_ASSERT(manifold->ContactCount >= 1);

    const Vector2 normalFromCapsule = NormalFrom(*manifold, capsule);

    TEST_ASSERT_NEAR(normalFromCapsule.X, 0.0f, 0.05f);

    TEST_ASSERT_NEAR(normalFromCapsule.Y, 1.0f, 0.05f);

    TEST_ASSERT_NEAR(manifold->Penetration, 5.0f, 1.0f);
}

// A horizontal capsule lying flat across the top of a box must produce a stable
// TWO-point manifold (the resting case).
TEST_CASE(CapsuleVsBox_Parallel_GeneratesTwoContacts)
{
    Tests::PhysicsScenarioFixture2D fixture;

    Scene& scene = fixture.GetScene();

    Entity* boxEntity = MakeEntity(scene, "box", {0.0f, 0.0f});

    BoxCollider2D* box = boxEntity->AddComponent<BoxCollider2D>();

    box->SetSize({200.0f, 40.0f}); // top face at Y=-20, spanning X[-100,100]

    // Rotated 90 degrees, the capsule's core lies along X (x in [-20,20]); with
    // radius 10 its bottom edge sits at Y=-15, penetrating the top face by 5.
    Entity* capsuleEntity = MakeEntity(scene, "capsule", {0.0f, -25.0f}, 90.0f);

    CapsuleCollider2D* capsule = capsuleEntity->AddComponent<CapsuleCollider2D>(10.0f, 20.0f);

    fixture.Step(1);

    const CollisionManifold2D* manifold = FindContact(fixture.GetPhysics(), box, capsule);

    TEST_ASSERT(manifold != nullptr);

    TEST_ASSERT(manifold->ContactCount == 2);

    const Vector2 normalFromCapsule = NormalFrom(*manifold, capsule);

    TEST_ASSERT_NEAR(normalFromCapsule.X, 0.0f, 0.05f);

    TEST_ASSERT_NEAR(normalFromCapsule.Y, 1.0f, 0.05f);

    TEST_ASSERT_NEAR(manifold->Penetration, 5.0f, 1.0f);
}

// A vertical capsule overlapping the top of a square polygon must be detected,
// exercising the GetWorldPolygon() path (distinct from the box conversion path).
TEST_CASE(CapsuleVsPolygon_Overlap_GeneratesContact)
{
    Tests::PhysicsScenarioFixture2D fixture;

    Scene& scene = fixture.GetScene();

    Entity* polygonEntity = MakeEntity(scene, "polygon", {0.0f, 0.0f});

    PolygonCollider2D* polygon = polygonEntity->AddComponent<PolygonCollider2D>();

    polygon->SetVertices(std::vector<Vector2>{
        {-60.0f, -60.0f},
        { 60.0f, -60.0f},
        { 60.0f,  60.0f},
        {-60.0f,  60.0f}
    }); // top face at Y=-60

    // Vertical capsule: core Y[-105,-65], +radius => bottom tip at Y=-55,
    // penetrating the polygon top (Y=-60) by 5 units.
    Entity* capsuleEntity = MakeEntity(scene, "capsule", {0.0f, -85.0f});

    CapsuleCollider2D* capsule = capsuleEntity->AddComponent<CapsuleCollider2D>(10.0f, 20.0f);

    fixture.Step(1);

    const CollisionManifold2D* manifold = FindContact(fixture.GetPhysics(), polygon, capsule);

    TEST_ASSERT(manifold != nullptr);

    TEST_ASSERT(manifold->ContactCount >= 1);

    const Vector2 normalFromCapsule = NormalFrom(*manifold, capsule);

    TEST_ASSERT_NEAR(normalFromCapsule.X, 0.0f, 0.05f);

    TEST_ASSERT_NEAR(normalFromCapsule.Y, 1.0f, 0.05f);

    TEST_ASSERT_NEAR(manifold->Penetration, 5.0f, 1.0f);
}

// A capsule well clear of a box must produce no contact between the two.
TEST_CASE(CapsuleVsBox_Separated_NoContact)
{
    Tests::PhysicsScenarioFixture2D fixture;

    Scene& scene = fixture.GetScene();

    Entity* boxEntity = MakeEntity(scene, "box", {0.0f, 0.0f});

    BoxCollider2D* box = boxEntity->AddComponent<BoxCollider2D>();

    box->SetSize({40.0f, 40.0f});

    Entity* capsuleEntity = MakeEntity(scene, "capsule", {0.0f, -200.0f});

    CapsuleCollider2D* capsule = capsuleEntity->AddComponent<CapsuleCollider2D>(10.0f, 20.0f);

    fixture.Step(1);

    const CollisionManifold2D* manifold = FindContact(fixture.GetPhysics(), box, capsule);

    TEST_ASSERT(manifold == nullptr);
}
