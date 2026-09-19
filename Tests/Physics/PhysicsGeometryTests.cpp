#include "../TestFramework.h"

#include "../../Engine/Physics/PhysicsGeometry2D.h"

TEST_CASE(ClosestPointOnSegment_MiddleProjection)
{
    const Engine::Vector2 point{5.0f, 5.0f};

    const Engine::Vector2 a{0.0f, 0.0f};

    const Engine::Vector2 b{10.0f, 10.0f};

    const Engine::Vector2 result = Engine::PhysicsGeometry2D::ClosestPointOnSegment(point, a, b);

    const Engine::Vector2 expected{5.0f, 5.0f};

    TEST_ASSERT_VECTOR_NEAR(result, expected, 0.0001f);
}

TEST_CASE(ClosestPointOnSegment_ClampsToStart)
{
    const Engine::Vector2 result = Engine::PhysicsGeometry2D::ClosestPointOnSegment({-10.0f, 3.0f}, {0.0f, 0.0f}, {10.0f, 0.0f});

    Engine::Vector2 expected{0.0f, 0.0f};

    TEST_ASSERT_VECTOR_NEAR(result, expected, 0.0001f);
}

TEST_CASE(ClosestPointOnSegment_ClampedToEnd)
{
    const Engine::Vector2 result = Engine::PhysicsGeometry2D::ClosestPointOnSegment({20.0f, -4.0f}, {0.0f, 0.0f}, {10.0f, 0.0f});

    Engine::Vector2 expected{10.0f, 0.0f};

    TEST_ASSERT_VECTOR_NEAR(result, expected, 0.0001f);
}

TEST_CASE(ClosestPointOnSegment_DegenerateSegment)
{
    const Engine::Vector2 segmentPoint{20.0f, 35.0f};

    const Engine::Vector2 result = Engine::PhysicsGeometry2D::ClosestPointOnSegment({100.0f, 100.0f}, segmentPoint, segmentPoint);

    TEST_ASSERT_VECTOR_NEAR(result, segmentPoint, 0.0001f);
}