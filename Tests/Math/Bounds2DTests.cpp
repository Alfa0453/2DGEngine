#include "../TestFramework.h"

#include "../../Engine/Math/Bounds2D.h"

// Test intersection
TEST_CASE(Bounds2D_OverlappingBoundsIntersect)
{
    Engine::Bounds2D a;

    a.Min = {0.0f, 0.0f};

    a.Max = {10.0f, 10.0f};

    Engine::Bounds2D b;

    b.Min = {5.0f, 5.0f};

    b.Max = {15.0f, 15.0f};

    TEST_ASSERT(a.Intersects(b));
}

// Test non-intersection
TEST_CASE(Bounds2D_SeparatedBoundsDoNotIntersect)
{
    Engine::Bounds2D a;

    a.Min = {0.0f, 0.0f};

    a.Max = {10.0f, 10.0f};

    Engine::Bounds2D b;

    b.Min = {20.0f, 20.0f};

    b.Max = {30.0f, 30.0f};

    TEST_ASSERT(!a.Intersects(b));
}

TEST_CASE(Bounds2D_TouchingEdgesInterset)
{
    Engine::Bounds2D a;

    a.Min = {0.0f, 0.0f};

    a.Max = {10.0f, 10.0f};

    Engine::Bounds2D b;

    b.Min = {10.0f, 0.0f};

    b.Max = {20.0f, 10.0f};

    TEST_ASSERT(a.Intersects(b));
}

TEST_CASE(Bounds2D_GetCenter)
{
    Engine::Bounds2D bounds;

    bounds.Min = {-10.0f, -20.0f};

    bounds.Max = {30.0f, 40.0f};

    Engine::Vector2 expected{10.0f, 10.0f};

    TEST_ASSERT_VECTOR_NEAR(bounds.GetCenter(), expected, 0.0001f);
}

TEST_CASE(Bounds2D_GetSize)
{
    Engine::Bounds2D bounds;

    bounds.Min = {-10.0f, -20.0f};

    bounds.Max = {30.0f, 40.0f};

    Engine::Vector2 expected{40.0f, 60.0f};

    TEST_ASSERT_VECTOR_NEAR(bounds.GetSize(), expected, 0.0001f);
}