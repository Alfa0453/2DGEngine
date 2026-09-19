#include "PhysicsWorld2D.h"

#include "BoxCollider2D.h"
#include "BroadPhaseProxy2D.h"
#include "CachedContactPair2D.h"
#include "CapsuleCollider2D.h"
#include "CircleCollider2D.h"
#include "Collider2D.h"
#include "ColliderPair2D.h"
#include "CollisionEvents2D.h"
#include "CollisionManifold2D.h"
#include "Joint2D.h"
#include "OrientedBox2D.h"
#include "OverlapHit2D.h"
#include "PhysicsDebugDrawSettings2D.h"
#include "PhysicsGeometry2D.h"
#include "PhysicsIsland2D.h"
#include "PhysicsQueryContext2D.h"
#include "PhysicsQueryFilter2D.h"
#include "PhysicsStats2D.h"
#include "PolygonCollider2D.h"
#include "RaycastHit2D.h"
#include "Rigidbody2D.h"
#include "ShapeCastHit2D.h"
#include "SpatialCell2D.h"
#include "CollisionDetectionMode2D.h"
#include "SweepHit2D.h"
#include "SweptAABBHit2D.h"
#include "PhysicsGeometry2D.h"

#include "../Scene/Scene.h"
#include "../Scene/Entity.h"
#include "../Math/Bounds2D.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace Engine
{
    PhysicsWorld2D::PhysicsWorld2D(Scene* scene)
        : m_Scene(scene)
    {
    }

    void PhysicsWorld2D::SetScene(Scene* scene)
    {
        if (m_Scene == scene)
        {
            return;
        }

        m_ContactCache.clear();

        m_CurrentContacts.clear();

        m_CurrentOverlaps.clear();

        m_PreviousOverlaps.clear();

        m_Islands.clear();

        m_Joints.clear();

        m_ActiveColliders.clear();

        m_SpatialGrid.clear();

        m_BroadPhaseProxies.clear();

        m_CandidatePairs.clear();

        m_SweptTriggerPairsThisStep.clear();

        m_CCDResolvePairsThisStep.clear();

        m_CCDDebugRecords.clear();

        m_Stats = PhysicsStats2D{};

        m_Accumulator = 0.0f;

        m_Scene = scene;
    }

    Scene* PhysicsWorld2D::GetScene() const
    {
        return m_Scene;
    }

    void PhysicsWorld2D::Update(float deltaTime)
    {
        m_Accumulator += deltaTime;

        std::size_t subSteps = 0;

        while (m_Accumulator >= m_Settings.FixedDeltaTime && subSteps < m_Settings.MaxSubSteps)
        {
            Step(m_Settings.FixedDeltaTime);

            m_Accumulator -= m_Settings.FixedDeltaTime;

            ++subSteps;
        }

        m_Stats.SubStepsThisUpdate = subSteps;

        if (subSteps == m_Settings.MaxSubSteps)
        {
            m_Accumulator = 0.0f;
        }
    }

    void PhysicsWorld2D::Step(float deltaTime)
    {
        (void)deltaTime;

        if (!m_Scene)
        {
            return;
        }

        m_CCDDebugRecords.clear();

        ResetStepStats(deltaTime);

        // =====================================
        // 1. Tentative integration
        // =====================================
        
        IntegrateBodies(deltaTime);

        // =====================================
        // 2. Pre-CCD spatial structure
        // =====================================
        
        CollectColliders();

        BuildSpatialGrid();

        m_Stats.SpatialCells = m_SpatialGrid.size();

        m_Stats.BroadPhaseProxies = m_BroadPhaseProxies.size();

        // =====================================
        // 3. CCD
        // =====================================

        m_CCDResolvePairsThisStep.clear();

        m_SweptTriggerPairsThisStep.clear();

        RunContinuousCollisionPass(deltaTime);

        // =====================================
        // 4. CCD changed transforms.
        // Rebuild final broad-phase state.
        // =====================================
        
        CollectColliders();

        BuildSpatialGrid();

        GenerateCandidatePairs();

        m_Stats.CandidatePairs = m_CandidatePairs.size();

        // =====================================
        // 5. Regular discrete contacts
        // =====================================

        m_CurrentOverlaps.clear();

        m_CurrentContacts.clear();


        ProcessCandidatePairs();

        // Restore previous-step impulses
        RestoreCachedContactImpulses();

        // Wake sleeping bodies
        WakeBodiesFromContacts();

        // Build connected Dynamic groups.
        BuildIslands();

        // Wake entire connected islands when one member is active.
        PropagateIslandWakeStates();

        // Velocity solver by island.
        for (PhysicsIsland2D& island : m_Islands)
        {
            PrepareVelocityIsland(island);

            PrepareJoints(island, deltaTime);

            WarmStartVelocityIsland(island);

            WarmStartJoints(island);

            SolveVelocityIsland(island);
        }

        // Save solved velocity impulses
        StoreContactCache();

        // Position solver by island.
        for (PhysicsIsland2D& island : m_Islands)
        {
            SolvePositionIsland(island);
        }

        // Sleep evaluation
        UpdateIslandSleepStates(deltaTime);

        CountBodiesForStats();

        // Collision events
        PublishPairEvents();

        // Store overlap state
        m_PreviousOverlaps = m_CurrentOverlaps;

        // Remove cache entries for pairs that no longer exists.
        RemoveStaleCachedContacts();

        FinalizeStepStats();
    }

    void PhysicsWorld2D::CollectColliders()
    {
        m_ActiveColliders.clear();

        if (!m_Scene)
        {
            return;
        }

        for (Entity* root : m_Scene->GetRootEntities())
        {
            CollectCollidersRecursive(root);
        }
    }

    void PhysicsWorld2D::CollectCollidersRecursive(Entity* entity)
    {
        if (!entity)
        {
            return;
        }

        if (auto* box = entity->GetComponent<BoxCollider2D>())
        {
            const Vector2& size = box->GetSize();

            if (box->IsEnabled() && size.X > 0.0f && size.Y > 0.0f)
            {
                m_ActiveColliders.push_back(box);
            }
        }

        if (auto* circle = entity->GetComponent<CircleCollider2D>())
        {
            if (circle->IsEnabled() && circle->GetRadius() > 0.0f)
            {
                m_ActiveColliders.push_back(circle);
            }
        }

        if (auto* capsule = entity->GetComponent<CapsuleCollider2D>())
        {
            if (capsule->IsEnabled() && capsule->GetRadius() > 0.0f)
            {
                m_ActiveColliders.push_back(capsule);
            }
        }

        if (auto* polygon = entity->GetComponent<PolygonCollider2D>())
        {
            if (polygon->IsEnabled() && polygon->IsValidPolygon())
            {
                m_ActiveColliders.push_back(polygon);
            }
        }

        for (const EntityHandle& childHandle : entity->GetChildren())
        {
            CollectCollidersRecursive(childHandle.Get());
        }
    }

    void PhysicsWorld2D::CollectMovingCCDCandidates(Collider2D* movingCollider, const Bounds2D& movingSweptBounds, std::vector<Collider2D*>& inOutCandidates) const
    {
        std::unordered_set<Collider2D*> unique;

        for (Collider2D* existing : inOutCandidates)
        {
            if (existing)
            {
                unique.insert(existing);
            }
        }

        for (Collider2D* candidate : m_ActiveColliders)
        {
            if (!candidate || candidate == movingCollider)
            {
                continue;
            }

            Entity* owner = candidate->GetOwner();

            if (!owner)
            {
                continue;
            }

            Rigidbody2D* body = owner->GetComponent<Rigidbody2D>();

            TransformComponent* transform = owner->GetComponent<TransformComponent>();

            // Collider-only and Static targets are already efficiently handled by the spatial query.
            //
            // This fallback exists specifically for moving targets.

            if (!body || !transform || body->IsStatic())
            {
                continue;
            }

            const Vector2 bodyMotion = GetBodyStepMotion(body, transform);

            if (bodyMotion.LengthSquared() <= 0.000001f)
            {
                continue;
            }

            const Bounds2D startBounds = ReconstructStartBounds(*candidate, *body, *transform);

            const Bounds2D sweptBounds = BuildSweptBounds(startBounds, bodyMotion);

            if (!AABBsOverlap(movingSweptBounds, sweptBounds))
            {
                continue;
            }

            if (unique.insert(candidate).second)
            {
                inOutCandidates.push_back(candidate);
            }
        }
    }

    void PhysicsWorld2D::IntegrateEntityRecursive(Entity* entity, float deltaTime)
    {
        if (!entity)
        {
            std::cout << "entity is null\n";
            return;
        }

        Rigidbody2D* body = entity->GetComponent<Rigidbody2D>();

        TransformComponent* transform = entity->GetComponent<TransformComponent>();

        if (body && transform)
        {
            switch (body->GetBodyType())
            {
                case BodyType2D::Static:
                    break;

                case BodyType2D::Kinematic:
                    IntegrateKinematicBody(*body, *transform, deltaTime);
                    break;

                case BodyType2D::Dynamic:
                {
                    if (!body->IsSleeping())
                    {
                        IntegrateDynamicBody(*body, *transform, deltaTime);
                    }

                    break;
                }
            }
        }

        for (const EntityHandle& childHandle : entity->GetChildren())
        {
            IntegrateEntityRecursive(childHandle.Get(), deltaTime);
        }
    }

    bool PhysicsWorld2D::ShouldTestPair(Collider2D* a, Collider2D* b) const
    {
        if (!a || !b || a == b)
        {
            return false;
        }

        if (!a->IsEnabled() || !b->IsEnabled())
        {
            return false;
        }

        Entity* ownerA = a->GetOwner();

        Entity* ownerB = b->GetOwner();

        if (!ownerA || !ownerB)
        {
            return false;
        }

        if (ownerA == ownerB)
        {
            return false;
        }

        return a->CanInteractWith(*b);
    }

    bool PhysicsWorld2D::BroadPhaseOverlap(const Collider2D& a, const Collider2D& b) const
    {
        return a.GetWorldBounds().Intersects(b.GetWorldBounds());
    }

    bool PhysicsWorld2D::BoxVsBox(BoxCollider2D& a, BoxCollider2D& b, CollisionManifold2D& manifold) const
    {
        const OrientedBox2D boxA = a.GetWorldOrientedBox();

        const OrientedBox2D boxB = b.GetWorldOrientedBox();

        const Vector2 axes[4]
        {
            boxA.AxisX,
            boxA.AxisY,
            boxB.AxisX,
            boxB.AxisY
        };

        float minimumOverlap = std::numeric_limits<float>::max();

        int minimumAxisIndex = -1;

        Vector2 minimumAxis{0.0f, 0.0f};

        // =========================================================
        // TEST ALL FOUR AXES
        // =========================================================

        for (int i = 0; i < 4; ++i)
        {
            float overlap = 0.0f;

            if (!TestOBBAxis(boxA, boxB, axes[i], overlap))
            {
                // One separating axis is enough to prove there is no collision.
                return false;
            }

            // Keep the shallowest penetration.
            if (overlap < minimumOverlap)
            {
                minimumOverlap = overlap;

                minimumAxis = axes[i];

                minimumAxisIndex = i;
            }
        }

        // =========================================================
        // ORIENT NORMAL A -> B
        // =========================================================

        const Vector2 centerDelta = boxB.Center - boxA.Center;

        if (Vector2::Dot(centerDelta, minimumAxis) < 0.0f)
        {
            minimumAxis *= -1.0f;
        }

        // =========================================================
        // BUILD MANIFOLD
        // =========================================================

        manifold.A = &a;

        manifold.B = &b;

        manifold.Normal = minimumAxis;

        manifold.Penetration = minimumOverlap;

        manifold.IsTrigger = a.IsTrigger() || b.IsTrigger();

        manifold.ClearContacts();


        // =========================================================
        // ROTATED CONTACT POINTS
        // =========================================================

        Vector2 contacts[2];

        const std::size_t contactCount = BuildOBBContactPoints(boxA, boxB, manifold.Normal, minimumAxisIndex, contacts);

        for (std::size_t i = 0; i < contactCount; ++i)
        {
            manifold.AddContactPoint(contacts[i]);
        }

        // =========================================================
        // FALLBACK CONTACT
        // =========================================================

        if (manifold.ContactCount == 0)
        {
            const Vector2 pointA = GetOBBSupportPoint(boxA, manifold.Normal);

            const Vector2 pointB = GetOBBSupportPoint(boxB, manifold.Normal * -1.0f);

            manifold.AddContactPoint((pointA + pointB) * 0.5f);
        }
        
        return true;
    }

    bool PhysicsWorld2D::CircleVsCircle(CircleCollider2D& a, CircleCollider2D& b, CollisionManifold2D& manifold) const
    {
        const Vector2 centerA = a.GetWorldCenter();

        const Vector2 centerB = b.GetWorldCenter();

        const float radiusA = a.GetWorldRadius();

        const float radiusB = b.GetWorldRadius();

        const Vector2 delta = centerB - centerA;

        const float distanceSquared = delta.LengthSquared();

        const float combinedRadius = radiusA + radiusB;

        const float combinedRadiusSquared = combinedRadius * combinedRadius;

        if (distanceSquared >= combinedRadiusSquared)
        {
            return false;
        }

        manifold.A = &a;

        manifold.B = &b;

        manifold.IsTrigger = a.IsTrigger() || b.IsTrigger();

        manifold.ClearContacts();

        constexpr float epsilon = 0.000001f;

        if (distanceSquared > epsilon)
        {
            const float distance = std::sqrt(distanceSquared);

            const Vector2 normal = delta * (1.0f / distance);

            manifold.Normal = normal;

            manifold.Penetration = combinedRadius - distance;

            const Vector2 pointOnA = centerA + normal * radiusA;

            const Vector2 pointOnB = centerB - normal * radiusB;

            const Vector2 contactPoint = (pointOnA + pointOnB) * 0.5f;

            manifold.AddContactPoint(contactPoint);
        }
        else {
            // Both centers are essentially at exactly
        // the same position, so a normal cannot
        // be derived from delta.

            manifold.Normal = {1.0f, 0.0f};

            manifold.Penetration = combinedRadius;

            // Use the surface of Circle A in the
            // fallback normal direction.
            manifold.AddContactPoint(centerA + manifold.Normal * radiusA);
        }

        return true;
    }

    bool PhysicsWorld2D::BoxVsCircle(BoxCollider2D& box, CircleCollider2D& circle, CollisionManifold2D& manifold) const
    {
        // =========================================================
        // TRUE ROTATED BOX GEOMETRY
        // =========================================================

        const OrientedBox2D orientedBox = box.GetWorldOrientedBox();

        const Vector2 circleCenter = circle.GetWorldCenter();

        const float radius = circle.GetWorldRadius();

        // =========================================================
        // CONVERT CIRCLE CENTER INTO BOX-LOCAL SPACE
        // =========================================================

        const Vector2 localCircleCenter = WorldPointToOBBLocal(orientedBox, circleCenter);

        const Vector2 half = orientedBox.HalfExtents;

        // =========================================================
        // FIND CLOSEST POINT ON LOCAL BOX
        // =========================================================

        const Vector2 localClosestPoint
        {
            std::clamp(localCircleCenter.X, -half.X, half.X),

            std::clamp(localCircleCenter.Y, -half.Y, half.Y)
        };

        // Circle center relative to the closest Box point.
        const Vector2 localDelta = localCircleCenter - localClosestPoint;

        const float distanceSquared = localDelta.LengthSquared();

        const float radiusSquared = radius * radius;

        // =========================================================
        // NO COLLISION
        // =========================================================

        if (distanceSquared > radiusSquared)
        {
            return false;
        }

        // =========================================================
        // INITIALIZE MANIFOLD
        // =========================================================

        manifold.A = &box;

        manifold.B = &circle;

        manifold.IsTrigger = box.IsTrigger() || circle.IsTrigger();

        manifold.ClearContacts();

        constexpr float epsilon = 0.000001f;

        // =========================================================
        // CIRCLE CENTER IS OUTSIDE THE BOX
        // =========================================================

        if (distanceSquared > epsilon)
        {
            const float distance = std::sqrt(distanceSquared);
        

            // -----------------------------------------------------
            // Local Box -> Circle normal
            // -----------------------------------------------------

            const Vector2 localNormal = localDelta * (1.0f / distance);

            // -----------------------------------------------------
            // Convert normal back to world space.
            // -----------------------------------------------------

            manifold.Normal = OBBLocalDirectionToWorld(orientedBox, localNormal);

            manifold.Penetration = radius - distance;

            if (manifold.Penetration <= 0.0f)
            {
                return false;
            }

            // -----------------------------------------------------
            // Convert closest point back to world space.
            // -----------------------------------------------------

            manifold.AddContactPoint(OBBLocalPointToWorld(orientedBox, localClosestPoint));

            return true;
        }


        // =========================================================
        // CIRCLE CENTER IS INSIDE THE BOX
        //
        // Or is exactly on the Box boundary.
        // localDelta is zero, so it cannot be normalized.
        // =========================================================

        // Distance to left local face.
        const float toLeft = localCircleCenter.X + half.X;

        // Distance to right local face.
        const float toRight = half.X - localCircleCenter.X;

        // Distance to top local face.
        const float toTop = localCircleCenter.Y + half.Y;

        // Distance to bottom local face.
        const float toBottom = half.Y - localCircleCenter.Y;


        // =========================================================
        // START WITH LEFT FACE
        // =========================================================

        float nearest = toLeft;

        Vector2 localNormal{-1.0f, 0.0f};

        Vector2 localContact{-half.X, localCircleCenter.Y};


        // =========================================================
        // RIGHT FACE
        // =========================================================

        if (toRight < nearest)
        {
            nearest = toRight;

            localNormal = {1.0f, 0.0f};

            localContact = {half.X, localCircleCenter.Y};
        }


        // =========================================================
        // TOP FACE
        // =========================================================

        if (toTop < nearest)
        {
            nearest = toTop;

            localNormal = {0.0f, -1.0f};

            localContact = {localCircleCenter.X, -half.Y};
        }


        // =========================================================
        // BOTTOM FACE
        // =========================================================

        if (toBottom < nearest)
        {
            nearest = toBottom;

            localNormal = {0.0f, 1.0f};

            localContact = {localCircleCenter.X, half.Y};
        }


        // =========================================================
        // LOCAL NORMAL -> WORLD NORMAL
        // =========================================================

        manifold.Normal = OBBLocalDirectionToWorld(orientedBox, localNormal);


        // =========================================================
        // PENETRATION
        // =========================================================

        manifold.Penetration = radius + nearest;

        if (manifold.Penetration <= 0.0f)
        {
            return false;
        }


        // =========================================================
        // LOCAL CONTACT -> WORLD CONTACT
        // =========================================================

        manifold.AddContactPoint(OBBLocalPointToWorld(orientedBox, localContact));


        return true;
    }


    bool PhysicsWorld2D::CapsuleVsCircle(CapsuleCollider2D& capsuleCollider, CircleCollider2D& circle, CollisionManifold2D& manifold) const
    {
        const Capsule2D capsule = capsuleCollider.GetWorldCapsule();

        const Vector2 circleCenter = circle.GetWorldCenter();

        const float circleRadius = circle.GetWorldRadius();

        const Vector2 closestPoint = PhysicsGeometry2D::ClosestPointOnSegment(circleCenter, capsule.PointA, capsule.PointB);

        const Vector2 delta = circleCenter - closestPoint;

        const float distanceSquared = delta.LengthSquared();

        const float combinedRadius = capsule.Radius + circleRadius;

        const float combinedRadiusSquared = combinedRadius * combinedRadius;

        if (distanceSquared >= combinedRadiusSquared)
        {
            return false;
        }

        manifold.A = &capsuleCollider;

        manifold.B = &circle;

        manifold.IsTrigger = capsuleCollider.IsTrigger() || circle.IsTrigger();

        manifold.ClearContacts();

        constexpr float epsilon = 0.000001f;

        if (distanceSquared > epsilon)
        {
            const float distance = std::sqrt(distanceSquared);

            manifold.Normal = delta * (1.0f / distance);

            manifold.Penetration = combinedRadius - distance;

            const Vector2 pointOnCapsule = closestPoint + manifold.Normal * capsule.Radius;

            const Vector2 pointOnCircle = circleCenter - manifold.Normal * circleRadius;

            manifold.AddContactPoint((pointOnCapsule + pointOnCircle) * 0.5f);

            return true;
        }

        // Circle center lies directly on the capsule center segment.

        Vector2 segment = capsule.PointB - capsule.PointA;

        const float segmentLengthSquared = segment.LengthSquared();

        if (segmentLengthSquared > epsilon)
        {
            segment *= 1.0f / std::sqrt(segmentLengthSquared);

            manifold.Normal = {-segment.Y, segment.X};
        }
        else 
        {
            manifold.Normal = {1.0f, 0.0f};
        }

        manifold.Penetration = combinedRadius;

        manifold.AddContactPoint(closestPoint + manifold.Normal * capsule.Radius);

        return true;
    }


    bool PhysicsWorld2D::CapsuleVsCapsule(CapsuleCollider2D& a, CapsuleCollider2D& b, CollisionManifold2D& manifold) const
    {
        const Capsule2D capsuleA = a.GetWorldCapsule();

        const Capsule2D capsuleB = b.GetWorldCapsule();

        const ClosestSegmentPoints2D closest = ClosestPointsBetweenSegments(capsuleA.PointA, capsuleA.PointB, capsuleB.PointA, capsuleB.PointB);

        const Vector2 delta = closest.PointB - closest.PointA;

        const float distanceSquared = delta.LengthSquared();

        const float combinedRadius = capsuleA.Radius + capsuleB.Radius;

        if (distanceSquared >= combinedRadius * combinedRadius)
        {
            return false;
        }

        manifold.A = &a;

        manifold.B = &b;

        manifold.IsTrigger = a.IsTrigger() || b.IsTrigger();

        manifold.ClearContacts();

        constexpr float epsilon = 0.000001f;

        if (distanceSquared > epsilon)
        {
            const float distance = std::sqrt(distanceSquared);

            manifold.Normal = delta * (1.0f / distance);

            manifold.Penetration = combinedRadius - distance;

            const Vector2 pointOnA = closest.PointA + manifold.Normal * capsuleA.Radius;

            const Vector2 pointOnB = closest.PointB - manifold.Normal * capsuleB.Radius;

            manifold.AddContactPoint((pointOnA + pointOnB) * 0.5f);

            return true;
        }

        // DEGENERATE CASE

        const Vector2 centerA = (capsuleA.PointA + capsuleA.PointB) * 0.5f;

        const Vector2 centerB = (capsuleB.PointA + capsuleB.PointB) * 0.5f;

        Vector2 centerDelta = centerB - centerA;

        const float centerDistanceSquared = centerDelta.LengthSquared();

        if (centerDistanceSquared > epsilon)
        {
            manifold.Normal = centerDelta * (1.0f / std::sqrt(centerDistanceSquared));
        }
        else 
        {
            Vector2 segemntA = capsuleA.PointB - capsuleA.PointA;

            const float segmentLengthSquared = segemntA.LengthSquared();

            if (segmentLengthSquared > epsilon)
            {
                segemntA *= 1.0f / std::sqrt(segmentLengthSquared);

                manifold.Normal = {-segemntA.Y, segemntA.X};
            }
            else 
            {
                manifold.Normal = {1.0f, 0.0f};
            }
        }

        manifold.Penetration = combinedRadius;

        manifold.AddContactPoint((closest.PointA + closest.PointB) * 0.5f);

        return true;
    }


    bool PhysicsWorld2D::CapsuleVsBox(CapsuleCollider2D& capsuleCollider, BoxCollider2D& boxCollider, CollisionManifold2D& manifold) const
    {
        // A box is a convex polygon; reuse the capsule-vs-polygon core.

        const Capsule2D capsule = capsuleCollider.GetWorldCapsule();

        const Polygon2D polygon = OrientedBoxToPolygon(boxCollider.GetWorldOrientedBox());

        if (!BuildCapsulePolygonManifold(capsule, polygon, manifold))
        {
            return false;
        }

        // Normal points from A (capsule) toward B (box).

        manifold.A = &capsuleCollider;

        manifold.B = &boxCollider;

        manifold.IsTrigger = capsuleCollider.IsTrigger() || boxCollider.IsTrigger();

        return true;
    }


    bool PhysicsWorld2D::CapsuleVsPolygon(CapsuleCollider2D& capsuleCollider, PolygonCollider2D& polygonCollider, CollisionManifold2D& manifold) const
    {
        const Capsule2D capsule = capsuleCollider.GetWorldCapsule();

        const Polygon2D polygon = polygonCollider.GetWorldPolygon();

        if (!BuildCapsulePolygonManifold(capsule, polygon, manifold))
        {
            return false;
        }

        // Normal points from A (capsule) toward B (polygon).

        manifold.A = &capsuleCollider;

        manifold.B = &polygonCollider;

        manifold.IsTrigger = capsuleCollider.IsTrigger() || polygonCollider.IsTrigger();

        return true;
    }


    bool PhysicsWorld2D::BuildCapsulePolygonManifold(const Capsule2D& capsule, const Polygon2D& polygon, CollisionManifold2D& manifold) const
    {
        if (polygon.Vertices.size() < 3 || polygon.Normals.size() != polygon.Vertices.size())
        {
            return false;
        }

        const Vector2 pointA = capsule.PointA;

        const Vector2 pointB = capsule.PointB;

        const float radius = capsule.Radius;

        constexpr float epsilon = 0.000001f;

        // =====================================================
        // SEPARATING-AXIS TEST
        //
        // A capsule is a segment inflated by a radius. Along any axis its
        // projection is the projection of its core segment widened by the
        // radius on both ends. The candidate axes for a segment-vs-convex
        // test are: the polygon face normals, the capsule's own side normal
        // (perpendicular to its core), and, for each polygon vertex, the axis
        // toward the closest point of the core segment (this captures the
        // rounded caps hitting a corner).
        // =====================================================

        float minimumOverlap = std::numeric_limits<float>::max();

        Vector2 minimumAxis{0.0f, 0.0f};

        bool found = false;

        const auto testAxis = [&](Vector2 axis) -> bool
        {
            const float axisLengthSquared = axis.LengthSquared();

            if (axisLengthSquared <= epsilon)
            {
                // Degenerate axis carries no separation information.

                return true;
            }

            axis *= 1.0f / std::sqrt(axisLengthSquared);

            float minPolygon = 0.0f;

            float maxPolygon = 0.0f;

            ProjectPolygon(polygon, axis, minPolygon, maxPolygon);

            const float d0 = Vector2::Dot(pointA, axis);

            const float d1 = Vector2::Dot(pointB, axis);

            const float minCapsule = std::min(d0, d1) - radius;

            const float maxCapsule = std::max(d0, d1) + radius;

            const float overlap = std::min(maxPolygon, maxCapsule) - std::max(minPolygon, minCapsule);

            if (overlap <= 0.0f)
            {
                // A separating axis exists: the shapes do not intersect.

                return false;
            }

            if (overlap < minimumOverlap)
            {
                minimumOverlap = overlap;

                minimumAxis = axis;

                found = true;
            }

            return true;
        };

        // Polygon face normals.

        for (const Vector2& normal : polygon.Normals)
        {
            if (!testAxis(normal))
            {
                return false;
            }
        }

        // Capsule side normal (perpendicular to the core segment).

        Vector2 segment = pointB - pointA;

        const float segmentLengthSquared = segment.LengthSquared();

        if (segmentLengthSquared > epsilon)
        {
            const Vector2 segmentDirection = segment * (1.0f / std::sqrt(segmentLengthSquared));

            if (!testAxis(Vector2{-segmentDirection.Y, segmentDirection.X}))
            {
                return false;
            }
        }

        // Rounded cap versus polygon vertex axes.

        for (const Vector2& vertex : polygon.Vertices)
        {
            const Vector2 closest = PhysicsGeometry2D::ClosestPointOnSegment(vertex, pointA, pointB);

            if (!testAxis(vertex - closest))
            {
                return false;
            }
        }

        if (!found)
        {
            return false;
        }

        // =====================================================
        // NORMAL ORIENTATION (A = capsule -> B = polygon)
        // =====================================================

        const Vector2 capsuleCenter = (pointA + pointB) * 0.5f;

        const Vector2 polygonCenter = GetPolygonCenter(polygon);

        if (Vector2::Dot(polygonCenter - capsuleCenter, minimumAxis) < 0.0f)
        {
            minimumAxis *= -1.0f;
        }

        manifold.Normal = minimumAxis;

        manifold.Penetration = minimumOverlap;

        manifold.ClearContacts();

        // =====================================================
        // CONTACT GENERATION
        //
        // Pick the polygon face most opposed to the contact normal (the face
        // the capsule presses on). If the capsule core is nearly parallel to
        // that face, clip the core to the face span to produce a stable two
        // point manifold (the resting case). Otherwise fall back to a single
        // contact at the capsule's deepest surface point.
        // =====================================================

        const std::size_t vertexCount = polygon.Vertices.size();

        std::size_t referenceIndex = 0;

        float bestAlignment = -std::numeric_limits<float>::max();

        for (std::size_t i = 0; i < polygon.Normals.size(); ++i)
        {
            const float alignment = Vector2::Dot(polygon.Normals[i], manifold.Normal * -1.0f);

            if (alignment > bestAlignment)
            {
                bestAlignment = alignment;

                referenceIndex = i;
            }
        }

        const Vector2 faceStart = polygon.Vertices[referenceIndex];

        const Vector2 faceEnd = polygon.Vertices[(referenceIndex + 1) % vertexCount];

        Vector2 faceTangent = faceEnd - faceStart;

        const float faceTangentLengthSquared = faceTangent.LengthSquared();

        bool generatedTwoPoints = false;

        if (faceTangentLengthSquared > epsilon)
        {
            faceTangent *= 1.0f / std::sqrt(faceTangentLengthSquared);

            const Vector2 core = pointB - pointA;

            const float coreLength = core.Length();

            // Parallel when the core has almost no component along the normal.

            const bool nearlyParallel =
                coreLength > epsilon &&
                (std::abs(Vector2::Dot(core, manifold.Normal)) / coreLength) < 0.05f;

            if (nearlyParallel)
            {
                const float faceLow = Vector2::Dot(faceStart, faceTangent);

                const float faceHigh = Vector2::Dot(faceEnd, faceTangent);

                const float projectionA = Vector2::Dot(pointA, faceTangent);

                const float projectionB = Vector2::Dot(pointB, faceTangent);

                if (std::abs(projectionB - projectionA) > epsilon)
                {
                    const float parameterAtLow = (faceLow - projectionA) / (projectionB - projectionA);

                    const float parameterAtHigh = (faceHigh - projectionA) / (projectionB - projectionA);

                    const float clipLow = std::clamp(std::min(parameterAtLow, parameterAtHigh), 0.0f, 1.0f);

                    const float clipHigh = std::clamp(std::max(parameterAtLow, parameterAtHigh), 0.0f, 1.0f);

                    if (clipHigh - clipLow > epsilon)
                    {
                        const Vector2 coreLowPoint = pointA + (pointB - pointA) * clipLow;

                        const Vector2 coreHighPoint = pointA + (pointB - pointA) * clipHigh;

                        // Move from the core to the capsule surface facing the polygon.

                        manifold.AddContactPoint(coreLowPoint + manifold.Normal * radius);

                        manifold.AddContactPoint(coreHighPoint + manifold.Normal * radius);

                        generatedTwoPoints = true;
                    }
                }
            }
        }

        if (!generatedTwoPoints)
        {
            // Single contact at the capsule surface point deepest toward the polygon.

            const float d0 = Vector2::Dot(pointA, manifold.Normal);

            const float d1 = Vector2::Dot(pointB, manifold.Normal);

            const Vector2 deepestCore = (d0 >= d1) ? pointA : pointB;

            manifold.AddContactPoint(deepestCore + manifold.Normal * radius);
        }

        return true;
    }


    bool PhysicsWorld2D::PolygonVsPolygon(PolygonCollider2D& a, PolygonCollider2D& b, CollisionManifold2D& manifold) const
    {
        const Polygon2D polygonA = a.GetWorldPolygon();

        const Polygon2D polygonB = b.GetWorldPolygon();

        if (polygonA.Vertices.size() < 3 || polygonB.Vertices.size() < 3)
        {
            return false;
        }

        const PolygonSATResult2D sat = TestPolygonSAT(polygonA, polygonB);

        if (!sat.Overlapping)
        {
            return false;
        }

        manifold.A = &a;

        manifold.B = &b;

        manifold.Normal = sat.Axis;

        manifold.Penetration = sat.Overlap;

        manifold.IsTrigger = a.IsTrigger() || b.IsTrigger();

        manifold.ClearContacts();

        Vector2 contacts[2];

        const std::size_t contactCount = BuildPolygonContactPoints(polygonA, polygonB, sat, contacts);

        for (std::size_t i = 0; i < contactCount; ++i)
        {
            manifold.AddContactPoint(contacts[i]);
        }

        // FALLBACK

        if (manifold.ContactCount == 0)
        {
            const Vector2 pointA = GetPolygonSupportPoint(polygonA, manifold.Normal);

            const Vector2 pointB = GetPolygonSupportPoint(polygonB, manifold.Normal * -1.0);

            manifold.AddContactPoint((pointA + pointB) * 0.5f);
        }

        return true;
    }


    bool PhysicsWorld2D::PolygonVsBox(PolygonCollider2D& polygonCollider, BoxCollider2D& boxCollider, CollisionManifold2D& manifold) const
    {
        const Polygon2D polygonA = polygonCollider.GetWorldPolygon();

        const Polygon2D polygonB = OrientedBoxToPolygon(boxCollider.GetWorldOrientedBox());

        if (polygonA.Vertices.size() < 3)
        {
            return false;
        }

        const PolygonSATResult2D sat = TestPolygonSAT(polygonA, polygonB);

        if (!sat.Overlapping)
        {
            return false;
        }

        manifold.A = &polygonCollider;

        manifold.B = &boxCollider;

        manifold.Normal = sat.Axis;

        manifold.Penetration = sat.Overlap;

        manifold.IsTrigger = polygonCollider.IsTrigger() || boxCollider.IsTrigger();

        manifold.ClearContacts();

        Vector2 contacts[2];

        const std::size_t contactCount = BuildPolygonContactPoints(polygonA, polygonB, sat, contacts);

        for (std::size_t i = 0; i < contactCount; ++i)
        {
            manifold.AddContactPoint(contacts[i]);
        }

        if (manifold.ContactCount == 0)
        {
            const Vector2 pointA = GetPolygonSupportPoint(polygonA, manifold.Normal);

            const Vector2 pointB = GetPolygonSupportPoint(polygonB, manifold.Normal * -1.0f);

            manifold.AddContactPoint((pointA + pointB) * 0.5f);
        }

        return true;
    }


    bool PhysicsWorld2D::PolygonVsCircle(PolygonCollider2D& polygonCollider, CircleCollider2D& circle, CollisionManifold2D& manifold) const
    {
        const Polygon2D polygon = polygonCollider.GetWorldPolygon();

        if (polygon.Vertices.size() < 3)
        {
            return false;
        }

        const Vector2 circleCenter = circle.GetWorldCenter();

        const float circleRadius = circle.GetWorldRadius();

        float minimumOverlap = std::numeric_limits<float>::max();

        Vector2 minimumAxis{0.0f, 0.0f};

        // POLYGON EDGE NORMAL

        for (const Vector2& axis : polygon.Normals)
        {
            if (axis.LengthSquared() <= 0.000001f)
            {
                continue;
            }

            float minPolygon = 0.0f;
            float maxPolygon = 0.0f;

            float minCircle = 0.0f;
            float maxCircle = 0.0f;

            ProjectPolygon(polygon, axis, minPolygon, maxPolygon);

            ProjectCircle(circleCenter, circleRadius, axis, minCircle, maxCircle);

            const float overlap = std::min(maxPolygon, maxCircle) - std::max(minPolygon, minCircle);

            if (overlap <= 0.0f)
            {
                return false;
            }

            if (overlap < minimumOverlap)
            {
                minimumOverlap = overlap;

                minimumAxis = axis;
            }
        }

        // CLOSEST VERTEX AXIS

        const Vector2 closestVertex = FindClosestPolygonVertex(polygon, circleCenter);

        Vector2 vertexAxis = circleCenter - closestVertex;

        const float vertexAxisLengthSquared = vertexAxis.LengthSquared();

        if ( vertexAxisLengthSquared > 0.000001f)
        {
            vertexAxis *= 1.0f / std::sqrt(vertexAxisLengthSquared);

            float minPolygon = 0.0f;
            float maxPolygon = 0.0f;

            float minCircle = 0.0f;
            float maxCircle = 0.0f;

            ProjectPolygon(polygon, vertexAxis, minPolygon, maxPolygon);

            ProjectCircle(circleCenter, circleRadius, vertexAxis, minCircle, maxCircle);

            const float overlap = std::min(maxPolygon, maxCircle) - std::max(minPolygon, minCircle);

            if (overlap <= 0.0f)
            {
                return false;
            }

            if (overlap < minimumOverlap)
            {
                minimumOverlap = overlap;

                minimumAxis = vertexAxis;
            }
        }

        if (minimumOverlap == std::numeric_limits<float>::max())
        {
            return false;
        }

        // NORMAL A -> B
        //
        // A = Polygon
        // B = Circle

        const Vector2 polygonCenter = GetPolygonCenter(polygon);

        if (Vector2::Dot(circleCenter - polygonCenter, minimumAxis) < 0.0f)
        {
            minimumAxis *= -1.0f;
        }

        // MANIFOLD

        manifold.A = &polygonCollider;

        manifold.B = &circle;

        manifold.Normal = minimumAxis;

        manifold.Penetration = minimumOverlap;

        manifold.IsTrigger = polygonCollider.IsTrigger() || circle.IsTrigger();

        manifold.ClearContacts();

        // CONTACT

        const Vector2 pointOnPolygon = GetPolygonSupportPoint(polygon, manifold.Normal);

        const Vector2 pointOnCircle = circleCenter - manifold.Normal * circleRadius;

        manifold.AddContactPoint((pointOnPolygon + pointOnCircle) * 0.5f);

        return true;
    }


    void PhysicsWorld2D::ProjectOrientedBox(const OrientedBox2D& box, const Vector2& axis, float& outMin, float& outMax) const
    {
        outMin = Vector2::Dot(box.Vertices[0], axis);

        outMax = outMin;

        for (std::size_t i = 1; i < 4; ++i)
        {
            const float projection = Vector2::Dot(box.Vertices[i], axis);

            outMin = std::min(outMin, projection);

            outMax = std::max(outMax, projection);
        }
    }


    bool PhysicsWorld2D::TestOBBAxis(const OrientedBox2D& a, const OrientedBox2D& b, const Vector2& axis, float& outOverlap) const
    {
        float minA = 0.0f;
        float maxA = 0.0f;

        float minB = 0.0f;
        float maxB = 0.0f;

        ProjectOrientedBox(a, axis, minA, maxA);

        ProjectOrientedBox(b, axis, minB, maxB);

        outOverlap = std::min(maxA, maxB) - std::max(minA, minB);

        return outOverlap > 0.0f;
    }


    Vector2 PhysicsWorld2D::GetOBBSupportPoint(const OrientedBox2D& box, const Vector2& direction) const
    {
        std::size_t bestIndex = 0;

        float bestProjection = Vector2::Dot(box.Vertices[0], direction);

        for (std::size_t i = 1; i < 4; ++i)
        {
            const float projection = Vector2::Dot(box.Vertices[i], direction);

            if (projection > bestProjection)
            {
                bestProjection = projection;

                bestIndex = i;
            }
        }

        return box.Vertices[bestIndex];
    }


    std::size_t PhysicsWorld2D::ClipSegmentToSpan(const Vector2& p0, const Vector2& p1, const Vector2& origin, const Vector2& tangent, float halfLength, Vector2 outPoints[2]) const
    {
        const Vector2 delta = p1 - p0;

        const float s0 = Vector2::Dot(p0 - origin, tangent);

        const float sd = Vector2::Dot(delta, tangent);

        float tMin = 0.0f;

        float tMax = 1.0f;

        constexpr float epsilon = 0.000001f;

        // =========================================================
        // SEGMENT IS PARALLEL TO THE SPAN PLANES
        // =========================================================

        if (std::abs(sd) <= epsilon)
        {
            if (s0 < -halfLength || s0 > halfLength)
            {
                return 0;
            }
        }

        // =========================================================
        // CLIP SEGMENT
        // =========================================================

        else {
            float t1 = (-halfLength - s0) / sd;

            float t2 = (halfLength - s0) / sd;

            if (t1 > t2)
            {
                std::swap(t1, t2);
            }

            tMin = std::max(tMin, t1);

            tMax = std::min(tMax, t2);

            if (tMin > tMax)
            {
                return 0;
            }
        }

        // =========================================================
        // OUTPUT
        // =========================================================

        std::size_t count = 0;

        outPoints[count++] = p0 + delta * tMin;

        if (tMax - tMin > epsilon)
        {
            outPoints[count++] = p0 + delta * tMax;
        }

        return count;
    }


    std::size_t PhysicsWorld2D::BuildOBBContactPoints(const OrientedBox2D& a, const OrientedBox2D& b, const Vector2& normal, int minimumAxisIndex, Vector2 outContacts[2]) const
    {
        // =========================================================
        // CHOOSE REFERENCE / INCIDENT BOX
        // =========================================================

        const bool referenceIsA = minimumAxisIndex < 2;

        const OrientedBox2D reference = referenceIsA ? a : b;

        const OrientedBox2D& incident = referenceIsA ? b : a;

        // Normal from reference toward incident.
        const Vector2 referenceToIncident = referenceIsA ? normal : normal * -1.0f;

        // =========================================================
        // CHOOSE REFERENCE FACE
        // =========================================================

        const float refDotX = Vector2::Dot(referenceToIncident, reference.AxisX);

        const float refDotY = Vector2::Dot(referenceToIncident, reference.AxisY);

        Vector2 referenceNormal;
        Vector2 referenceTangent;

        Vector2 referenceFaceCenter;

        float referenceHalfLength = 0.0f;

        // ---------------------------------------------------------
        // Reference X face
        // ---------------------------------------------------------

        if (std::abs(refDotX) > std::abs(refDotY))
        {
            const float sign = refDotX >= 0.0f ? 1.0f : -1.0f;

            referenceNormal = reference.AxisX * sign;

            referenceTangent = reference.AxisY;

            referenceFaceCenter = reference.Center + referenceNormal * reference.HalfExtents.X;

            referenceHalfLength = reference.HalfExtents.Y;
        }

        // ---------------------------------------------------------
        // Reference Y face
        // ---------------------------------------------------------

        else {
            const float sign = refDotY >= 0.0f ? 1.0f : -1.0f;

            referenceNormal = reference.AxisY * sign;

            referenceTangent = reference.AxisX;

            referenceFaceCenter = reference.Center + referenceNormal * reference.HalfExtents.Y;

            referenceHalfLength = reference.HalfExtents.X;
        }

        // =========================================================
        // FIND INCIDENT FACE
        // =========================================================

        const Vector2 incidentNormals[4]
        {
            incident.AxisX,

            incident.AxisX * -1.0f,

            incident.AxisY,

            incident.AxisY * -1.0f
        };

        int incidentNormalIndex = 0;

        float smallestDot = Vector2::Dot(incidentNormals[0], referenceNormal);

        for (int i = 1; i < 4; ++i)
        {
            const float value = Vector2::Dot(incidentNormals[i], referenceNormal);

            if (value < smallestDot)
            {
                smallestDot = value;

                incidentNormalIndex = i;
            }
        }

        Vector2 incidentFaceCenter;
        Vector2 incidentTangent;

        float incidentHalfLength = 0.0f;

        // =========================================================
        // BUILD INCIDENT FACE
        // =========================================================

        switch (incidentNormalIndex)
        {
            // +X face
            case 0:
            {
                incidentFaceCenter = incident.Center + incident.AxisX * incident.HalfExtents.X;

                incidentTangent = incident.AxisY;

                incidentHalfLength = incident.HalfExtents.Y;

                break;
            }

            // -X face
            case 1:
            {
                incidentFaceCenter = incident.Center - incident.AxisX * incident.HalfExtents.X;

                incidentTangent = incident.AxisY;

                incidentHalfLength = incident.HalfExtents.Y;

                break;
            }

            // +Y face
            case 2:
            {
                incidentFaceCenter = incident.Center + incident.AxisY * incident.HalfExtents.Y;

                incidentTangent = incident.AxisX;

                incidentHalfLength = incident.HalfExtents.X;

                break;
            }

            // -Y face
            default:
            {
                incidentFaceCenter = incident.Center - incident.AxisY * incident.HalfExtents.Y;

                incidentTangent = incident.AxisX;

                incidentHalfLength = incident.HalfExtents.X;

                break;
            }
        }

        // =========================================================
        // INCIDENT EDGE ENDPOINTS
        // =========================================================

        const Vector2 incidentP0 = incidentFaceCenter - incidentTangent * incidentHalfLength;

        const Vector2 incidentP1 = incidentFaceCenter + incidentTangent * incidentHalfLength;

        // =========================================================
        // CLIP INCIDENT EDGE
        // =========================================================

        Vector2 clipped[2];

        const std::size_t clippedCount = ClipSegmentToSpan(incidentP0, incidentP1, referenceFaceCenter, referenceTangent, referenceHalfLength, clipped);

        // =========================================================
        // KEEP POINTS BEHIND REFERENCE FACE
        // =========================================================

        std::size_t contactCount =  0;

        constexpr float contactTolerance = 0.001f;

        for (std::size_t i = 0; i < clippedCount; ++i)
        {
            const float separation = Vector2::Dot(clipped[i] - referenceFaceCenter, referenceNormal);

            // Poin is outside the reference face.
            if (separation > contactTolerance)
            {
                continue;
            }

            // Put the contact approximately halfway through the penetration region;
            outContacts[contactCount++] = clipped[i] - referenceNormal * (separation * 0.5f);

            if (contactCount == 2)
            {
                break;
            }
        }

        return contactCount;
    }


    Vector2 PhysicsWorld2D::WorldPointToOBBLocal(const OrientedBox2D& box, const Vector2& worldPoint) const
    {
        const Vector2 delta = worldPoint - box.Center;

        return 
        {
            Vector2::Dot(delta, box.AxisX),

            Vector2::Dot(delta, box.AxisY)
        };
    }


    Vector2 PhysicsWorld2D::OBBLocalPointToWorld(const OrientedBox2D& box, const Vector2& localPoint) const
    {
        return box.Center + box.AxisX * localPoint.X + box.AxisY * localPoint.Y;
    }


    Vector2 PhysicsWorld2D::OBBLocalDirectionToWorld(const OrientedBox2D& box, const Vector2& localDirection) const
    {
        return box.AxisX * localDirection.X + box.AxisY * localDirection.Y;
    }


    bool PhysicsWorld2D::GenerateManifold(Collider2D& a, Collider2D& b, CollisionManifold2D& manifold) const
    {
        const ColliderShape2D shapeA = a.GetShape();

        const ColliderShape2D shapeB = b.GetShape();

        // Box Vs Box

        if (shapeA == ColliderShape2D::Box && shapeB == ColliderShape2D::Box)
        {
            return BoxVsBox(static_cast<BoxCollider2D&>(a), static_cast<BoxCollider2D&>(b), manifold);
        }

        // Circle Vs Circle

        if (shapeA == ColliderShape2D::Circle && shapeB == ColliderShape2D::Circle)
        {
            return CircleVsCircle(static_cast<CircleCollider2D&>(a), static_cast<CircleCollider2D&>(b), manifold);
        }

        // Box Vs Circle

        if (shapeA == ColliderShape2D::Box && shapeB == ColliderShape2D::Circle)
        {
            return BoxVsCircle(static_cast<BoxCollider2D&>(a), static_cast<CircleCollider2D&>(b), manifold);
        }

        // Circle Vs Box

        if (shapeA == ColliderShape2D::Circle && shapeB == ColliderShape2D::Box)
        {
            const bool hit = BoxVsCircle(static_cast<BoxCollider2D&>(b), static_cast<CircleCollider2D&>(a), manifold);

            if (hit)
            {
                std::swap(manifold.A, manifold.B);

                manifold.Normal *= -1.0f;
            }

            return hit;
        }

        // Capsule Vs Circle

        if (shapeA == ColliderShape2D::Capsule && shapeB == ColliderShape2D::Circle)
        {
            return CapsuleVsCircle(static_cast<CapsuleCollider2D&>(a), static_cast<CircleCollider2D&>(b), manifold);
        }

        // Circle Vs Capsule

        if (shapeA == ColliderShape2D::Circle && shapeB == ColliderShape2D::Capsule)
        {
            const bool hit = CapsuleVsCircle(static_cast<CapsuleCollider2D&>(b), static_cast<CircleCollider2D&>(a), manifold);

            if (hit)
            {
                std::swap(manifold.A, manifold.B);

                manifold.Normal *= -1.0f;
            }

            return hit;
        }

        // Capsule Vs Capsule

        if (shapeA == ColliderShape2D::Capsule && shapeB == ColliderShape2D::Capsule)
        {
            return CapsuleVsCapsule(static_cast<CapsuleCollider2D&>(a), static_cast<CapsuleCollider2D&>(b), manifold);
        }

        // Polygon Vs Polygon

        if (shapeA == ColliderShape2D::Polygon && shapeB == ColliderShape2D::Polygon)
        {
            return PolygonVsPolygon(static_cast<PolygonCollider2D&>(a), static_cast<PolygonCollider2D&>(b), manifold);
        }

        // Polygon Vs Box

        if (shapeA == ColliderShape2D::Polygon && shapeB == ColliderShape2D::Box)
        {
            return PolygonVsBox(static_cast<PolygonCollider2D&>(a), static_cast<BoxCollider2D&>(b), manifold);
        }

        // Box Vs Polygon

        if (shapeA == ColliderShape2D::Box && shapeB == ColliderShape2D::Polygon)
        {
            const bool hit = PolygonVsBox(static_cast<PolygonCollider2D&>(b), static_cast<BoxCollider2D&>(a), manifold);

            if (hit)
            {
                std::swap(manifold.A, manifold.B);

                manifold.Normal *= -1.0f;
            }

            return hit;
        }

        // Polygon Vs Circle

        if (shapeA == ColliderShape2D::Polygon && shapeB == ColliderShape2D::Circle)
        {
            return PolygonVsCircle(static_cast<PolygonCollider2D&>(a), static_cast<CircleCollider2D&>(b), manifold);
        }

        // Circle Vs Polygon

        if (shapeA == ColliderShape2D::Circle && shapeB == ColliderShape2D::Polygon)
        {
            const bool hit = PolygonVsCircle(static_cast<PolygonCollider2D&>(b), static_cast<CircleCollider2D&>(a), manifold);

            if (hit)
            {
                std::swap(manifold.A, manifold.B);

                manifold.Normal *= -1.0f;
            }

            return hit;
        }

        // Capsule Vs Box

        if (shapeA == ColliderShape2D::Capsule && shapeB == ColliderShape2D::Box)
        {
            return CapsuleVsBox(static_cast<CapsuleCollider2D&>(a), static_cast<BoxCollider2D&>(b), manifold);
        }

        // Box Vs Capsule

        if (shapeA == ColliderShape2D::Box && shapeB == ColliderShape2D::Capsule)
        {
            const bool hit = CapsuleVsBox(static_cast<CapsuleCollider2D&>(b), static_cast<BoxCollider2D&>(a), manifold);

            if (hit)
            {
                std::swap(manifold.A, manifold.B);

                manifold.Normal *= -1.0f;
            }

            return hit;
        }

        // Capsule Vs Polygon

        if (shapeA == ColliderShape2D::Capsule && shapeB == ColliderShape2D::Polygon)
        {
            return CapsuleVsPolygon(static_cast<CapsuleCollider2D&>(a), static_cast<PolygonCollider2D&>(b), manifold);
        }

        // Polygon Vs Capsule

        if (shapeA == ColliderShape2D::Polygon && shapeB == ColliderShape2D::Capsule)
        {
            const bool hit = CapsuleVsPolygon(static_cast<CapsuleCollider2D&>(b), static_cast<PolygonCollider2D&>(a), manifold);

            if (hit)
            {
                std::swap(manifold.A, manifold.B);

                manifold.Normal *= -1.0f;
            }

            return hit;
        }

        return false;
    }

    void PhysicsWorld2D::PublishPairEvents()
    {
        // ---------------------------------
        // Begin + Stay
        // ---------------------------------

        for (const ColliderPair2D& pair : m_CurrentOverlaps)
        {
            if (m_PreviousOverlaps.find(pair) == m_PreviousOverlaps.end())
            {
                PublishBegin(pair);
            }
            else
            {
                PublishStay(pair);
            }
        }

        // ---------------------------------
        // End
        // ---------------------------------

        for (const ColliderPair2D& pair : m_PreviousOverlaps)
        {
            if (m_CurrentOverlaps.find(pair) == m_CurrentOverlaps.end())
            {
                PublishEnd(pair);
            }
        }
    }

    void PhysicsWorld2D::PublishBegin(const ColliderPair2D& pair)
    {
        if (!m_Scene || !pair.A || !pair.B)
        {
            return;
        }

        Entity* entityA = pair.A->GetOwner();

        Entity* entityB = pair.B->GetOwner();

        if (!entityA || !entityB)
        {
            return;
        }

        CollisionBeginEvent2D event;

        event.A = m_Scene->CreateHandle(entityA);

        event.B = m_Scene->CreateHandle(entityB);

        event.ColliderA = pair.A;

        event.ColliderB = pair.B;

        event.IsTrigger = pair.A->IsTrigger() || pair.B->IsTrigger();

        m_Scene->GetEventBus().Publish<CollisionBeginEvent2D>(event);
    }

    void PhysicsWorld2D::PublishStay(const ColliderPair2D& pair)
    {
        if (!m_Scene || !pair.A || !pair.B)
        {
            return;
        }

        Entity* entityA = pair.A->GetOwner();

        Entity* entityB = pair.B->GetOwner();

        if (!entityA || !entityB)
        {
            return;
        }

        CollisionStayEvent2D event;

        event.A = m_Scene->CreateHandle(entityA);

        event.B = m_Scene->CreateHandle(entityB);

        event.ColliderA = pair.A;

        event.ColliderB = pair.B;

        event.IsTrigger = pair.A->IsTrigger() || pair.B->IsTrigger();

        m_Scene->GetEventBus().Publish<CollisionStayEvent2D>(event);
    }

    void PhysicsWorld2D::PublishEnd(const ColliderPair2D& pair)
    {
        if (!m_Scene || !pair.A || !pair.B)
        {
            return;
        }

        Entity* entityA = pair.A->GetOwner();

        Entity* entityB = pair.B->GetOwner();

        if (!entityA || !entityB)
        {
            return;
        }

        CollisionEndEvent2D event;

        event.A = m_Scene->CreateHandle(entityA);

        event.B = m_Scene->CreateHandle(entityB);

        event.ColliderA = pair.A;

        event.ColliderB = pair.B;

        event.WasTrigger = pair.A->IsTrigger() || pair.B->IsTrigger();

        m_Scene->GetEventBus().Publish<CollisionEndEvent2D>(event);
    }

    std::size_t PhysicsWorld2D::GetActiveColliderCount() const
    {
        return m_ActiveColliders.size();
    }

    std::size_t PhysicsWorld2D::GetCurrentOverlapCount() const
    {
        return m_CurrentOverlaps.size();
    }

    void PhysicsWorld2D::SetGravity(const Vector2& gravity)
    {
        m_Settings.Gravity = gravity;
    }

    const Vector2& PhysicsWorld2D::GetGravity() const
    {
        return m_Settings.Gravity;
    }

    void PhysicsWorld2D::IntegrateDynamicBody(Rigidbody2D& body, TransformComponent& transform, float deltaTime)
    {
        // -------------------------------
        // Force acceleration
        // -------------------------------

        Vector2 acceleration = body.GetAccumulatedForce() * body.GetInverseMass();

        // -------------------------------
        // Gravity
        // -------------------------------

        acceleration += m_Settings.Gravity * body.GetGravityScale();

        const Vector2 startPosition = transform.GetWorldTransform().Position;

        body.SetPreviousPosition(startPosition);

        // -------------------------------
        // Update velocity
        // -------------------------------

        Vector2 velocity = body.GetVelocity();

        velocity += acceleration * deltaTime;

        // -------------------------------
        // Linear damping
        // -------------------------------

        const float dampingFactor = 1.0f / (1.0f + body.GetLinearDamping() * deltaTime);

        velocity *= dampingFactor;

        body.SetVelocityFromPhysics(velocity);

        // -------------------------------
        // LINEAR MOVEMENT
        // -------------------------------

        transform.Translate(velocity * deltaTime);


        // -------------------------------
        // ANGULAR ACCELERATION
        // -------------------------------

        const float angularAcceleration = body.GetAccumulatedTorque() * body.GetInverseInertia();

        // -------------------------------
        // ANGULAR VELOCITY

        // radians / second
        // -------------------------------

        float angularVelocity = body.GetAngularVelocity();

        angularVelocity += angularAcceleration * deltaTime;

        // -------------------------------
        // ANGULAR DAMPING
        // -------------------------------

        const float angularDampingFactor = 1.0f / (1.0f + body.GetAngularDamping() * deltaTime);

        angularVelocity *= angularDampingFactor;

        body.SetAngularVelocityFromPhysics(angularVelocity);


        // -------------------------------
        // ROTATION INTEGRATION
        // -------------------------------

        constexpr float radiansToDegrees = 57.29577951308232f;

        const float rotationDeltaDegrees = angularVelocity * deltaTime * radiansToDegrees;

        transform.RotateBy(rotationDeltaDegrees);

        // --------------------------------
        // ACCUMULATORS LAST ONE FIXED STEP
        // --------------------------------

        body.ClearForces();

        body.ClearTorque();
    }

    void PhysicsWorld2D::IntegrateKinematicBody(Rigidbody2D& body, TransformComponent& transform, float deltaTime)
    {
        // --------------------------------
        // STORE START OF FIXED STEP
        // --------------------------------

        const Vector2 startPosition = transform.GetWorldTransform().Position;

        body.SetPreviousPosition(startPosition);

        // --------------------------------
        // LINEAR KINEMATIC MOTION
        // --------------------------------

        transform.Translate(body.GetVelocity() * deltaTime);

        // --------------------------------
        // ANGULAR KINEMATIC MOTION
        // --------------------------------

        constexpr float radiansToDegrees = 57.29577951308232f;

        transform.RotateBy(body.GetAngularVelocity() * deltaTime * radiansToDegrees);

        // Kinematic bodies do not react to force/torque.

        body.ClearForces();

        body.ClearTorque();
    }

    void PhysicsWorld2D::IntegrateBodies(float deltaTime)
    {
        if (!m_Scene)
        {
            return;
        }

        for (Entity* root : m_Scene->GetRootEntities())
        {
            IntegrateEntityRecursive(root, deltaTime);
        }
    }

    void PhysicsWorld2D::ApplyPositionalCorrection(const CollisionManifold2D& manifold, Rigidbody2D* bodyA, Rigidbody2D* bodyB, TransformComponent& transformA, TransformComponent& transformB)
    {
        const float inverseMassA = GetSolverInverseMass(bodyA);

        const float inverseMassB = GetSolverInverseMass(bodyB);

        const float totalInverseMass = inverseMassA + inverseMassB;

        if (totalInverseMass <= 0.0f)
        {
            return;
        }


        const float correctedPenetration = std::max(manifold.Penetration - m_Settings.PositionSlop, 0.0f);

        const Vector2 correction = manifold.Normal * (correctedPenetration * m_Settings.PositionCorrectionPercent / totalInverseMass);

        if (inverseMassA > 0.0f)
        {
            transformA.Translate(correction * -inverseMassA);
        }

        if (inverseMassB > 0.0f)
        {
            transformB.Translate(correction * inverseMassB);
        }
    }

    void PhysicsWorld2D::ApplyVelocityResponse(CollisionManifold2D& manifold, Rigidbody2D* bodyA, Rigidbody2D* bodyB)
    {
        // =========================================================
        // SOLVER MASS / INERTIA
        // =========================================================

        const float inverseMassA = GetSolverInverseMass(bodyA);

        const float inverseMassB = GetSolverInverseMass(bodyB);

        const float inverseInertiaA = GetSolverInverseInertia(bodyA);

        const float inverseInertiaB = GetSolverInverseInertia(bodyB);

        if (inverseMassA + inverseMassB + inverseInertiaA + inverseInertiaB <= 0.0f)
        {
            return;
        }

        // =========================================================
        // OWNERS / TRANSFORMS
        // =========================================================

        Entity* entityA = manifold.A ? manifold.A->GetOwner() : nullptr;

        Entity* entityB = manifold.B ? manifold.B->GetOwner() : nullptr;

        if (!entityA || !entityB)
        {
            return;
        }

        TransformComponent* transformA = entityA->GetComponent<TransformComponent>();

        TransformComponent* transformB = entityB->GetComponent<TransformComponent>();

        if (!transformA || !transformB)
        {
            return;
        }

        // =========================================================
        // CENTER OF MASS
        // =========================================================

        const Vector2 centerA = transformA->GetWorldTransform().Position;

        const Vector2 centerB = transformB->GetWorldTransform().Position;

        // =========================================================
        // LOCAL SOLVER VELOCITIES
        // =========================================================

        Vector2 velocityA = bodyA ? bodyA->GetVelocity() : Vector2{0.0f, 0.0f};

        Vector2 velocityB = bodyB ? bodyB->GetVelocity() : Vector2{0.0f, 0.0f};

        float angularVelocityA = bodyA ? bodyA->GetAngularVelocity() : 0.0f;

        float angularVelocityB = bodyB ? bodyB->GetAngularVelocity() : 0.0f;

        // =========================================================
        // RESTITUTION
        // =========================================================

        const float staticFriction = CombineStaticFriction(*manifold.A, *manifold.B);

        const float dynamicFriction = CombineDynamicFriction(*manifold.A, *manifold.B);

        constexpr float epsilon = 0.000001f;

        if (manifold.ContactCount > CollisionManifold2D::MaxContactPoints)
        {
            std::cerr << "ERROR: Invalid manifold ContactCount: " << manifold.ContactCount << '\n';

            return;
        }

        // =========================================================
        // SOLVE CONTACTS SEQUENTIALLY
        // =========================================================

        for (std::size_t contactIndex = 0; contactIndex < manifold.ContactCount; ++contactIndex)
        {
            const Vector2 contactPoint = manifold.ContactPoints[contactIndex];

            const Vector2 rA = contactPoint - centerA;

            const Vector2 rB = contactPoint - centerB;

            // =====================================================
            // VELOCITY AT CONTACT BEFORE NORMAL SOLVE
            // =====================================================

            Vector2 contactVelocityA = GetVelocityAtPoint(velocityA, angularVelocityA, rA);

            Vector2 contactVelocityB = GetVelocityAtPoint(velocityB, angularVelocityB, rB);

            Vector2 relativeVelocity = contactVelocityB - contactVelocityA;

            const float velocityAlongNormal = Vector2::Dot(relativeVelocity, manifold.Normal);

            const float raCrossNormal = Cross2D(rA, manifold.Normal);

            const float rbCrossNormal = Cross2D(rB, manifold.Normal);

            const float normalDenominator = 
                inverseMassA + inverseMassB + raCrossNormal * raCrossNormal *
                inverseInertiaA + rbCrossNormal * rbCrossNormal *inverseInertiaB;

            
                

            if (normalDenominator > epsilon)
            {
                const float restitutionBias = manifold.RestitutionBiases[contactIndex];
                
                float deltaNormalImpulse = -(velocityAlongNormal - restitutionBias) / normalDenominator;

                const float oldNormalImpulse = manifold.AccumulatedNormalImpulses[contactIndex];

                const float newNormalImpulse = std::max(oldNormalImpulse + deltaNormalImpulse, 0.0f);

                deltaNormalImpulse = newNormalImpulse - oldNormalImpulse;

                manifold.AccumulatedNormalImpulses[contactIndex] = newNormalImpulse;

                const Vector2 normalImpulse = manifold.Normal * deltaNormalImpulse;

                // =============================================
                // BODY A
                // =============================================

                if (bodyA && inverseMassA > 0.0f)
                {
                    velocityA -= normalImpulse * inverseMassA;
                }

                if (bodyA && inverseInertiaA > 0.0f)
                {
                    angularVelocityA -= Cross2D(rA, normalImpulse) * inverseInertiaA;
                }

                // =============================================
                // BODY B
                // =============================================

                if (bodyB && inverseMassB > 0.0f)
                {
                    velocityB += normalImpulse * inverseMassB;
                }

                if (bodyB && inverseInertiaB > 0.0f)
                {
                    angularVelocityB += Cross2D(rB, normalImpulse) * inverseInertiaB;
                }
                
            }

            // =====================================================
            // RECOMPUTE CONTACT VELOCITY AFTER NORMAL IMPULSE
            //
            // The normal impulse changed both linear and angular
            // velocity. Friction must use the updated state.
            // =====================================================

            contactVelocityA = GetVelocityAtPoint(velocityA, angularVelocityA, rA);

            contactVelocityB = GetVelocityAtPoint(velocityB, angularVelocityB, rB);

            relativeVelocity = contactVelocityB - contactVelocityA;

            // =====================================================
            // TANGENT
            // =====================================================

            const Vector2 tangent
            {
                -manifold.Normal.Y,
                manifold.Normal.X
            };

            // Surface velocity (conveyor belts). Friction normally drives the
            // tangential relative velocity to zero; when a collider carries a
            // surface velocity we instead drive it toward that belt speed by
            // subtracting the desired relative tangential velocity from the
            // measured one. Zero surface velocities leave behaviour unchanged.

            const Vector2 surfaceVelocityA = manifold.A ? manifold.A->GetSurfaceVelocity() : Vector2{0.0f, 0.0f};

            const Vector2 surfaceVelocityB = manifold.B ? manifold.B->GetSurfaceVelocity() : Vector2{0.0f, 0.0f};

            const float surfaceRelativeTangent = Vector2::Dot(surfaceVelocityB - surfaceVelocityA, tangent);

            const float velocityAlongTangent = Vector2::Dot(relativeVelocity, tangent) - surfaceRelativeTangent;

            // =====================================================
            // TANGENTIAL EFFECTIVE MASS
            // =====================================================

            const float raCrossTangent = Cross2D(rA, tangent);

            const float rbCrossTangent = Cross2D(rB, tangent);

            const float tangentDenominator =
                inverseMassA + inverseMassB + raCrossTangent * raCrossTangent *
                inverseInertiaA + rbCrossTangent * rbCrossTangent *inverseInertiaB;

            if (tangentDenominator <= epsilon)
            {
                continue;
            }

            // =====================================================
            // FRICTION IMPULSE CHANGE
            // =====================================================

            float deltaTangentImpulse = -velocityAlongTangent / tangentDenominator;

            const float oldTangentImpulse = manifold.AccumulatedTangentImpulses[contactIndex];

            const float candidateTangentImpulse = oldTangentImpulse + deltaTangentImpulse;

            const float accumulatedNormalImpulse = manifold.AccumulatedNormalImpulses[contactIndex];

            const float staticLimit = staticFriction * accumulatedNormalImpulse;

            float newTangentImpulse = 0.0f;

            // -----------------------------------------------------
            // STATIC FRICTION
            //
            // Can friction completely cancel tangential motion?
            // -----------------------------------------------------

            if (std::abs(candidateTangentImpulse) <= staticLimit)
            {
                newTangentImpulse = candidateTangentImpulse;
            }

            // -----------------------------------------------------
            // DYNAMIC FRICTION
            // -----------------------------------------------------

            else {
                const float dynamicLimit = accumulatedNormalImpulse * dynamicFriction;

                newTangentImpulse = std::clamp(candidateTangentImpulse, -dynamicLimit, dynamicLimit);
            }

            deltaTangentImpulse = newTangentImpulse - oldTangentImpulse;

            manifold.AccumulatedTangentImpulses[contactIndex] = newTangentImpulse;
            
            const Vector2 frictionImpulse  = tangent * deltaTangentImpulse;

            // =====================================================
            // APPLY FRICTION TO BODY A
            // =====================================================

            if (bodyA && inverseMassA > 0.0f)
            {
                velocityA -= frictionImpulse * inverseMassA;
            }

            if (bodyA && inverseInertiaA > 0.0f)
            {
                angularVelocityA -= Cross2D(rA, frictionImpulse) * inverseInertiaA;
            }

            // =====================================================
            // APPLY FRICTION TO BODY B
            // =====================================================

            if (bodyB && inverseMassB > 0.0f)
            {
                velocityB += frictionImpulse * inverseMassB;
            }

            if (bodyB && inverseInertiaB > 0.0f)
            {
                angularVelocityB += Cross2D(rB, frictionImpulse) * inverseInertiaB;
            }
        }

        
        // =========================================================
        // COMMIT ONCE
        // =========================================================

        if (bodyA)
        {
            bodyA->SetVelocityFromPhysics(velocityA);

            bodyA->SetAngularVelocityFromPhysics(angularVelocityA);
        }

        if (bodyB)
        {
            bodyB->SetVelocityFromPhysics(velocityB);

            bodyB->SetAngularVelocityFromPhysics(angularVelocityB);
        }
    }

    float PhysicsWorld2D::CombineRestitution(const Collider2D& a, const Collider2D& b) const
    {
        return
            std::max(
                a.GetPhysicsMaterial().Restitution,

                b.GetPhysicsMaterial().Restitution
            );
    }

    float PhysicsWorld2D::CombineStaticFriction(const Collider2D& a, const Collider2D& b) const
    {
        return 
            std::sqrt(
                a.GetPhysicsMaterial().StaticFriction
                *
                b.GetPhysicsMaterial().StaticFriction
            );
    }

    float PhysicsWorld2D::CombineDynamicFriction(const Collider2D& a, const Collider2D& b) const
    {
        return 
            std::sqrt(
                a.GetPhysicsMaterial().DynamicFriction
                *
                b.GetPhysicsMaterial().DynamicFriction
            );
    }

    void PhysicsWorld2D::SetVelocityIterations(std::size_t iterations)
    {
        m_Settings.VelocityIterations = std::max<std::size_t>(1, iterations);
    }

    std::size_t PhysicsWorld2D::GetVelocityIterations() const
    {
        return m_Settings.VelocityIterations;
    }

    void PhysicsWorld2D::SetPositionIterations(std::size_t iterations)
    {
        m_Settings.PositionIterations = std::max<std::size_t>(1, iterations);
    }

    std::size_t PhysicsWorld2D::GetPositionIterations() const
    {
        return m_Settings.PositionIterations;
    }

    void PhysicsWorld2D::SolveVelocityContact(CollisionManifold2D& manifold)
    {
        if (!manifold.IsValid() || manifold.IsTrigger)
        {
            return;
        }

        Entity* entityA = manifold.A->GetOwner();

        Entity* entityB = manifold.B->GetOwner();

        if (!entityA || !entityB)
        {
            return;
        }

        Rigidbody2D* bodyA = entityA->GetComponent<Rigidbody2D>();

        Rigidbody2D* bodyB = entityB->GetComponent<Rigidbody2D>();

        const float inverseMassA = GetSolverInverseMass(bodyA);

        const float inverseMassB = GetSolverInverseMass(bodyB);

        const float inverseInertiaA = GetSolverInverseInertia(bodyA);

        const float inverseInertiaB = GetSolverInverseInertia(bodyB);

        if (inverseMassA + inverseMassB + inverseInertiaA + inverseInertiaB <= 0.0f)
        {
            return;
        }

        ApplyVelocityResponse(manifold, bodyA, bodyB);
    }

    void PhysicsWorld2D::SolveVelocityContacts()
    {
        for (std::size_t iteration = 0; iteration < m_Settings.VelocityIterations; ++iteration)
        {
            for (CollisionManifold2D& manifold : m_CurrentContacts)
            {
                SolveVelocityContact(manifold);
            }
        }
    }

    bool PhysicsWorld2D::RefreshManifold(CollisionManifold2D& manifold) const
    {
        if (!manifold.A || !manifold.B)
        {
            return false;
        }

        CollisionManifold2D refreshed;

        if (!GenerateManifold(*manifold.A, *manifold.B, refreshed))
        {
            return false;
        }

        manifold = refreshed;

        return true;
    }

    void PhysicsWorld2D::SolvePositionContacts()
    {
        for (std::size_t iteration = 0; iteration < m_Settings.PositionIterations; ++iteration)
        {
            bool correctedAny = false;

            for (CollisionManifold2D& manifold : m_CurrentContacts)
            {
                if (manifold.IsTrigger)
                {
                    continue;
                }

                if (!RefreshManifold(manifold))
                {
                    continue;
                }

                if (manifold.Penetration <= 0.01f)
                {
                    continue;
                }

                Entity* entityA = manifold.A->GetOwner();

                Entity* entityB = manifold.B->GetOwner();

                if (!entityA || !entityB)
                {
                    continue;
                }

                TransformComponent* transformA = entityA->GetComponent<TransformComponent>();

                TransformComponent* transformB = entityB->GetComponent<TransformComponent>();

                if (!transformA || !transformB)
                {
                    continue;
                }

                Rigidbody2D* bodyA = entityA->GetComponent<Rigidbody2D>();

                Rigidbody2D* bodyB = entityB->GetComponent<Rigidbody2D>();

                const float inverseMassA = GetSolverInverseMass(bodyA);
                const float inverseMassB = GetSolverInverseMass(bodyB);

                if (inverseMassA + inverseMassB <= 0.0f)
                {
                    continue;
                }

                ApplyPositionalCorrection(manifold, bodyA, bodyB, *transformA, *transformB);

                correctedAny = true;
            }

            if (!correctedAny)
            {
                break;
            }
        }
    }

    float PhysicsWorld2D::GetSolverInverseMass(const Rigidbody2D* body) const
    {
        if (!body)
        {
            return 0.0f;
        }

        if (!body->IsDynamic() || body->IsSleeping())
        {
            return 0.0f;
        }

        return body->GetInverseMass();
    }

    float PhysicsWorld2D::GetSolverInverseInertia(const Rigidbody2D* body) const
    {
        if (!body)
        {
            return 0.0f;
        }

        if (!body->IsDynamic() || body->IsSleeping())
        {
            return 0.0f;
        }

        return body->GetInverseInertia();
    }

    float PhysicsWorld2D::Cross2D(const Vector2& a, const Vector2& b) const
    {
        return a.X * b.Y - a.Y * b.X;
    }

    Vector2 PhysicsWorld2D::AngularCrossVector(float angularVelocity, const Vector2& vector) const
    {
        return 
            {
                -angularVelocity * vector.Y,

                angularVelocity * vector.X
            };
    }

    Vector2 PhysicsWorld2D::GetVelocityAtPoint(const Vector2& linearVelocity, float angularVelocity, const Vector2& leverArm) const
    {
        return linearVelocity + AngularCrossVector(angularVelocity, leverArm);
    }

    void PhysicsWorld2D::WakeBodiesFromContacts()
    {
        const float wakeSpeedSquared = m_Settings.CollisionWakeSpeed * m_Settings.CollisionWakeSpeed;

        for (const CollisionManifold2D& manifold : m_CurrentContacts)
        {
            if (manifold.IsTrigger || !manifold.A || !manifold.B)
            {
                continue;
            }

            Entity* entityA = manifold.A->GetOwner();

            Entity* entityB = manifold.B->GetOwner();

            if (!entityA || !entityB)
            {
                continue;
            }

            Rigidbody2D* bodyA = entityA->GetComponent<Rigidbody2D>();

            Rigidbody2D* bodyB = entityB->GetComponent<Rigidbody2D>();

            TransformComponent* transformA = entityA->GetComponent<TransformComponent>();

            TransformComponent* transfromB = entityB->GetComponent<TransformComponent>();

            if (!transformA || !transfromB)
            {
                continue;
            }

            const Vector2 centerA = transformA->GetWorldTransform().Position;

            const Vector2 centerB = transfromB->GetWorldTransform().Position;

            const Vector2 velocityA = bodyA ? bodyA->GetVelocity() : Vector2{0.0f, 0.0f};

            const Vector2 velocityB = bodyB ? bodyB->GetVelocity() : Vector2{0.0f, 0.0f};

            const float angularVelocityA = bodyA ? bodyA->GetAngularVelocity() : 0.0f;

            const float angularVelocityB = bodyB ? bodyB->GetAngularVelocity() : 0.0f;

            bool shouldWake = false;

            if (manifold.ContactCount > 0)
            {
                for (std::size_t i = 0; i < manifold.ContactCount; ++i)
                {
                    const Vector2 point = manifold.ContactPoints[i];

                    const Vector2 rA = point - centerA;

                    const Vector2 rB = point - centerB;

                    const Vector2 contactVelocityA = GetVelocityAtPoint(velocityA, angularVelocityA, rA);

                    const Vector2 contactVelocityB = GetVelocityAtPoint(velocityB, angularVelocityB, rB);

                    const Vector2 relativeVelocity = contactVelocityB - contactVelocityA;

                    if (relativeVelocity.LengthSquared() >= wakeSpeedSquared)
                    {
                        shouldWake = true;

                        break;
                    }
                }
            }
            else 
            {
                const Vector2 relativeVelocity = velocityB - velocityA;

                shouldWake = relativeVelocity.LengthSquared() >= wakeSpeedSquared;
            }

            if (!shouldWake)
            {
                continue;
            }

            if (bodyA && bodyA->IsSleeping())
            {
                bodyA->Wake();
            }

            if (bodyB && bodyB->IsSleeping())
            {
                bodyB->Wake();
            }
        }
    }

    void PhysicsWorld2D::UpdateSleepStates(float deltaTime)
    {
        if (!m_Scene)
        {
            return;
        }

        for (Entity* root : m_Scene->GetRootEntities())
        {
            UpdateEntitySleepRecursive(root, deltaTime);
        }
    }

    void PhysicsWorld2D::UpdateEntitySleepRecursive(Entity* entity, float deltaTime)
    {
        if (!entity)
        {
            return;
        }

        Rigidbody2D* body = entity->GetComponent<Rigidbody2D>();

        if (body && body->CanSleep() && !body->IsSleeping())
        {
            const float LinearThresholdSquared = m_Settings.SleepLinearSpeedThreshold * m_Settings.SleepLinearSpeedThreshold;

            const bool linearMotionIsSmall = body->GetVelocity().LengthSquared() <= LinearThresholdSquared;

            const bool angularMotionIsSmall = std::abs(body->GetAngularVelocity()) <= m_Settings.SleepAngularSpeedThreshold;

            if (linearMotionIsSmall && angularMotionIsSmall)
            {
                body->AddSleepTime(deltaTime);

                if (body->GetSleepTimer() >= m_Settings.TimeToSleep)
                {
                    body->Sleep();
                }
            }
            else {
                body->ResetSleepTimer();
            }
        }

        for (const EntityHandle& childHandle : entity->GetChildren())
        {
            UpdateEntitySleepRecursive(childHandle.Get(), deltaTime);
        }
    }

    void PhysicsWorld2D::SetSpatialCellSize(float cellSize)
    {
        m_Settings.SpatialCellSize = std::max(1.0f, cellSize);
    }

    float PhysicsWorld2D::GetSpatialCellSize() const
    {
        return m_Settings.SpatialCellSize;
    }

    SpatialCell2D PhysicsWorld2D::WorldToCell(const Vector2& worldPosition) const
    {
        return 
        {
            static_cast<std::int32_t>(std::floor(worldPosition.X / m_Settings.SpatialCellSize)),

            static_cast<std::int32_t>(std::floor(worldPosition.Y / m_Settings.SpatialCellSize))
        };
    }

    void PhysicsWorld2D::BuildSpatialGrid()
    {
        m_SpatialGrid.clear();

        m_BroadPhaseProxies.clear();

        for (Collider2D* collider : m_ActiveColliders)
        {
            if (!collider)
            {
                continue;
            }

            BroadPhaseProxy2D proxy = BuildProxy(collider);

            auto [it, inserted] = m_BroadPhaseProxies.emplace(collider, std::move(proxy));

            if (!inserted)
            {
                continue;
            }

            InsertProxyIntoGrid(it->second);
        }
    }

    void PhysicsWorld2D::GenerateCandidatePairs()
    {
        m_CandidatePairs.clear();

        for (auto& entry : m_SpatialGrid)
        {
            SpatialBucket2D& bucket = entry.second;

            for (std::size_t i = 0; i < bucket.size(); ++i)
            {
                for (std::size_t j = i + 1; j < bucket.size(); ++j)
                {
                    Collider2D* a = bucket[i];

                    Collider2D* b = bucket[j];

                    if (!a || !b || a == b)
                    {
                        continue;
                    }

                    m_CandidatePairs.insert(ColliderPair2D::Make(a, b));
                }
            }
        }
    }

    bool PhysicsWorld2D::ShouldBlockOneWayContact(const CollisionManifold2D& manifold) const
    {
        // Returns true when the contact should remain solid.
        //
        // manifold.Normal points from A toward B, i.e. it is the direction in
        // which B is pushed away from A. For a one-way collider we require the
        // other body to be pushed out roughly along that collider's one-way
        // axis (its solid outward normal); otherwise the body is approaching
        // from a pass-through side and the contact is discarded.

        if (!manifold.A || !manifold.B)
        {
            return true;
        }

        if (manifold.A->IsOneWay())
        {
            // Direction B (the other body) is pushed away from A.

            const Vector2 pushDirection = manifold.Normal;

            if (Vector2::Dot(pushDirection, manifold.A->GetOneWayAxis()) < manifold.A->GetOneWayThreshold())
            {
                return false;
            }
        }

        if (manifold.B->IsOneWay())
        {
            // Direction A (the other body) is pushed away from B is the
            // opposite of the manifold normal.

            const Vector2 pushDirection = manifold.Normal * -1.0f;

            if (Vector2::Dot(pushDirection, manifold.B->GetOneWayAxis()) < manifold.B->GetOneWayThreshold())
            {
                return false;
            }
        }

        return true;
    }

    void PhysicsWorld2D::ProcessCandidatePairs()
    {
        for (const ColliderPair2D& pair : m_CandidatePairs)
        {
            Collider2D* a = pair.A;

            Collider2D* b = pair.B;

            // Collision filtering

            if (!ShouldTestPair(a, b))
            {
                continue;
            }

            // AABB broad phase

            if (!BroadPhaseOverlap(*a, *b))
            {
                continue;
            }

            // Shape narrow phase

            CollisionManifold2D manifold;

            ++m_Stats.NarrowPhaseTests;

            if (!GenerateManifold(*a, *b, manifold))
            {
                continue;
            }

            // One-way (pass-through) platform filtering. Only solid contacts
            // can be dropped this way; triggers still fire from every side.

            if (!manifold.IsTrigger &&
                (a->IsOneWay() || b->IsOneWay()) &&
                !ShouldBlockOneWayContact(manifold))
            {
                continue;
            }

            if (manifold.IsTrigger)
            {
                ++m_Stats.TriggerPairs;
            }

            ++m_Stats.GeneratedManifolds;

            // Real contact

            m_CurrentOverlaps.insert(pair);

            m_CurrentContacts.push_back(manifold);

            m_Stats.ContactPoints += manifold.ContactCount;
        }
    }

    std::size_t PhysicsWorld2D::GetSpatialCellCount() const
    {
        return m_SpatialGrid.size();
    }

    std::size_t PhysicsWorld2D::GetCandidatePairCount() const
    {
        return m_CandidatePairs.size();
    }

    const std::vector<Collider2D*>& PhysicsWorld2D::GetActiveColliders() const
    {
        return m_ActiveColliders;
    }

    const std::vector<CollisionManifold2D>& PhysicsWorld2D::GetCurrentContacts() const
    {
        return m_CurrentContacts;
    }

    const std::unordered_map<SpatialCell2D, SpatialBucket2D, SpatialCell2DHash>& PhysicsWorld2D::GetSpatialGrid() const
    {
        return m_SpatialGrid;
    }

    PhysicsDebugStatus2D PhysicsWorld2D::GetDebugStatus() const
    {
        PhysicsDebugStatus2D status;

        status.ActiveColliders = m_ActiveColliders.size();

        status.SpatialCells = m_SpatialGrid.size();

        status.CandidatePairs = m_CandidatePairs.size();

        status.Contacts = m_CurrentContacts.size();

        return status;
    }

    SweptAABBHit2D PhysicsWorld2D::SweptAABB(const Bounds2D& movingStartBounds, const Vector2& relativeMotion, const Bounds2D& targetBounds) const
    {
        SweptAABBHit2D result;

        const Vector2 movingHalf = movingStartBounds.GetSize() * 0.5f;

        const Vector2 movingCenter = movingStartBounds.GetCenter();

        const Vector2 expandedMin = targetBounds.Min - movingHalf;

        const Vector2 expandedMax = targetBounds.Max + movingHalf;

        const float negativeInfinity = -std::numeric_limits<float>::infinity();

        const float positioveInfinity = std::numeric_limits<float>::infinity();

        float xEntry;
        float xExit;

        float yEntry;
        float yExit;

        // X

        if (relativeMotion.X > 0.0f)
        {
            xEntry = (expandedMin.X - movingCenter.X) / relativeMotion.X;

            xExit = (expandedMax.X - movingCenter.X) / relativeMotion.X;
        }
        else if (relativeMotion.X < 0.0f)
        {
            xEntry = (expandedMax.X - movingCenter.X) / relativeMotion.X;

            xExit = (expandedMin.X - movingCenter.X) / relativeMotion.X;
        }
        else
        {
            if (movingCenter .X < expandedMin.X || movingCenter.X > expandedMax.X)
            {
                return result;
            }

            xEntry = negativeInfinity;

            xExit = positioveInfinity;
        }

        // Y

        if (relativeMotion.Y > 0.0f)
        {
            yEntry = (expandedMin.Y - movingCenter.Y) / relativeMotion.Y;

            yExit = (expandedMax.Y - movingCenter.Y) / relativeMotion.Y;
        }
        else if (relativeMotion.Y < 0.0f)
        {
            yEntry = (expandedMax.Y - movingCenter.Y) / relativeMotion.Y;

            yExit = (expandedMin.Y - movingCenter.Y) / relativeMotion.Y;
        }
        else
        {
            if (movingCenter .Y < expandedMin.Y || movingCenter.Y > expandedMax.Y)
            {
                return result;
            }

            yEntry = negativeInfinity;

            yExit = positioveInfinity;
        }

        const float entryTime = std::max(xEntry, yEntry);

        const float exitTime = std::min(xExit, yExit);

        if (entryTime > exitTime || exitTime < 0.0f || entryTime < 0.0f || entryTime > 1.0f)
        {
            return result;
        }

        result.Hit = true;

        result.Time = entryTime;

        if (xEntry > yEntry)
        {
            result.Normal = relativeMotion.X > 0.0f ? Vector2{-1.0f, 0.0f} : Vector2{1.0f, 0.0f};
        }
        else
        {
            result.Normal = relativeMotion.Y > 0.0f ? Vector2{0.0f, -1.0f} : Vector2{0.0f, 1.0f};
        }

        return result;
    }

    Bounds2D PhysicsWorld2D::ReconstructStartBounds(const Collider2D& collider, const Rigidbody2D& body, const TransformComponent& transform) const
    {
        const Bounds2D endBounds = collider.GetWorldBounds();

        const Vector2 currentPosition = transform.GetWorldTransform().Position;

        const Vector2 displacement = currentPosition - body.GetPreviousPosition();

        Bounds2D startBounds = endBounds;

        startBounds.Min -= displacement;

        startBounds.Max -= displacement;

        return startBounds;
    }

    bool PhysicsWorld2D::FindEarliestContinuousHit(Collider2D* movingCollider, const Bounds2D& movingStartBounds, const Vector2& movingMotion, float remainingFraction, SweepHit2D& outHit, Collider2D*& outOtherCollider, Vector2& outOtherMotion)
    {
        outHit = SweepHit2D{};

        outOtherCollider = nullptr;

        outOtherMotion = {0.0f, 0.0f};

        if (!movingCollider)
        {
            return false;
        }

        // -----------------------------------------
        // No movement = nothing to sweep.
        // -----------------------------------------

        if (movingMotion.LengthSquared() <= 0.000001f)
        {
            return false;
        }

        // -----------------------------------------
        // Build AABB covering the entire motion.
        // -----------------------------------------

        const Bounds2D movingSweptBounds = BuildSweptBounds(movingStartBounds, movingMotion);

        // -----------------------------------------
        // Spatial query.
        // -----------------------------------------

        std::vector<Collider2D*> candidates;

        PhysicsQueryFilter2D filter;

        filter.Ignore = movingCollider;

        // We want solid CCD targets here.
        filter.IncludeTriggers = false;

        filter.LayerMask = movingCollider->GetMask();

        QueryBounds(movingSweptBounds, filter, candidates);

        CollectMovingCCDCandidates(movingCollider, movingSweptBounds, candidates);

        // -----------------------------------------
        // Find earliest actual shape hit.
        // -----------------------------------------

        bool foundHit = false;

        float earliestTime = 1.0f;

        Collider2D* earliestCollider = nullptr;

        SweepHit2D earliestHit;

        Vector2 earliestOtherMotion{0.0f, 0.0f};

        for (Collider2D* other : candidates)
        {
            if (!other)
            {
                continue;
            }

            // QueryBounds did some filtering,
            // but ShouldTestPair still performs
            // the complete bilateral collision rules.
            if (!ShouldTestPair(movingCollider, other))
            {
                continue;
            }

            if (movingCollider->IsTrigger() || other->IsTrigger())
            {
                continue;
            }

            const ColliderPair2D pair = ColliderPair2D::Make(movingCollider, other);

            if (m_CCDResolvePairsThisStep.find(pair) != m_CCDResolvePairsThisStep.end())
            {
                continue;
            }

            ++m_Stats.CCDCandidateTests;

            Entity* otherOwner = other->GetOwner();

            if (!otherOwner)
            {
                continue;
            }

            Rigidbody2D* otherBody = otherOwner->GetComponent<Rigidbody2D>();

            TransformComponent* otherTransform = otherOwner->GetComponent<TransformComponent>();

            Vector2 otherFullStepMotion{0.0f, 0.0f};

            if (otherBody && otherTransform)
            {
                otherFullStepMotion = GetBodyStepMotion(otherBody, otherTransform);
            }

            const Vector2 otherMotion = otherFullStepMotion * remainingFraction;

            const Vector2 relativeMotion = movingMotion - otherMotion;

            if (relativeMotion.LengthSquared() <= 0.000001f)
            {
                continue;
            }

            Bounds2D otherStartBounds = other->GetWorldBounds();

            if (otherBody && otherTransform)
            {
                otherStartBounds = ReconstructStartBounds(*other, *otherBody, *otherTransform);
            }

            const Bounds2D otherSweptBouds = BuildSweptBounds(otherStartBounds, otherMotion);

            if (!AABBsOverlap(movingSweptBounds, otherSweptBouds))
            {
                continue;
            }

            ++m_Stats.CCDNarrowPhaseTests;

            const SweepHit2D hit = SweepColliderAgainstMovingCollider(*movingCollider, movingStartBounds, movingMotion, *other, otherStartBounds, otherMotion);

            if (!hit.Hit)
            {
                continue;
            }

            // Prevent near-zero repeated hits.
            if (hit.Time < m_Settings.CCDTimeEpsilon)
            {
                continue;
            }

            ++m_Stats.CCDHits;

            if (hit.Time < earliestTime)
            {
                earliestTime = hit.Time;

                earliestHit = hit;

                earliestCollider = other;

                earliestOtherMotion = otherMotion;

                foundHit = true;
            }
        }

        if (!foundHit)
        {
            return false;
        }

        outHit = earliestHit;

        outOtherCollider = earliestCollider;

        outOtherMotion = earliestOtherMotion;

        return true;
    }

    void PhysicsWorld2D::RunContinuousCollisionPass(float deltaTime)
    {
        if (!m_Scene)
        {
            return;
        }

        const auto processEntity = 
            [&](auto&& self, Entity* entity) -> void
            {
                if (!entity)
                {
                    return;
                }

                Rigidbody2D* body = entity->GetComponent<Rigidbody2D>();

                TransformComponent* transform = entity->GetComponent<TransformComponent>();

                if (body && transform && body->IsDynamic() && !body->IsSleeping() && body->GetCollisionDetectionMode() == CollisionDetectionMode2D::Continuous)
                {
                    ++m_Stats.CCDBodies;

                    Collider2D* movingCollider = nullptr;

                    // Box first.
                    if (auto* box = entity->GetComponent<BoxCollider2D>())
                    {
                        if (box->IsEnabled())
                        {
                            movingCollider = box;
                        }
                    }

                    // Otherwise Circle.
                    if (!movingCollider)
                    {
                        if (auto* circle = entity->GetComponent<CircleCollider2D>())
                        {
                            if (circle->IsEnabled())
                            {
                                movingCollider = circle;
                            }
                        }
                    }

                    if (movingCollider)
                    {
                        ProcessContinuousBody(movingCollider, body, transform, deltaTime);
                    }
                }

                for (const EntityHandle& childHandle : entity->GetChildren())
                {
                    self(self, childHandle.Get());
                }
            };

        for (Entity* root : m_Scene->GetRootEntities())
        {
            processEntity(processEntity, root);
        }
    }

    Bounds2D PhysicsWorld2D::BuildSweptBounds(const Bounds2D& startBounds, const Vector2& motion) const
    {
        Bounds2D endBounds = startBounds;

        endBounds.Min += motion;

        endBounds.Max += motion;

        Bounds2D swept;

        swept.Min = 
        {
            std::min(startBounds.Min.X, endBounds.Min.X), 
            std::min(startBounds.Min.Y, endBounds.Min.Y)
        };

        swept.Max = 
        {
            std::max(startBounds.Max.X, endBounds.Max.X),
            std::max(startBounds.Max.Y, endBounds.Max.Y)
        };

        return swept;
    }

    void PhysicsWorld2D::ProcessContinuousBody(Collider2D* movingCollider, Rigidbody2D* body, TransformComponent* transform, float deltaTime)
    {
        if (!movingCollider || !body || !transform)
        {
            return;
        }

        const Vector2 originalStart = body->GetPreviousPosition();

        const Vector2 integratedEnd = transform->GetWorldTransform().Position;

        Vector2 currentPosition = originalStart;

        Vector2 remainingMotion = integratedEnd - originalStart;

        Vector2 otherMotion{0.0f, 0.0f};

        float remainingFraction = 1.0f;

        // Go back to the actual start.
        transform->SetWorldPosition(currentPosition);

        Bounds2D startBounds = movingCollider->GetWorldBounds();

        for (std::size_t impactIndex = 0; impactIndex < m_Settings.MaxCCDImpacts; ++impactIndex)
        {
            if (remainingMotion.LengthSquared() <= 0.000001f)
            {
                break;
            }

            // Swept riggers will be handled here.
            PublishSweptTriggers(movingCollider, startBounds, remainingMotion);

            SweepHit2D hit;

            Collider2D* other = nullptr;

            if (!FindEarliestContinuousHit(movingCollider, startBounds, remainingMotion, remainingFraction, hit, other, otherMotion))
            {
                currentPosition += remainingMotion;

                transform->SetWorldPosition(currentPosition);

                break;
            }

            const Vector2 impactPoint = currentPosition + remainingMotion * hit.Time;

            PhysicsCCDDebug2D debugRecord;

            debugRecord.MovingCollider = movingCollider;

            debugRecord.TargetCollider = other;

            debugRecord.Start = currentPosition;

            debugRecord.End = currentPosition + remainingMotion;

            debugRecord.ImpactPoint = impactPoint;

            debugRecord.ImpactNormal = hit.Normal;

            debugRecord.TimeOfImpact = hit.Time;

            debugRecord.Hit = hit.Hit;

            m_CCDDebugRecords.push_back(debugRecord);

            // Move to impact.

            currentPosition += remainingMotion * hit.Time;

            currentPosition += hit.Normal * m_Settings.CCDSeparation;

            transform->SetWorldPosition(currentPosition);

            // Target body.

            Entity* otherOwner = other ? other->GetOwner() : nullptr;

            Rigidbody2D* otherBody = otherOwner ? otherOwner->GetComponent<Rigidbody2D>() : nullptr;

            TransformComponent* otherTransform = otherOwner ? otherOwner->GetComponent<TransformComponent>() : nullptr;

            if (otherBody && otherTransform && !otherBody->IsStatic())
            {
                const Vector2 otherStartPosition = otherBody->GetPreviousPosition();

                const Vector2 otherImpactPosition = otherStartPosition + otherMotion * hit.Time;

                otherTransform->SetWorldPosition(otherImpactPosition);
            }

            if (otherBody && otherBody->IsDynamic() && otherBody->IsSleeping())
            {
                otherBody->Wake();
            }

            if (body->IsSleeping())
            {
                body->Wake();
            }

            // Convert CCD result into the generic collision manifold.

            CollisionManifold2D manifold;

            manifold.A = movingCollider;

            manifold.B = other;

            manifold.Normal = hit.Normal * -1.0f;

            manifold.Penetration = 0.0f;

            manifold.IsTrigger = false;

            manifold.ClearContacts();

            if (movingCollider->GetShape() == ColliderShape2D::Circle)
            {
                CircleCollider2D& circle = static_cast<CircleCollider2D&>(*movingCollider);

                manifold.AddContactPoint(circle.GetWorldCenter() - hit.Normal * circle.GetWorldRadius());
            }
            else if (movingCollider->GetShape() == ColliderShape2D::Box)
            {
                const Bounds2D impactBounds = movingCollider->GetWorldBounds();

                const Vector2 impactCenter = impactBounds.GetCenter();

                const Vector2 halfSize = impactBounds.GetSize() * 0.5f;

                Vector2 contactPoint = impactCenter;

                constexpr float epsilon = 0.000001f;

                if (std::abs(hit.Normal.X) > epsilon)
                {
                    contactPoint.X -= hit.Normal.X * halfSize.X;
                }

                if (std::abs(hit.Normal.Y) > epsilon)
                {
                    contactPoint.Y -= hit.Normal.Y * halfSize.Y;
                }

                manifold.AddContactPoint(contactPoint);
            }

            PrepareVelocityContact(manifold);

            // Existing material solver.

            ApplyVelocityResponse(manifold, body, otherBody);

            ++m_Stats.CCDResolvedImpacts;

            m_CCDResolvePairsThisStep.insert(ColliderPair2D::Make(movingCollider, other));

            // Remaining portion of the fixed step.

            remainingFraction *= (1.0f - hit.Time);

            if (remainingFraction <= m_Settings.CCDTimeEpsilon)
            {
                break;
            }

            // New motion uses the solved velocity.

            remainingMotion = body->GetVelocity() * deltaTime * remainingFraction;

            //  Next sweep starts here.
            startBounds = movingCollider->GetWorldBounds();
        }
    }

    void PhysicsWorld2D::PublishSweptTriggers(Collider2D* movingCollider, const Bounds2D& startBounds, const Vector2& motion)
    {
        if (!movingCollider || !m_Scene)
        {
            return;
        }

        // -----------------------------------------
        // Swept region.
        // -----------------------------------------

        const Bounds2D sweptBounds = BuildSweptBounds(startBounds, motion);

        // -----------------------------------------
        // Spatial query.
        // -----------------------------------------

        std::vector<Collider2D*> candidates;

        PhysicsQueryFilter2D filter;

        filter.Ignore = movingCollider;

        // We need triggers here.
        filter.IncludeTriggers = true;

        filter.LayerMask = movingCollider->GetMask();

        QueryBounds(sweptBounds, filter, candidates);

        // -----------------------------------------
        // Test candidates.
        // -----------------------------------------

        for (Collider2D* other : candidates)
        {
            if (!other)
            {
                continue;
            }

            if (!ShouldTestPair(movingCollider, other))
            {
                continue;
            }

            // One of the two must be a Trigger.
            if (!movingCollider->IsTrigger() && !other->IsTrigger())
            {
                continue;
            }

            const SweepHit2D hit = SweepColliderAgainstCollider(*movingCollider, startBounds, motion, *other);

            if (!hit.Hit)
            {
                continue;
            }

            // -------------------------------------
            // Avoid publishing the same Trigger pair
            // several times during one CCD step.
            // -------------------------------------

            const ColliderPair2D pair = ColliderPair2D::Make(movingCollider, other);

            if (!m_SweptTriggerPairsThisStep.insert(pair).second)
            {
                continue;
            }

            Entity* ownerA = movingCollider->GetOwner();

            Entity* ownerB = other->GetOwner();

            if (!ownerA || !ownerB)
            {
                continue;
            }

            // -------------------------------------
            // Publish swept trigger event.
            // -------------------------------------

            SweptTriggerEvent2D event;

            event.A = m_Scene->CreateHandle(ownerA);

            event.B = m_Scene->CreateHandle(ownerB);

            event.ColliderA = movingCollider;

            event.ColliderB = other;

            event.TimeOfImpact = hit.Time;

            ++m_Stats.SweptTriggerHits;

            m_Scene->GetEventBus().Publish<SweptTriggerEvent2D>(event);
        }
    }

    SweepHit2D PhysicsWorld2D::SweepCircleVsCircle(const Vector2& startCenterA, float radiusA, const Vector2& motion, const Vector2& centerB, float radiusB) const
    {
        SweepHit2D result;

        // Relative start position.

        const Vector2 m = startCenterA - centerB;

        const float combinedRadius = radiusA + radiusB;

        // Are the circles already overlapping?
        // Leave existing overlap resolution to the normal discrete solver.

        const float c = Vector2::Dot(m, m) - combinedRadius * combinedRadius;

        if (c <= 0.0f)
        {
            return result;
        }

        // Motion magnitude squared.

        const float a = Vector2::Dot(motion, motion);

        if (a <= 0.000001f)
        {
            return result;
        }

        // Relative motion toward/away from target.

        const float b = Vector2::Dot(m, motion);

        // Circle is moving away from the target.
        if (b >= 0.0f)
        {
            return result;
        }

        // Quadratic discriminant.

        const float discriminant = b * b - a * c;

        if (discriminant < 0.0f)
        {
            return result;
        }

        // Earliest root.

        const float time = (-b - std::sqrt(discriminant)) / a;

        if (time < 0.0f || time > 1.0f)
        {
            return result;
        }

        // Position of moving circle at impact.

        const Vector2 impactCenter = startCenterA + motion * time;

        // Surface normal:
        // target circle -> moving circle.

        Vector2 normal = impactCenter - centerB;

        const float normalLengthSquared = normal.LengthSquared();

        if (normalLengthSquared <= 0.000001f)
        {
            return result;
        }

        normal *= 1.0f / std::sqrt(normalLengthSquared);

        result.Hit = true;

        result.Time = time;

        result.Normal = normal;

        return result;
    }

    SweepHit2D PhysicsWorld2D::SweepCircleVsBox(const Vector2& startCenter, float radius, const Vector2& motion, const Bounds2D& boxBounds) const
    {
        SweepHit2D best;

        if (motion.LengthSquared() <= 0.000001f)
        {
            return best;
        }

        // Initial overlap test.

        const Vector2 closestStart
        {
            std::clamp(startCenter.X, boxBounds.Min.X, boxBounds.Max.X),
            
            std::clamp(startCenter.Y, boxBounds.Min.Y, boxBounds.Max.Y)
        };

        const Vector2 startDelta = startCenter - closestStart;

        if (startDelta.LengthSquared() <= radius * radius)
        {
            return best;
        }

        // =========================================================
        // FACE CANDIDATES
        // =========================================================

        // Left face.
        if (motion.X > 0.0f)
        {
            const float targetX = boxBounds.Min.X - radius;

            const float time = (targetX - startCenter.X) / motion.X;

            if (time >= 0.0f && time <= 1.0f)
            {
                const float y = startCenter.Y + motion.Y * time;

                if (y >= boxBounds.Min.Y && y <= boxBounds.Max.Y)
                {
                    ConsiderSweepCandidate(time, {-1.0f, 0.0f}, best);
                }
            }
        }

        // Right face.
        if (motion.X < 0.0f)
        {
            const float targetX = boxBounds.Max.X + radius;

            const float time = (targetX - startCenter.X) / motion.X;

            if (time >= 0.0f && time <= 1.0f)
            {
                const float y = startCenter.Y + motion.Y * time;

                if (y >= boxBounds.Min.Y && y <= boxBounds.Max.Y)
                {
                    ConsiderSweepCandidate(time, {1.0f, 0.0f}, best);
                }
            }
        }

        // Top face.
        if (motion.Y > 0.0f)
        {
            const float targetY = boxBounds.Min.Y - radius;

            const float time = (targetY - startCenter.Y) / motion.Y;

            if (time >= 0.0f && time <= 1.0f)
            {
                const float x = startCenter.X + motion.X * time;

                if (x >= boxBounds.Min.X && x <= boxBounds.Max.X)
                {
                    ConsiderSweepCandidate(time, {0.0f, -1.0f}, best);
                }
            }
        }

        // Bottom face.
        if (motion.Y < 0.0f)
        {
            const float targetY = boxBounds.Max.Y + radius;

            const float time = (targetY - startCenter.Y) / motion.Y;

            if (time >= 0.0f && time <= 1.0f)
            {
                const float x = startCenter.X + motion.X * time;

                if (x >= boxBounds.Min.X && x <= boxBounds.Max.X)
                {
                    ConsiderSweepCandidate(time, {0.0f, 1.0f}, best);
                }
            }
        }

        // =========================================================
        // CORNER CANDIDATES
        // =========================================================

        const Vector2 topLeft{boxBounds.Min.X, boxBounds.Min.Y};

        const Vector2 topRight{boxBounds.Max.X, boxBounds.Min.Y};

        const Vector2 bottomRight{boxBounds.Max.X, boxBounds.Max.Y};

        const Vector2 bottomLeft{boxBounds.Min.X, boxBounds.Max.Y};

        // Top-left
        {
            const SweepHit2D hit = 
                SweepCircleVsCircle(startCenter, radius, motion, topLeft, 0.0f);

            if (hit.Hit)
            {
                const Vector2 impactCenter = startCenter + motion * hit.Time;

                if (impactCenter.X <= boxBounds.Min.X && impactCenter.Y <= boxBounds.Min.Y)
                {
                    if (!best.Hit || hit.Time < best.Time)
                    {
                        best = hit;
                    }
                }
            }
        }

        // Top-right
        {
            const SweepHit2D hit = 
                SweepCircleVsCircle(startCenter, radius, motion, topRight, 0.0f);

            if (hit.Hit)
            {
                const Vector2 impactCenter = startCenter + motion * hit.Time;

                if (impactCenter.X >= boxBounds.Max.X && impactCenter.Y <= boxBounds.Min.Y)
                {
                    if (!best.Hit || hit.Time < best.Time)
                    {
                        best = hit;
                    }
                }
            }
        }

        // Bottom-right.
        {
            const SweepHit2D hit = 
                SweepCircleVsCircle(startCenter, radius, motion, bottomRight, 0.0f);

            if (hit.Hit)
            {
                const Vector2 impactCenter = startCenter + motion * hit.Time;

                if (impactCenter.X >= boxBounds.Max.X && impactCenter.Y >= boxBounds.Max.Y)
                {
                    if (!best.Hit || hit.Time < best.Time)
                    {
                        best = hit;
                    }
                }
            }
        }

        // Bottom-left.
        {
            const SweepHit2D hit = 
                SweepCircleVsCircle(startCenter, radius, motion, bottomLeft, 0.0f);

            if (hit.Hit)
            {
                const Vector2 impactCenter = startCenter + motion * hit.Time;

                if (impactCenter.X <= boxBounds.Min.X && impactCenter.Y >= boxBounds.Max.Y)
                {
                    if (!best.Hit || hit.Time < best.Time)
                    {
                        best = hit;
                    }
                }
            }
        }

        return best;
    }

    SweepHit2D PhysicsWorld2D::SweepColliderAgainstCollider(Collider2D& moving, const Bounds2D& movingStartBounds, const Vector2& motion, Collider2D& target) const
    {
        const ColliderShape2D movingShape = moving.GetShape();

        const ColliderShape2D targetShape = target.GetShape();

        // =========================================================
        // OBB vs OBB
        // =========================================================

        if (movingShape == ColliderShape2D::Box && targetShape == ColliderShape2D::Box)
        {
            BoxCollider2D& movingBox = static_cast<BoxCollider2D&>(moving);

            BoxCollider2D& targetBox = static_cast<BoxCollider2D&>(target);

            return SweepOBBVsOBB(movingBox.GetWorldOrientedBox(), motion, targetBox.GetWorldOrientedBox());
        }

        // =========================================================
        // Circle vs Circle
        // =========================================================

        if (movingShape == ColliderShape2D::Circle && targetShape == ColliderShape2D::Circle)
        {
            auto& movingCircle = static_cast<CircleCollider2D&>(moving);

            auto& targetCircle = static_cast<CircleCollider2D&>(target);

            return 
                SweepCircleVsCircle(
                    movingCircle.GetWorldCenter(),
                    movingCircle.GetWorldRadius(),
                    motion,
                    targetCircle.GetWorldCenter(),
                    targetCircle.GetWorldRadius()
                );
        }

        // =========================================================
        // Circle vs OBB
        // =========================================================

        if (movingShape == ColliderShape2D::Circle && targetShape == ColliderShape2D::Box)
        {
            CircleCollider2D& movingCircle = static_cast<CircleCollider2D&>(moving);

            BoxCollider2D& targetBox = static_cast<BoxCollider2D&>(target);

            return SweepCircleVsOBB(movingCircle.GetWorldCenter(), movingCircle.GetWorldRadius(), motion, targetBox.GetWorldOrientedBox());
        }

        // =========================================================
        // OBB vs Circle
        // =========================================================

        if (movingShape == ColliderShape2D::Box && targetShape == ColliderShape2D::Circle)
        {
            BoxCollider2D& movingBox = static_cast<BoxCollider2D&>(moving);

            CircleCollider2D& targetCircle = static_cast<CircleCollider2D&>(target);

            SweepHit2D hit = SweepCircleVsOBB(targetCircle.GetWorldCenter(), targetCircle.GetWorldRadius(), motion * -1.0f, movingBox.GetWorldOrientedBox());

            if (hit.Hit)
            {
                hit.Normal *= -1.0f;
            }

            return hit;
        }

        return SweepHit2D{};
    }

    SweepHit2D PhysicsWorld2D::SweepColliderAgainstMovingCollider(Collider2D& moving, const Bounds2D& movingStartBounds, const Vector2& movingMotion, Collider2D& target, const Bounds2D& targetStartBounds, const Vector2& targetMotion) const
    {
        const ColliderShape2D movingShape = moving.GetShape();

        const ColliderShape2D targetShape = target.GetShape();

        const Vector2 relativeMotion = movingMotion - targetMotion;

        // BOX VS BOX

        if (movingShape == ColliderShape2D::Box && targetShape == ColliderShape2D::Box)
        {
            BoxCollider2D& movingBoxCollider = static_cast<BoxCollider2D&>(moving);

            BoxCollider2D& targetBoxCollider = static_cast<BoxCollider2D&>(target);

            // A has already been placed at current CCD interval start.

            const OrientedBox2D movingStartBox = movingBoxCollider.GetWorldOrientedBox();

            // B is reconstructed translationally.

            const OrientedBox2D targetStartBox = TranslateOrientedBox(targetBoxCollider.GetWorldOrientedBox(), targetMotion * -1.0f);

            return SweepOBBVsOBB(movingStartBox, relativeMotion, targetStartBox);
        }

        // CIRCLE VS CIRCLE

        if (movingShape == ColliderShape2D::Circle && targetShape == ColliderShape2D::Circle)
        {
            CircleCollider2D& movingCircle = static_cast<CircleCollider2D&>(moving);

            CircleCollider2D& targetCircle = static_cast<CircleCollider2D&>(target);

            const Vector2 movingStartCenter = movingCircle.GetWorldCenter();

            const Vector2 targetStartCenter = targetCircle.GetWorldCenter() - targetMotion;

            return SweepCircleVsCircle(movingStartCenter, movingCircle.GetWorldRadius(), relativeMotion, targetStartCenter, targetCircle.GetWorldRadius());
        }

        // CIRCLE VS BOX

        if (movingShape == ColliderShape2D::Circle && targetShape == ColliderShape2D::Box)
        {
            CircleCollider2D& movingCircle = static_cast<CircleCollider2D&>(moving);

            BoxCollider2D& targetBoxCollider = static_cast<BoxCollider2D&>(target);

            // A is aleady at its sweep start.
            const Vector2 movingStartCenter = movingCircle.GetWorldCenter();

            // B currently represents its integrated transform.
            // Reconstruct translational start state.

            const OrientedBox2D targetStartBox = TranslateOrientedBox(targetBoxCollider.GetWorldOrientedBox(), targetMotion * -1.0f);

            return SweepCircleVsOBB(movingStartCenter, movingCircle.GetWorldRadius(), relativeMotion, targetStartBox);
        }

        // BOX VS CIRCLE

        if (movingShape == ColliderShape2D::Box && targetShape == ColliderShape2D::Circle)
        {
            BoxCollider2D& movingBox = static_cast<BoxCollider2D&>(moving);

            CircleCollider2D& targetCircle = static_cast<CircleCollider2D&>(target);

            // A has already been rewound by ProcessContinuousBody().
            const OrientedBox2D movingStartBox = movingBox.GetWorldOrientedBox();

            // Reconstruct B's trans;ational start position.

            const Vector2 targetStartCenter = targetCircle.GetWorldCenter() - targetMotion;

            SweepHit2D hit = SweepCircleVsOBB(targetStartCenter, targetCircle.GetWorldRadius(), relativeMotion * -1.0f, movingStartBox);

            if (hit.Hit)
            {
                hit.Normal *= -1.0f;
            }

            return hit;
        }

        return SweepHit2D{};
    }


    SweepHit2D PhysicsWorld2D::SweepCircleVsOBB(const Vector2& startCenter, float radius, const Vector2& motion, const OrientedBox2D& box) const
    {
        // WORLD -> OBB LOCAL

        const Vector2 localStartCenter = WorldPointToOBBLocal(box, startCenter);

        const Vector2 localMotion{Vector2::Dot(motion, box.AxisX), Vector2::Dot(motion, box.AxisY)};

        // OBB BECOMES LOCAL AABB

        Bounds2D localBounds;

        localBounds.Min = {-box.HalfExtents.X, -box.HalfExtents.Y};

        localBounds.Max = {box.HalfExtents.X, box.HalfExtents.Y};

        // EXISTING EXACT CIRCLE/AABB SWEEP

        SweepHit2D hit = SweepCircleVsBox(localStartCenter, radius, localMotion, localBounds);

        if (!hit.Hit)
        {
            return hit;
        }

        // LOCAL NORMAL -> WORLD NORMAL

        hit.Normal = OBBLocalDirectionToWorld(box, hit.Normal);

        return hit;
    }


    SweepHit2D PhysicsWorld2D::SweepOBBVsOBB(const OrientedBox2D& movingStartBox, const Vector2& relativeMotion, const OrientedBox2D& targetStartBox) const
    {
        SweepHit2D result;

        const Vector2 axes[4]
        {
            movingStartBox.AxisX,
            movingStartBox.AxisY,
            targetStartBox.AxisX,
            targetStartBox.AxisY
        };

        float globalEntryTime = -std::numeric_limits<float>::infinity();

        float globalExitTime = std::numeric_limits<float>::infinity();

        Vector2 impactNormal{0.0f, 0.0f};

        for (const Vector2& axis : axes)
        {
            if (axis.LengthSquared() <= 0.000001f)
            {
                continue;
            }

            float minA = 0.0f;
            float maxA = 0.0f;

            float minB = 0.0f;
            float maxB = 0.0f;

            ProjectOrientedBox(movingStartBox, axis, minA, maxA);

            ProjectOrientedBox(targetStartBox, axis, minB, maxB);

            const float relativeSpeed = Vector2::Dot(relativeMotion, axis);

            const SweptAxisResult2D axisResult = SweepIntervalsOnAxis(minA, maxA, minB, maxB, relativeSpeed, axis);

            if (!axisResult.Valid)
            {
                return result;
            }

            if (axisResult.EntryTime > globalEntryTime)
            {
                globalEntryTime = axisResult.EntryTime;

                impactNormal = axisResult.Normal;
            }

            globalExitTime = std::min(globalExitTime, axisResult.ExitTime);

            // The shapes can never overlap simulaneously on all axes.

            if (globalEntryTime > globalExitTime)
            {
                return result;
            }
        }

        // RANGE TEST

        if (globalExitTime < 0.0f)
        {
            return result;
        }

        // Initial overlap belongs to the discrete solver.

        if (globalEntryTime < 0.0f)
        {
            return result;
        }

        if (globalEntryTime > 1.0f)
        {
            return result;
        }

        if (impactNormal.LengthSquared() <= 0.000001f)
        {
            return result;
        }

        result.Hit = true;

        result.Time = globalEntryTime;

        result.Normal = impactNormal;

        return result;
    }


    OrientedBox2D PhysicsWorld2D::TranslateOrientedBox(const OrientedBox2D& box, const Vector2& translation) const
    {
        OrientedBox2D result = box;

        result.Center += translation;

        for (std::size_t i = 0; i < 4; ++i)
        {
            result.Vertices[i] += translation;
        }

        return result;
    }


    void PhysicsWorld2D::ConsiderSweepCandidate(float time, const Vector2& normal, SweepHit2D& bestHit) const
    {
        if (time < 0.0f || time > 1.0f)
        {
            return;
        }

        if (!bestHit.Hit || time < bestHit.Time)
        {
            bestHit.Hit = true;

            bestHit.Time = time;

            bestHit.Normal = normal;
        }
    }

    void PhysicsWorld2D::QueryBounds(const Bounds2D& bounds, const PhysicsQueryFilter2D& filter, std::vector<Collider2D*>& outResults) const
    {
        outResults.clear();

        const SpatialCell2D minCell = WorldToCell(bounds.Min);

        const SpatialCell2D maxCell = WorldToCell(bounds.Max);

        std::unordered_set<Collider2D*> unique;

        for (std::int32_t y = minCell.Y; y <= maxCell.Y; ++y)
        {
            for (std::int32_t x = minCell.X; x <= maxCell.X; ++x)
            {
                const auto it = m_SpatialGrid.find(SpatialCell2D{x, y});

                if (it == m_SpatialGrid.end())
                {
                    continue;
                }

                for (Collider2D* collider : it->second)
                {
                    if (!collider)
                    {
                        continue;
                    }

                    // =============================================
                    // Ignore specific collider.
                    // =============================================

                    if (collider == filter.Ignore)
                    {
                        continue;
                    }

                    // =============================================
                    // Disabled collider.
                    // =============================================

                    if (!collider->IsEnabled())
                    {
                        continue;
                    }

                    // =============================================
                    // Trigger filtering.
                    // =============================================

                    if (!filter.IncludeTriggers && collider->IsTrigger())
                    {
                        continue;
                    }

                    // =============================================
                    // Layer filtering.
                    // =============================================

                    if ((filter.LayerMask & collider->GetLayer()) == 0)
                    {
                        continue;
                    }

                    // =============================================
                    // Deduplicate because one collider may occupy
                    // several cells.
                    // =============================================

                    if (unique.insert(collider).second)
                    {
                        outResults.push_back(collider);
                    }
                }
            }
        }
    }

    bool PhysicsWorld2D::Raycast(const Vector2& origin, const Vector2& direction, float maxDistance, RaycastHit2D& outHit, const PhysicsQueryFilter2D& filter) const
    {
        // ---------------------------------------------------------
        // Always reset output first.
        // ---------------------------------------------------------

        outHit = RaycastHit2D{};

        if (!m_Scene || maxDistance <= 0)
        {
            return false;
        }

        SynchronizeBroadPhaseForQueries();


        // =========================================================
        // NORMALIZE DIRECTION
        // =========================================================

        Vector2 rayDirection = direction;

        const float directionLengthSquared = rayDirection.LengthSquared();

        if (directionLengthSquared <= 0.000001f)
        {
            return false;
        }

        rayDirection *= 1.0f / std::sqrt(directionLengthSquared);


        // =========================================================
        // FINITE RAY END
        // =========================================================

        const Vector2 end = origin + rayDirection * maxDistance;


        // =========================================================
        // BROAD-PHASE BOUNDS
        // =========================================================

        Bounds2D rayBounds;

        rayBounds.Min = {std::min(origin.X, end.X), std::min(origin.Y, end.Y)};

        rayBounds.Max = {std::max(origin.X, end.X), std::max(origin.Y, end.Y)};

        constexpr float rayBoundsPadding = 0.001f;

        rayBounds.Min -= Vector2{rayBoundsPadding, rayBoundsPadding};

        rayBounds.Max += Vector2{rayBoundsPadding, rayBoundsPadding};


        // =========================================================
        // SPATIAL QUERY
        // =========================================================

        std::vector<Collider2D*> candidates;

        QueryBounds(rayBounds, filter, candidates);


        // =========================================================
        // CLOSEST-HIT SEARCH
        // =========================================================

        bool foundHit = false;

        float closestDistance = maxDistance;

        Collider2D* closestCollider = nullptr;

        RayShapeHit2D closestShapeHit;

        for (Collider2D* collider : candidates)
        {
            if (!collider)
            {
                continue;
            }

            const RayShapeHit2D hit = RaycastCollider(origin, rayDirection, maxDistance, *collider);

            if (!hit.Hit)
            {
                continue;
            }

            if (!foundHit || hit.Distance < closestDistance)
            {
                foundHit = true;

                closestDistance = hit.Distance;

                closestCollider = collider;

                closestShapeHit = hit;
            }
        }

        if (!foundHit)
        {
            return false;
        }


        // =========================================================
        // BUILD PUBLIC RESULT
        // =========================================================

        Entity* owner = closestCollider->GetOwner();

        outHit.Hit = true;

        outHit.Collider = closestCollider;

        outHit.Point = origin + rayDirection * closestShapeHit.Distance;

        outHit.Normal = closestShapeHit.Normal;

        outHit.Distance = closestShapeHit.Distance;

        outHit.Fraction = closestShapeHit.Distance / maxDistance;

        if (owner)
        {
            outHit.Entity = m_Scene->CreateHandle(owner);
        }

        return true;
    }

    std::size_t PhysicsWorld2D::RaycastAll(const Vector2& origin, const Vector2& direction, float maxDistance, std::vector<RaycastHit2D>& outHits, const PhysicsQueryFilter2D& filter) const
    {
        outHits.clear();

        if (!m_Scene || maxDistance <= 0.0f)
        {
            return 0;
        }

        SynchronizeBroadPhaseForQueries();

        // =========================================================
        // Normalize direction
        // =========================================================

        Vector2 rayDirection = direction;

        const float directionLengthSquared = rayDirection.LengthSquared();

        if (directionLengthSquared <= 0.000001f)
        {
            return 0;
        }

        rayDirection *= 1.0f / std::sqrt(directionLengthSquared);


        // =========================================================
        // Ray end
        // =========================================================

        const Vector2 end = origin + rayDirection * maxDistance;


        // =========================================================
        // Broad-phase bounds
        // =========================================================

        Bounds2D rayBounds;

        rayBounds.Min = {std::min(origin.X, end.X), std::min(origin.Y, end.Y)};

        rayBounds.Max = {std::max(origin.X, end.X), std::max(origin.Y, end.Y)};

        constexpr float padding = 0.001f;

        rayBounds.Min -= Vector2{padding, padding};

        rayBounds.Max += Vector2{padding, padding};


        // =========================================================
        // Spatial candidates
        // =========================================================

        std::vector<Collider2D*> candidates;

        QueryBounds(rayBounds, filter, candidates);


        // =========================================================
        // Exact ray tests
        // =========================================================

        for (Collider2D* collider : candidates)
        {
            if (!collider)
            {
                continue;
            }

            const RayShapeHit2D shapeHit = RaycastCollider(origin, rayDirection, maxDistance, *collider);

            if (!shapeHit.Hit)
            {
                continue;
            }

            RaycastHit2D hit;

            hit.Hit = true;

            hit.Collider = collider;

            hit.Point = origin + rayDirection * shapeHit.Distance;

            hit.Normal = shapeHit.Normal;

            hit.Distance = shapeHit.Distance;

            hit.Fraction = shapeHit.Distance / maxDistance;

            Entity* owner = collider->GetOwner();

            if (owner && m_Scene)
            {
                hit.Entity = m_Scene->CreateHandle(owner);
            }

            outHits.push_back(hit);
        }


        // =========================================================
        // Sort nearest -> farthest
        // =========================================================

        std::sort(outHits.begin(), outHits.end(),
            [](const RaycastHit2D& a, const RaycastHit2D& b)
            {
                return a.Distance < b.Distance;
            }
        );

        return outHits.size();
    }


    std::size_t PhysicsWorld2D::OverlapPoint(const Vector2& point, std::vector<OverlapHit2D>& outHits, const PhysicsQueryFilter2D& filer) const
    {
        outHits.clear();

        if (!m_Scene)
        {
            return 0;
        }

        SynchronizeBroadPhaseForQueries();

        constexpr float padding = 0.001f;

        Bounds2D queryBounds;

        queryBounds.Min = {point.X - padding, point.Y - padding};

        queryBounds.Max = {point.X + padding, point.Y + padding};

        std::vector<Collider2D*> candidates;

        QueryBounds(queryBounds, filer, candidates);

        for (Collider2D* collider : candidates)
        {
            if (!collider)
            {
                continue;
            }

            if (!PointOverlapsCollider(point, *collider))
            {
                continue;
            }

            OverlapHit2D hit;

            hit.Collider = collider;

            Entity* owner = collider->GetOwner();

            if (owner)
            {
                hit.Entity = m_Scene->CreateHandle(owner);
            }

            outHits.push_back(hit);
        }

        return outHits.size();
    }


    std::size_t PhysicsWorld2D::OverlapCircle(const Vector2& center, float radius, std::vector<OverlapHit2D>& outHits, const PhysicsQueryFilter2D& filter) const
    {
        outHits.clear();

        if (!m_Scene || radius < 0.0f)
        {
            return 0;
        }

        SynchronizeBroadPhaseForQueries();

        const Vector2 extent{radius, radius};

        Bounds2D queryBounds;

        queryBounds.Min = center - extent;

        queryBounds.Max = center + extent;

        std::vector<Collider2D*> candidates;

        QueryBounds(queryBounds, filter, candidates);

        for (Collider2D* collider : candidates)
        {
            if (!collider)
            {
                continue;
            }

            if (!CircleOverlapsCollider(center, radius, *collider))
            {
                continue;
            }

            OverlapHit2D hit;

            hit.Collider = collider;

            Entity* owner = collider->GetOwner();

            if (owner)
            {
                hit.Entity = m_Scene->CreateHandle(owner);
            }

            outHits.push_back(hit);
        }

        return outHits.size();
    }


    std::size_t PhysicsWorld2D::OverlapBox(const Bounds2D& bounds, std::vector<OverlapHit2D>& outHits, const PhysicsQueryFilter2D& filter) const
    {
        outHits.clear();

        if (!m_Scene)
        {
            return 0;
        }

        if (bounds.Max.X < bounds.Min.X || bounds.Max.Y < bounds.Min.Y)
        {
            return 0;
        }

        SynchronizeBroadPhaseForQueries();

        std::vector<Collider2D*> candidates;

        QueryBounds(bounds, filter, candidates);

        for (Collider2D* collider : candidates)
        {
            if (!collider)
            {
                continue;
            }

            if (!BoxOverlapsCollider(bounds, *collider))
            {
                continue;
            }

            OverlapHit2D hit;

            hit.Collider = collider;

            Entity* owner = collider->GetOwner();

            if (owner)
            {
                hit.Entity = m_Scene->CreateHandle(owner);
            }

            outHits.push_back(hit);
        }

        return outHits.size();
    }


    bool PhysicsWorld2D::CircleCast(const Vector2& origin, float radius, const Vector2& direction, float maxDistance, ShapeCastHit2D& outHit, const PhysicsQueryFilter2D& filter) const
    {
        PhysicsQueryContext2D context;

        return CircleCast(origin, radius, direction, maxDistance, outHit, context, filter);
    }

    bool PhysicsWorld2D::CircleCast(const Vector2& origin, float radius, const Vector2& direction, float maxDistance, ShapeCastHit2D& outHit, PhysicsQueryContext2D& context, const PhysicsQueryFilter2D& filter) const
    {
        outHit = ShapeCastHit2D{};

        if (!m_Scene || radius < 0.0f || maxDistance <= 0.0f)
        {
            return false;
        }

        Vector2 castDirection = direction;

        const float lengthSquared = castDirection.LengthSquared();

        if (lengthSquared <= 0.000001f)
        {
            return false;
        }

        SynchronizeBroadPhaseForQueries();

        castDirection *= 1.0f / std::sqrt(lengthSquared);

        const Vector2 motion = castDirection * maxDistance;

        const Vector2 extent{radius, radius};

        Bounds2D startBounds;

        startBounds.Min = origin - extent;

        startBounds.Max = origin + extent;

        const Bounds2D sweptBounds = BuildSweptBounds(startBounds, motion);

        const auto& candidates = QueryBoundsToContext(sweptBounds, filter, context);

        bool foundHit = false;

        float earliestTime = 1.0f;

        Collider2D* closestCollider = nullptr;

        SweepHit2D closestSweep;

        for (Collider2D* collider : candidates)
        {
            if (!collider)
            {
                continue;
            }

            const SweepHit2D hit = SweepCircleQueryAgainstCollider(origin, radius, motion, *collider);

            if (!hit.Hit)
            {
                continue;
            }

            if (!foundHit || hit.Time < earliestTime)
            {
                foundHit = true;

                earliestTime = hit.Time;

                closestCollider = collider;

                closestSweep = hit;
            }
        }

        if (!foundHit)
        {
            return false;
        }

        const Vector2 impactCenter = origin + motion * closestSweep.Time;

        outHit.Hit = true;

        outHit.Collider = closestCollider;

        outHit.Normal = closestSweep.Normal;

        outHit.Fraction = closestSweep.Time;

        outHit.Distance = closestSweep.Time * maxDistance;

        const Vector2 contactPoint = impactCenter - closestSweep.Normal * radius;

        outHit.Point = contactPoint;

        Entity* owner = closestCollider->GetOwner();

        if (owner)
        {
            outHit.Entity = m_Scene->CreateHandle(owner);
        }

        return true;
    }


    bool PhysicsWorld2D::BoxCast(const Bounds2D& startBounds, const Vector2& direction, float maxDistance, ShapeCastHit2D& outHit, const PhysicsQueryFilter2D& filter) const
    {
        PhysicsQueryContext2D context;

        return BoxCast(startBounds, direction, maxDistance, outHit, context, filter);
    }


    bool PhysicsWorld2D::BoxCast(const Bounds2D& startBounds, const Vector2& direction, float maxDistance, ShapeCastHit2D& outHit, PhysicsQueryContext2D& context, const PhysicsQueryFilter2D& filter) const
    {
        outHit = ShapeCastHit2D{};

        if (!m_Scene || maxDistance <= 0.0f)
        {
            return false;
        }

        if (startBounds.Max.X < startBounds.Min.X || startBounds.Max.Y < startBounds.Min.Y)
        {
            return false;
        }

        Vector2 castDirection = direction;

        const float lengthSquared = castDirection.LengthSquared();

        if (lengthSquared <= 0.000001f)
        {
            return false;
        }

        SynchronizeBroadPhaseForQueries();

        castDirection *= 1.0f / std::sqrt(lengthSquared);

        const Vector2 motion = castDirection * maxDistance;

        const Bounds2D sweptBounds = BuildSweptBounds(startBounds, motion);

        const auto& candidates = QueryBoundsToContext(sweptBounds, filter, context);

        bool foundHit = false;

        float earliestTime = 1.0f;

        Collider2D* closestCollider = nullptr;

        SweepHit2D closestSweep;

        for (Collider2D* collider : candidates)
        {
            if (!collider)
            {
                continue;
            }

            const SweepHit2D hit = SweepBoxQueryAgainstCollider(startBounds, motion, *collider);

            if (!hit.Hit)
            {
                continue;
            }

            if (!foundHit || hit.Time < earliestTime)
            {
                foundHit = true;

                earliestTime = hit.Time;

                closestCollider = collider;

                closestSweep = hit;
            }
        }

        if (!foundHit)
        {
            return false;
        }

        const Vector2 startCenter = startBounds.GetCenter();

        const Vector2 impactCenter = startCenter + motion * closestSweep.Time;

        const Vector2 halfSize = startBounds.GetSize() * 0.5f;

        Vector2 contactPoint = impactCenter;

        constexpr float epsilon = 0.000001f;

        if (std::abs(closestSweep.Normal.X) > epsilon)
        {
            contactPoint.X -= closestSweep.Normal.X * halfSize.X;
        }

        if (std::abs(closestSweep.Normal.Y) > epsilon)
        {
            contactPoint.Y -= closestSweep.Normal.Y * halfSize.Y;
        }

        outHit.Hit = true;

        outHit.Collider = closestCollider;

        outHit.Normal = closestSweep.Normal;

        outHit.Fraction = closestSweep.Time;

        outHit.Distance = closestSweep.Time * maxDistance;

        outHit.Point = contactPoint;

        Entity* owner = closestCollider->GetOwner();

        if (owner)
        {
            outHit.Entity = m_Scene->CreateHandle(owner);
        }

        return true;
    }


    PhysicsWorld2D::RayShapeHit2D PhysicsWorld2D::RaycastAABB(const Vector2& origin, const Vector2& direction, float maxDistance, const Bounds2D& bounds) const
    {
        RayShapeHit2D result;

        constexpr float epsilon = 0.000001f;

        float tMin = 0.0f;

        float tMax = maxDistance;

        Vector2 hitNormal{0.0f, 0.0f};

        // =========================================================
        // X SLAB
        // =========================================================

        if (std::abs(direction.X) < epsilon)
        {
            // Ray is parallel to the X slab.

            if (origin.X < bounds.Min.X || origin.X > bounds.Max.X)
            {
                return result;
            }
        }
        else
        {
            const float inverseDirection = 1.0f / direction.X;

            float t1 = (bounds.Min.X - origin.X) * inverseDirection;

            float t2 = (bounds.Max.X - origin.X) * inverseDirection;

            // Normal of the first X surface approached by the ray.
            Vector2 nearNormal = direction.X > 0.0f ? Vector2{-1.0f, 0.0f} : Vector2{1.0f, 0.0f};

            if (t1 > t2)
            {
                std::swap(t1, t2);
            }

            if (t1 > tMin)
            {
                tMin = t1;

                hitNormal = nearNormal;
            }

            tMax = std::min(tMax, t2);

            if (tMin > tMax)
            {
                return result;
            }
        }

        // =========================================================
        // Y SLAB
        // =========================================================

        if (std::abs(direction.Y) < epsilon)
        {
            if (origin.Y < bounds.Min.Y || origin.Y > bounds.Max.Y)
            {
                return result;
            }
        }
        else
        {
            const float inverseDirection = 1.0f / direction.Y;

            float t1 = (bounds.Min.Y - origin.Y) * inverseDirection;

            float t2 = (bounds.Max.Y - origin.Y) * inverseDirection;

            Vector2 nearNormal = direction.Y > 0.0f ? Vector2{0.0f, -1.0f} : Vector2{0.0f, 1.0f};

            if (t1 > t2)
            {
                std::swap(t1, t2);
            }

            if (t1 > tMin)
            {
                tMin = t1;

                hitNormal = nearNormal;
            }

            tMax = std::min(tMax, t2);

            if (tMin > tMax)
            {
                return result;
            }
        }

        // =========================================================
        // FINAL RANGE TEST
        // =========================================================

        if (tMin < 0.0f || tMin > maxDistance)
        {
            return result;
        }

        result.Hit = true;

        result.Distance = tMin;

        result.Normal = hitNormal;

        return  result;
    }

    PhysicsWorld2D::RayShapeHit2D PhysicsWorld2D::RaycastCircle(const Vector2& origin, const Vector2& direction, float maxDistance, const Vector2& center, float radius) const
    {
        RayShapeHit2D result;

        const Vector2 m = origin - center;

        const float c = Vector2::Dot(m, m) - radius * radius;

        // =========================================================
        // Ray begins inside Circle.
        // =========================================================

        if (c <= 0.0f)
        {
            result.Hit = true;

            result.Distance = 0.0f;

            result.Normal = {0.0f, 0.0f};

            return result;
        }

        const float b = Vector2::Dot(m, direction);

        // Outside the circle and moving away.
        if (b > 0.0f)
        {
            return result;
        }

        const float discriminant = b * b - c;

        if (discriminant < 0.0f)
        {
            return result;
        }

        const float distance = -b - std::sqrt(discriminant);

        if (distance < 0.0f || distance > maxDistance)
        {
            return result;
        }

        // =========================================================
        // Hit point.
        // =========================================================

        const Vector2 point = origin + direction * distance;

        // =========================================================
        // Circle outward normal.
        // =========================================================

        Vector2 normal = point - center;

        const float lengthSquared = normal.LengthSquared();

        if (lengthSquared > 0.000001f)
        {
            normal *= 1.0f / std::sqrt(lengthSquared);
        }

        result.Hit = true;

        result.Distance = distance;

        result.Normal = normal;

        return result;
    }

    PhysicsWorld2D::RayShapeHit2D PhysicsWorld2D::RaycastCollider(const Vector2& origin, const Vector2& direction, float maxDistance, const Collider2D& collider) const
    {
        switch (collider.GetShape())
        {
            // BOX

            case ColliderShape2D::Box:
            {
                return RaycastAABB(origin, direction, maxDistance, collider.GetWorldBounds());
            }

            // CIRCLE

            case ColliderShape2D::Circle:
            {
                const CircleCollider2D& circle = static_cast<const CircleCollider2D&>(collider);

                return RaycastCircle(origin, direction, maxDistance, circle.GetWorldCenter(), circle.GetWorldRadius());
            }

            // CAPSULE

            case ColliderShape2D::Capsule:
            {
                return {};
            }
        }

        return RayShapeHit2D{};
    }

    bool PhysicsWorld2D::PointInsideAABB(const Vector2& point, const Bounds2D& bounds) const
    {
        return point.X >= bounds.Min.X &&
               point.X <= bounds.Max.X &&
               point.Y >= bounds.Min.Y &&
               point.Y <= bounds.Max.Y;
    }

    bool PhysicsWorld2D::PointInsideCircle(const Vector2& point, const Vector2& center, float radius) const
    {
        const Vector2 delta = point - center;

        return delta.LengthSquared() <= radius * radius;
    }

    bool PhysicsWorld2D::PointOverlapsCollider(const Vector2& point, const Collider2D& collider) const
    {
        switch (collider.GetShape())
        {
            //BOX

            case ColliderShape2D::Box:
            {
                return PointInsideAABB(point, collider.GetWorldBounds());
            }

            // CIRCLE

            case ColliderShape2D::Circle:
            {
                const CircleCollider2D& circle = static_cast<const CircleCollider2D&>(collider);

                return PointInsideCircle(point, circle.GetWorldCenter(), circle.GetWorldRadius());
            }

            // CAPSULE

            case ColliderShape2D::Capsule:
            {
                return false;
            }
        }

        return false;
    }

    bool PhysicsWorld2D::CirclesOverlap(const Vector2& centerA, float radiusA, const Vector2& centerB, float radiusB) const
    {
        const Vector2 delta = centerB - centerA;

        const float combinedRadius = radiusA + radiusB;

        return delta.LengthSquared() <= combinedRadius * combinedRadius;
    }

    bool PhysicsWorld2D::CircleOverlapsAABB(const Vector2& center, float radius, const Bounds2D& bounds) const
    {
        const Vector2 closest{std::clamp(center.X, bounds.Min.X, bounds.Max.X), std::clamp(center.Y, bounds.Min.Y, bounds.Max.Y)};

        const Vector2 delta = center - closest;

        return delta.LengthSquared() <= radius * radius;
    }

    bool PhysicsWorld2D::CircleOverlapsCollider(const Vector2& center, float radius, const Collider2D& collider) const
    {
        switch (collider.GetShape())
        {
            //BOX

            case ColliderShape2D::Box:
            {
                return CircleOverlapsAABB(center, radius, collider.GetWorldBounds());
            }

            // CIRCLE

            case ColliderShape2D::Circle:
            {
                const CircleCollider2D& circle = static_cast<const CircleCollider2D&>(collider);

                return CirclesOverlap(center, radius, circle.GetWorldCenter(), circle.GetWorldRadius());
            }

            // CAPSULE

            case ColliderShape2D::Capsule:
            {
                return false;
            }
        }

        return false;
    }

    bool PhysicsWorld2D::AABBsOverlap(const Bounds2D& a, const Bounds2D& b) const
    {
        return a.Min.X <= b.Max.X &&
               a.Max.X >= b.Min.X &&
               a.Min.Y <= b.Max.Y &&
               a.Max.Y >= b.Min.Y;
    }

    bool PhysicsWorld2D::BoxOverlapsCollider(const Bounds2D& queryBox, const Collider2D& collider) const
    {
        switch (collider.GetShape())
        {
            //BOX

            case ColliderShape2D::Box:
            {
                return AABBsOverlap(queryBox, collider.GetWorldBounds());
            }

            // CIRCLE

            case ColliderShape2D::Circle:
            {
                const CircleCollider2D& circle = static_cast<const CircleCollider2D&>(collider);

                return CircleOverlapsAABB(circle.GetWorldCenter(), circle.GetWorldRadius(), queryBox);
            }

            // CAPSULE

            case ColliderShape2D::Capsule:
            {
                return false;
            }
        }

        return false;
    }

    const std::vector<Collider2D*>& PhysicsWorld2D::QueryBoundsToContext(const Bounds2D& bounds, const PhysicsQueryFilter2D& filter, PhysicsQueryContext2D& context) const
    {
        context.Reset();

        const SpatialCell2D minCell = WorldToCell(bounds.Min);

        const SpatialCell2D maxCell = WorldToCell(bounds.Max);

        for (std::int32_t y = minCell.Y; y <= maxCell.Y; ++y)
        {
            for (std::int32_t x = minCell.X; x <= maxCell.X; ++x)
            {
                const auto it = m_SpatialGrid.find(SpatialCell2D{x, y});

                if (it == m_SpatialGrid.end())
                {
                    continue;
                }

                for (Collider2D* collider : it->second)
                {
                    if (!collider)
                    {
                        continue;
                    }

                    if (collider == filter.Ignore)
                    {
                        continue;
                    }

                    if (!collider->IsEnabled())
                    {
                        continue;
                    }

                    if (!filter.IncludeTriggers && collider->IsTrigger())
                    {
                        continue;
                    }

                    if ((filter.LayerMask & collider->GetLayer()) == 0)
                    {
                        continue;
                    }

                    if (context.Unique.insert(collider).second)
                    {
                        context.Candidates.push_back(collider);
                    }
                }
            }
        }

        return context.Candidates;
    }

    SweepHit2D PhysicsWorld2D::SweepCircleQueryAgainstCollider(const Vector2& startCenter, float radius, const Vector2& motion, const Collider2D& target) const
    {
        switch (target.GetShape())
        {
            // Circle query vs Box

            case ColliderShape2D::Box:
            {
                return SweepCircleVsBox(startCenter, radius, motion, target.GetWorldBounds());
            }

            // Circle query vs Circle

            case ColliderShape2D::Circle:
            {
                const CircleCollider2D& circle = static_cast<const CircleCollider2D&>(target);

                return SweepCircleVsCircle(startCenter, radius, motion, circle.GetWorldCenter(), circle.GetWorldRadius());
            }
        }

        return SweepHit2D{};
    }

    SweepHit2D PhysicsWorld2D::SweepBoxQueryAgainstCollider(const Bounds2D& startBounds, const Vector2& motion, const Collider2D& target) const
    {
        switch (target.GetShape())
        {
            // BOX vs BOX

            case ColliderShape2D::Box:
            {
                const SweptAABBHit2D boxHit = SweptAABB(startBounds, motion, target.GetWorldBounds());

                SweepHit2D result;

                result.Hit = boxHit.Hit;

                result.Time = boxHit.Time;

                result.Normal = boxHit.Normal;

                return result;
            }

            // BOX vs CIRCLE

            case ColliderShape2D::Circle:
            {
                const CircleCollider2D& circle = static_cast<const CircleCollider2D&>(target);

                SweepHit2D result = SweepCircleVsBox(circle.GetWorldCenter(), circle.GetWorldRadius(), motion * -1.0f, startBounds);

                if (result.Hit)
                {
                    result.Normal *= -1.0f;
                }

                return result;
            }
        }
        
        return SweepHit2D{};
    }

    
    void PhysicsWorld2D::GetCellsForBounds(const Bounds2D& bounds, std::vector<SpatialCell2D>& outCells) const
    {
        outCells.clear();

        const SpatialCell2D minCell = WorldToCell(bounds.Min);

        const SpatialCell2D maxCell = WorldToCell(bounds.Max);

        const std::size_t width = static_cast<std::size_t>(maxCell.X - minCell.X + 1);

        const std::size_t height = static_cast<std::size_t>(maxCell.Y - minCell.Y + 1);

        outCells.reserve(width * height);

        for (std::int32_t y = minCell.Y; y <= maxCell.Y; ++y)
        {
            for (std::int32_t x = minCell.X; x <= maxCell.X; ++x)
            {
                outCells.push_back({x, y});
            }
        }
    }

    void PhysicsWorld2D::InsertProxyIntoGrid(BroadPhaseProxy2D& proxy) const
    {
        for (const SpatialCell2D& cell : proxy.Cells)
        {
            m_SpatialGrid[cell].push_back(proxy.Collider);
        }
    }

    void PhysicsWorld2D::RemoveProxyFromGrid(const BroadPhaseProxy2D& proxy) const
    {
        for (const SpatialCell2D& cell : proxy.Cells)
        {
            const auto gridIt = m_SpatialGrid.find(cell);

            if (gridIt == m_SpatialGrid.end())
            {
                continue;
            }

            auto& bucket = gridIt->second;

            bucket.erase(std::remove(bucket.begin(), bucket.end(), proxy.Collider), bucket.end());

            // Do not keep useless empty cells.
            if (bucket.empty())
            {
                m_SpatialGrid.erase(gridIt);
            }
        }
    }

    BroadPhaseProxy2D PhysicsWorld2D::BuildProxy(Collider2D* collider) const
    {
        BroadPhaseProxy2D proxy;

        if (!collider)
        {
            return proxy;
        }

        proxy.Collider = collider;

        proxy.Bounds = collider->GetWorldBounds();

        proxy.ColliderBoundsRevision = collider->GetBoundsRevision();

        Entity* owner = collider->GetOwner();

        if (owner)
        {
            TransformComponent* transform = owner->GetComponent<TransformComponent>();

            if (transform)
            {
                proxy.TransformWorldVersion = transform->GetWorldVersion();
            }
        }

        if (collider->IsEnabled())
        {
            GetCellsForBounds(proxy.Bounds, proxy.Cells);
        }

        return proxy;
    }

    bool PhysicsWorld2D::IsProxyDirty(const BroadPhaseProxy2D& proxy) const
    {
        Collider2D* collider = proxy.Collider;

        if (!collider)
        {
            return false;
        }

        if (proxy.ColliderBoundsRevision != collider->GetBoundsRevision())
        {
            return true;
        }

        Entity* owner = collider->GetOwner();

        if (!owner)
        {
            return true;
        }

        TransformComponent* transform = owner->GetComponent<TransformComponent>();

        if (!transform)
        {
            return false;
        }

        return proxy.TransformWorldVersion != transform->GetWorldVersion();
    }

    void PhysicsWorld2D::UpdateBroadPhaseProxy(BroadPhaseProxy2D& proxy) const
    {
        Collider2D* collider = proxy.Collider;

        if (!collider)
        {
            return;
        }

        RemoveProxyFromGrid(proxy);

        proxy.Cells.clear();

        proxy.ColliderBoundsRevision = collider->GetBoundsRevision();

        Entity* owner = collider->GetOwner();

        if (owner)
        {
            TransformComponent* transform = owner->GetComponent<TransformComponent>();

            if (transform)
            {
                proxy.TransformWorldVersion = transform->GetWorldVersion();
            }
        }

        if (!collider->IsEnabled())
        {
            return;
        }

        proxy.Bounds = collider->GetWorldBounds();

        GetCellsForBounds(proxy.Bounds, proxy.Cells);

        InsertProxyIntoGrid(proxy);
    }

    void PhysicsWorld2D::SynchronizeBroadPhaseForQueries() const
    {
        for (auto& entry : m_BroadPhaseProxies)
        {
            BroadPhaseProxy2D& proxy = entry.second;

            if (!proxy.Collider)
            {
                continue;
            }

            if (!IsProxyDirty(proxy))
            {
                continue;
            }

            UpdateBroadPhaseProxy(proxy);
        }
    }

    void PhysicsWorld2D::PrepareVelocityContacts()
    {
        for (CollisionManifold2D& manifold : m_CurrentContacts)
        {
            PrepareVelocityContact(manifold);
        }
    }

    void PhysicsWorld2D::PrepareVelocityContact(CollisionManifold2D& manifold)
    {
        if (!manifold.IsValid() || manifold.IsTrigger)
        {
            return;
        }

        Entity* entityA = manifold.A->GetOwner();

        Entity* entityB = manifold.B->GetOwner();

        if (!entityA || !entityB)
        {
            return;
        }

        TransformComponent* transformA = entityA->GetComponent<TransformComponent>();

        TransformComponent* transformB = entityB->GetComponent<TransformComponent>();

        if (!transformA || !transformB)
        {
            return;
        }

        Rigidbody2D* bodyA = entityA->GetComponent<Rigidbody2D>();

        Rigidbody2D* bodyB = entityB->GetComponent<Rigidbody2D>();

        const Vector2 centerA = transformA->GetWorldTransform().Position;

        const Vector2 centerB = transformB->GetWorldTransform().Position;

        const Vector2 velocityA = bodyA ? bodyA->GetVelocity() : Vector2{0.0f, 0.0f};

        const Vector2 velocityB = bodyB ? bodyB->GetVelocity() : Vector2{0.0f, 0.0f};

        const float angularVelocityA = bodyA ? bodyA->GetAngularVelocity() : 0.0f;

        const float angularVelocityB = bodyB ? bodyB->GetAngularVelocity() : 0.0f;

        const float restitution = CombineRestitution(*manifold.A, *manifold.B);

        const float restitutionVelocityThreshold = m_Settings.RestitutionVelocityThreshold;

        for (std::size_t i = 0; i <manifold.ContactCount; ++i)
        {
            const Vector2 contactPoint = manifold.ContactPoints[i];

            const Vector2 rA = contactPoint - centerA;

            const Vector2 rB = contactPoint - centerB;

            const Vector2 contactVelocityA = GetVelocityAtPoint(velocityA, angularVelocityA, rA);

            const Vector2 contactVelocityB = GetVelocityAtPoint(velocityB, angularVelocityB, rB);

            const Vector2 relativeVelocity = contactVelocityB - contactVelocityA;

            const float velocityAlongNormal = Vector2::Dot(relativeVelocity, manifold.Normal);

            manifold.RestitutionBiases[i] = 0.0f;

            if (velocityAlongNormal < -restitutionVelocityThreshold)
            {
                manifold.RestitutionBiases[i] = -restitution * velocityAlongNormal;
            }
        }
    }

    void PhysicsWorld2D::WarmStartVelocityContacts()
    {
        for (CollisionManifold2D& manifold : m_CurrentContacts)
        {
            WarmStartVelocityContact(manifold);
        }
    }

    void PhysicsWorld2D::WarmStartVelocityContact(CollisionManifold2D& manifold)
    {
        if (!manifold.IsValid() || manifold.IsTrigger)
        {
            return;
        }

        Entity* entityA = manifold.A->GetOwner();

        Entity* entityB = manifold.B->GetOwner();

        if (!entityA || !entityB)
        {
            return;
        }

        TransformComponent* transformA = entityA->GetComponent<TransformComponent>();

        TransformComponent* transformB = entityB->GetComponent<TransformComponent>();

        if (!transformA || !transformB)
        {
            return;
        }

        Rigidbody2D* bodyA = entityA->GetComponent<Rigidbody2D>();

        Rigidbody2D* bodyB = entityB->GetComponent<Rigidbody2D>();

        const float inverseMassA = GetSolverInverseMass(bodyA);

        const float inverseMassB = GetSolverInverseMass(bodyB);

        const float inverseInertiaA = GetSolverInverseInertia(bodyA);

        const float inverseInertiaB = GetSolverInverseInertia(bodyB);

        const Vector2 centerA = transformA->GetWorldTransform().Position;

        const Vector2 centerB = transformB->GetWorldTransform().Position;

        Vector2 velocityA = bodyA ? bodyA->GetVelocity() : Vector2{0.0f, 0.0f};

        Vector2 velocityB = bodyB ? bodyB->GetVelocity() : Vector2{0.0f, 0.0f};

        float angularVelocityA = bodyA ? bodyA->GetAngularVelocity() : 0.0f;
        
        float angularVelocityB = bodyB ? bodyB->GetAngularVelocity() : 0.0f;

        for (std::size_t i = 0; i < manifold.ContactCount; ++i)
        {
            const Vector2 Point = manifold.ContactPoints[i];

            const Vector2 rA = Point - centerA;

            const Vector2 rB = Point - centerB;

            // Tangent perpendicular to the normal.
            const Vector2 tangent
            {
                -manifold.Normal.Y,
                manifold.Normal.X
            };

            const Vector2 impulse = manifold.Normal * manifold.AccumulatedNormalImpulses[i] + tangent * manifold.AccumulatedTangentImpulses[i];

            if (bodyA && inverseMassA > 0.0f)
            {
                velocityA -= impulse * inverseMassA;
            }

            if (bodyA && inverseInertiaA > 0.0f)
            {
                angularVelocityA -= Cross2D(rA, impulse) * inverseInertiaA;
            }

            if (bodyB && inverseMassB > 0.0f)
            {
                velocityB += impulse * inverseMassB;
            }

            if (bodyB && inverseInertiaB > 0.0f)
            {
                angularVelocityB += Cross2D(rB, impulse) * inverseInertiaB;
            }
        }

        if (bodyA)
        {
            bodyA->SetVelocityFromPhysics(velocityA);

            bodyA->SetAngularVelocityFromPhysics(angularVelocityA);
        }

        if (bodyB)
        {
            bodyB->SetVelocityFromPhysics(velocityB);

            bodyB->SetAngularVelocityFromPhysics(angularVelocityB);
        }
    }

    void PhysicsWorld2D::RestoreCachedContactImpulses()
    {
        for (CollisionManifold2D& manifold : m_CurrentContacts)
        {
            RestoreCachedContactImpulses(manifold);
        }
    }

    void PhysicsWorld2D::RestoreCachedContactImpulses(CollisionManifold2D& manifold)
    {
        if (!manifold.IsValid() || manifold.IsTrigger)
        {
            return;
        }
        
        const ColliderPair2D pair = ColliderPair2D::Make(manifold.A, manifold.B);

        const auto cachedIt = m_ContactCache.find(pair);

        if (cachedIt == m_ContactCache.end())
        {
            return;
        }

        const CachedContactPair2D& cachedPair = cachedIt->second;

        bool used[2]{false, false};

        for (std::size_t contactIndex = 0; contactIndex < manifold.ContactCount; ++contactIndex)
        {
            const int cachedIndex = FindClosestCachedContact(cachedPair, manifold.ContactPoints[contactIndex], used);

            if (cachedIndex < 0)
            {
                continue;
            }

            ++m_Stats.RestoredCachedContacts;

            const std::size_t index = static_cast<std::size_t>(cachedIndex);

            used[index] = true;

            manifold.AccumulatedNormalImpulses[contactIndex] = cachedPair.Contacts[index].NormalImpulse;

            manifold.AccumulatedTangentImpulses[contactIndex] = cachedPair.Contacts[index].TangentImpulse;
        }
    }

    int PhysicsWorld2D::FindClosestCachedContact(const CachedContactPair2D& cachedPair, const Vector2& point, bool used[2]) const
    {
        constexpr float matchDistance = 5.0f;

        const float matchDistanceSquared = matchDistance * matchDistance;

        int bestIndex = -1;

        float bestDistanceSquared = matchDistanceSquared;

        for (std::size_t i = 0; i < cachedPair.ContactCount; ++i)
        {
            if (used[i])
            {
                continue;
            }

            const Vector2 delta = cachedPair.Contacts[i].Point - point;

            const float distanceSquared = delta.LengthSquared();

            if (distanceSquared < bestDistanceSquared)
            {
                bestDistanceSquared = distanceSquared;

                bestIndex = static_cast<int>(i);
            }
        }

        return bestIndex;
    }

    void PhysicsWorld2D::StoreContactCache()
    {
        for (const CollisionManifold2D& manifold : m_CurrentContacts)
        {
            StoreContactCache(manifold);
        }
    }

    void PhysicsWorld2D::StoreContactCache(const CollisionManifold2D& manifold)
    {
        if (!manifold.IsValid() || manifold.IsTrigger)
        {
            return;
        }

        const ColliderPair2D pair = ColliderPair2D::Make(manifold.A, manifold.B);

        CachedContactPair2D& cachedPair = m_ContactCache[pair];

        cachedPair.ContactCount = std::min(manifold.ContactCount, CachedContactPair2D::MaxContacts);

        for (std::size_t i = 0; i < cachedPair.ContactCount; ++i)
        {
            cachedPair.Contacts[i].Point = manifold.ContactPoints[i];

            cachedPair.Contacts[i].NormalImpulse = manifold.AccumulatedNormalImpulses[i];

            cachedPair.Contacts[i].TangentImpulse = manifold.AccumulatedTangentImpulses[i];
        }
    }

    void PhysicsWorld2D::RemoveStaleCachedContacts()
    {
        for (auto it = m_ContactCache.begin(); it != m_ContactCache.end();)
        {
            if (m_CurrentOverlaps.find(it->first) == m_CurrentOverlaps.end())
            {
                it = m_ContactCache.erase(it);
            }
            else 
            {
                ++it;
            }
        }
    }

    void PhysicsWorld2D::BuildIslands()
    {
        m_Islands.clear();

        std::unordered_map<Rigidbody2D*, std::vector<std::size_t>> bodyContacts;

        std::unordered_map<Rigidbody2D*, std::vector<Joint2D*>> bodyJoints;

        // Keep first-seen order rather than iteration the unordered_map directly.
        std::vector<Rigidbody2D*> bodyOrder;

        std::unordered_set<Rigidbody2D*> discoveredBodies;

        for (Joint2D* joint : m_Joints)
        {
            if (!joint || !joint->IsEnabled())
            {
                continue;
            }

            Rigidbody2D* bodyA = joint->GetBodyA();

            Rigidbody2D* bodyB = joint->GetBodyB();

            const bool dynamicA = IsIslandDynamicBody(bodyA);

            const bool dynamicB = IsIslandDynamicBody(bodyB);

            if (!dynamicA && !dynamicB)
            {
                continue;
            }

            if (dynamicA)
            {
                bodyJoints[bodyA].push_back(joint);

                if (discoveredBodies.insert(bodyA).second)
                {
                    bodyOrder.push_back(bodyA);
                }
            }

            if (dynamicB)
            {
                bodyJoints[bodyB].push_back(joint);

                if (discoveredBodies.insert(bodyB).second)
                {
                    bodyOrder.push_back(bodyB);
                }
            }

            m_Stats.Islands = m_Islands.size();
        }

        for (std::size_t contactIndex = 0; contactIndex < m_CurrentContacts.size(); ++contactIndex)
        {
            CollisionManifold2D& manifold = m_CurrentContacts[contactIndex];

            if (!manifold.IsValid() || manifold.IsTrigger)
            {
                continue;
            }

            Rigidbody2D* bodyA = GetColliderBody(manifold.A);

            Rigidbody2D* bodyB = GetColliderBody(manifold.B);

            const bool dynamicA = IsIslandDynamicBody(bodyA);

            const bool dynamicB = IsIslandDynamicBody(bodyB);

            // No awake Dynamic body participates.
            if (!dynamicA && !dynamicB)
            {
                continue;
            }

            if (dynamicA)
            {
                bodyContacts[bodyA].push_back(contactIndex);

                if (discoveredBodies.insert(bodyA).second)
                {
                    bodyOrder.push_back(bodyA);
                }
            }

            if (dynamicB)
            {
                bodyContacts[bodyB].push_back(contactIndex);

                if (discoveredBodies.insert(bodyB).second)
                {
                    bodyOrder.push_back(bodyB);
                }
            }
        }

        std::unordered_set<Rigidbody2D*> visitedBodies;

        for (Rigidbody2D* rootBody : bodyOrder)
        {
            if (!rootBody || visitedBodies.find(rootBody) != visitedBodies.end())
            {
                continue;
            }

            PhysicsIsland2D island;

            std::vector<Rigidbody2D*> stack;

            std::unordered_set<std::size_t> islandContacts;

            std::unordered_set<Joint2D*> islandJoints;

            stack.push_back(rootBody);

            visitedBodies.insert(rootBody);

            // =====================================================
            // DEPTH-FIRST SEARCH
            // =====================================================

            while (!stack.empty())
            {
                Rigidbody2D* body = stack.back();

                stack.pop_back();

                if (!body)
                {
                    continue;
                }

                island.Bodies.push_back(body);

                const auto adjacencyIt = bodyContacts.find(body);

                if (adjacencyIt != bodyContacts.end())
                {
                    for (const std::size_t contactIndex : adjacencyIt->second)
                    {
                        if (contactIndex >= m_CurrentContacts.size())
                        {
                            continue;
                        }

                        // Add each manifold only once to this island.
                        if (islandContacts.insert(contactIndex).second)
                        {
                            island.ContactIndices.push_back(contactIndex);
                        }

                        CollisionManifold2D& manifold = m_CurrentContacts[contactIndex];

                        Rigidbody2D* bodyA = GetColliderBody(manifold.A);

                        Rigidbody2D* bodyB = GetColliderBody(manifold.B);

                        Rigidbody2D* otherBody = nullptr;

                        if (body == bodyA)
                        {
                            otherBody =bodyB;
                        }
                        else if (body == bodyB)
                        {
                            otherBody = bodyA;
                        }

                        // =============================================
                        // Only Dynamic bodies propagate the island graph.
                        // Sleeping Dynamic bodies remain members so wake/sleep
                        // state can propagate across the complete island.
                        // =============================================

                        if (!IsIslandDynamicBody(otherBody))
                        {
                            continue;
                        }

                        if (visitedBodies.insert(otherBody).second)
                        {
                            stack.push_back(otherBody);
                        }
                    }
                }
                
                const auto jointAdjancencyIt = bodyJoints.find(body);

                if (jointAdjancencyIt != bodyJoints.end())
                {
                    for (Joint2D* joint : jointAdjancencyIt->second)
                    {
                        if (!joint || !joint->IsEnabled())
                        {
                            continue;
                        }

                        if (islandJoints.insert(joint).second)
                        {
                            island.Joints.push_back(joint);
                        }

                        Rigidbody2D* bodyA = joint->GetBodyA();

                        Rigidbody2D* bodyB = joint->GetBodyB();

                        Rigidbody2D* otherBody = nullptr;

                        if (body == bodyA)
                        {
                            otherBody = bodyB;
                        }
                        else if (body == bodyB)
                        {
                            otherBody = bodyA;
                        }

                        if (!IsIslandDynamicBody(otherBody))
                        {
                            continue;
                        }

                        if (visitedBodies.insert(otherBody).second)
                        {
                            stack.push_back(otherBody);
                        }
                    }
                }
            }

            if (!island.Empty())
            {
                m_Islands.push_back(std::move(island));
            }
        }
    }

    void PhysicsWorld2D::PrepareVelocityIsland(PhysicsIsland2D& island)
    {
        for (const std::size_t contactIndex : island.ContactIndices)
        {
            if (contactIndex >= m_CurrentContacts.size())
            {
                continue;
            }

            PrepareVelocityContact(m_CurrentContacts[contactIndex]);
        }
    }

    bool PhysicsWorld2D::IsIslandDynamicBody(const Rigidbody2D* body) const
    {
        return body && body->IsDynamic();
    }

    void PhysicsWorld2D::WarmStartVelocityIsland(PhysicsIsland2D& island)
    {
        for (const std::size_t contactIndex : island.ContactIndices)
        {
            if (contactIndex >= m_CurrentContacts.size())
            {
                continue;
            }

            WarmStartVelocityContact(m_CurrentContacts[contactIndex]);
        }
    }

    void PhysicsWorld2D::SolveVelocityIsland(PhysicsIsland2D& island)
    {
        for (std::size_t iteration = 0; iteration < m_Settings.VelocityIterations; ++iteration)
        {
            // CONTACT CONSTRAINTS

            for (const std::size_t contactIndex : island.ContactIndices)
            {

                if (contactIndex >= m_CurrentContacts.size())
                {
                    continue;
                }

                ++m_Stats.VelocityContactSolves;

                SolveVelocityContact(m_CurrentContacts[contactIndex]);
            }

            // JOINT CONSTRAINTS
            
            for (Joint2D* joint : island.Joints)
            {
                if (!joint || !joint->IsEnabled())
                {
                    continue;
                }

                ++m_Stats.JointVelocitySolves;

                joint->SolveVelocity(*this);
            }
        }
    }

    void PhysicsWorld2D::SolvePositionIsland(PhysicsIsland2D& island)
    {
        for (std::size_t iteration = 0; iteration < m_Settings.PositionIterations; ++iteration)
        {
            bool correctedAny = false;

            for (const std::size_t contactIndex : island.ContactIndices)
            {
                if (contactIndex >= m_CurrentContacts.size())
                {
                    continue;
                }

                CollisionManifold2D& manifold = m_CurrentContacts[contactIndex];

                if (manifold.IsTrigger)
                {
                    continue;
                }

                if (!RefreshManifold(manifold))
                {
                    continue;
                }

                if (manifold.Penetration <= 0.0f)
                {
                    continue;
                }

                Entity* entityA = manifold.A->GetOwner();

                Entity* entityB = manifold.B->GetOwner();

                if (!entityA || !entityB)
                {
                    continue;
                }

                TransformComponent* transformA = entityA->GetComponent<TransformComponent>();

                TransformComponent* transformB = entityB->GetComponent<TransformComponent>();

                if (!transformA || !transformB)
                {
                    continue;
                }

                Rigidbody2D* bodyA = entityA->GetComponent<Rigidbody2D>();

                Rigidbody2D* bodyB = entityB->GetComponent<Rigidbody2D>();

                const float inverseMassA = GetSolverInverseMass(bodyA);

                const float inverseMassB = GetSolverInverseMass(bodyB);

                if (inverseMassA + inverseMassB <= 0.0f)
                {
                    continue;
                }

                ++m_Stats.PositionContactSolves;

                ApplyPositionalCorrection(manifold, bodyA, bodyB, *transformA, *transformB);

                correctedAny = true;
            }

            for (Joint2D* joint : island.Joints)
            {
                if (!joint || !joint->IsEnabled())
                {
                    continue;
                }

                ++m_Stats.JointPositionSolves;

                if (joint->SolvePosition(*this))
                {
                    correctedAny = true;
                }
            }

            if (!correctedAny)
            {
                break;
            }
        }
    }

    Rigidbody2D* PhysicsWorld2D::GetColliderBody(Collider2D* collider) const
    {
        if (!collider)
        {
            return nullptr;
        }

        Entity* owner = collider->GetOwner();

        if (!owner)
        {
            return nullptr;
        }

        return owner->GetComponent<Rigidbody2D>();
    }

    std::size_t PhysicsWorld2D::GetIslandCount() const
    {
        return m_Islands.size();
    }


    void PhysicsWorld2D::UpdateIslandSleepStates(float deltaTime)
    {
        std::unordered_set<Rigidbody2D*> islandBodies;

        // Island bodies
        for (PhysicsIsland2D& island : m_Islands)
        {
            for (Rigidbody2D* body : island.Bodies)
            {
                if (body)
                {
                    islandBodies.insert(body);
                }
            }

            UpdateIslandSleepState(island, deltaTime);
        }

        // Isolated dynamic bodies
        if (!m_Scene)
        {
            return;
        }

        for (Entity* root : m_Scene->GetRootEntities())
        {
            UpdateIsolatedBodySleepRecursive(root, deltaTime, islandBodies);
        }
    }


    void PhysicsWorld2D::UpdateIslandSleepState(PhysicsIsland2D& island, float deltaTime)
    {
        if (island.Bodies.empty())
        {
            return;
        }

        bool allBodiesCanSleep = true;

        bool allBodiesAreQuite = true;

        // Determine whether the island is eligible.
        for (Rigidbody2D* body : island.Bodies)
        {
            if (!body)
            {
                continue;
            }

            if (!body->IsDynamic())
            {
                continue;
            }

            if (!body->CanSleep())
            {
                allBodiesCanSleep = false;

                break;
            }

            if (!IsBodyQuietForSleep(*body))
            {
                allBodiesAreQuite = false;
            }
        }

        // A body that cannot sleep keeps the whole island awake.
        if (!allBodiesCanSleep)
        {
            WakeIsland(island);

            return;
        }

        // Any meaningful motion keeps the whole island awake.
        if (!allBodiesAreQuite)
        {
            WakeIsland(island);

            return;
        }

        // Entire island is quiet.
        // Advance all body timers together.
        bool allTimersReady = true;

        for (Rigidbody2D* body : island.Bodies)
        {
            if (!body || !body->IsDynamic())
            {
                continue;
            }

            body->AddSleepTime(deltaTime);

            if (body->GetSleepTimer() < m_Settings.TimeToSleep)
            {
                allTimersReady = false;
            }
        }

        // Sleep as one group.
        if (allTimersReady)
        {
            SleepIsland(island);
        }
    }


    void PhysicsWorld2D::WakeIsland(PhysicsIsland2D& island)
    {
        for (Rigidbody2D* body : island.Bodies)
        {
            if (!body)
            {
                continue;
            }

            if (!body->IsDynamic())
            {
                continue;
            }

            if (body->IsSleeping())
            {
                body->Wake();
            }
            else 
            {
                body->ResetSleepTimer();
            }
        }
    }


    void PhysicsWorld2D::SleepIsland(PhysicsIsland2D& island)
    {
        for (Rigidbody2D* body : island.Bodies)
        {
            if (!body)
            {
                continue;
            }

            if (!body->IsDynamic())
            {
                continue;
            }

            if (!body->CanSleep())
            {
                continue;
            }

            body->Sleep();
        }
    }


    bool PhysicsWorld2D::IsBodyQuietForSleep(const Rigidbody2D& body) const
    {
        const float linearThresholdSquared = m_Settings.SleepLinearSpeedThreshold * m_Settings.SleepLinearSpeedThreshold;

        const bool linearMotionIsSmall = body.GetVelocity().LengthSquared() <= linearThresholdSquared;

        const bool angularMotionIsSmall = std::abs(body.GetAngularVelocity()) <= m_Settings.SleepAngularSpeedThreshold;

        return linearMotionIsSmall && angularMotionIsSmall;
    }


    void PhysicsWorld2D::UpdateIsolatedBodySleepRecursive(Entity* entity, float deltaTime, const std::unordered_set<Rigidbody2D*>& islandBodies)
    {
        if (!entity)
        {
            return;
        }

        Rigidbody2D* body = entity->GetComponent<Rigidbody2D>();

        if (body && body->IsDynamic() && body->CanSleep() && !body->IsSleeping())
        {
            const bool belongsToIsland = islandBodies.find(body) != islandBodies.end();

            if (!belongsToIsland)
            {
                if (IsBodyQuietForSleep(*body))
                {
                    body->AddSleepTime(deltaTime);

                    if (body->GetSleepTimer() >= m_Settings.TimeToSleep)
                    {
                        body->Sleep();
                    }
                }
                else 
                {
                    body->ResetSleepTimer();
                }
            }
        }

        for (const EntityHandle& childHandle : entity->GetChildren())
        {
            UpdateIsolatedBodySleepRecursive(childHandle.Get(), deltaTime, islandBodies);
        }
    }

    
    bool PhysicsWorld2D::IslandNeedsWake(const PhysicsIsland2D& island) const
    {
        for (Rigidbody2D* body : island.Bodies)
        {
            if (!body)
            {
                continue;
            }

            if (!body->IsDynamic())
            {
                continue;
            }

            if (!body->IsSleeping())
            {
                if (!IsBodyQuietForSleep(*body))
                {
                    return true;
                }
            }
        }

        return false;
    }


    void PhysicsWorld2D::PropagateIslandWakeStates()
    {
        for (PhysicsIsland2D& island : m_Islands)
        {
            if (!IslandNeedsWake(island))
            {
                continue;
            }

            WakeIsland(island);
        }
    }


    void PhysicsWorld2D::AddJoint(Joint2D* joint)
    {
        if (!joint)
        {
            return;
        }

        const auto it = std::find(m_Joints.begin(), m_Joints.end(), joint);

        if (it != m_Joints.end())
        {
            return;
        }

        m_Joints.push_back(joint);

        Rigidbody2D* bodyA = joint->GetBodyA();

        Rigidbody2D* bodyB = joint->GetBodyB();

        if (bodyA)
        {
            bodyA->Wake();
        }

        if (bodyB)
        {
            bodyB->Wake();
        }
    }

    void PhysicsWorld2D::RemoveJoint(Joint2D* joint)
    {
        if (!joint)
        {
            return;
        }

        auto it = std::remove(m_Joints.begin(), m_Joints.end(), joint);

        if (it == m_Joints.end())
        {
            return;
        }

        Rigidbody2D* bodyA = joint->GetBodyA();

        Rigidbody2D* bodyB = joint->GetBodyB();

        m_Joints.erase(it, m_Joints.end());

        if (bodyA)
        {
            bodyA->Wake();
        }

        if (bodyB)
        {
            bodyB->Wake();
        }
    }

    const std::vector<Joint2D*>& PhysicsWorld2D::GetJoints() const
    {
        return m_Joints;
    }

    void PhysicsWorld2D::PrepareJoints(PhysicsIsland2D& island, float deltaTime)
    {
        for (Joint2D* joint : island.Joints)
        {
            if (!joint || !joint->IsEnabled())
            {
                continue;
            }

            joint->Prepare(*this, deltaTime);
        }
    }

    void PhysicsWorld2D::WarmStartJoints(PhysicsIsland2D& island)
    {
        for (Joint2D* joint : island.Joints)
        {
            if (!joint || !joint->IsEnabled())
            {
                continue;
            }

            joint->WarmStart(*this);
        }
    }

    float PhysicsWorld2D::GetConstraintInverseMass(const Rigidbody2D* body) const
    {
        return GetSolverInverseMass(body);
    }

    float PhysicsWorld2D::GetConstraintInverseInertia(const Rigidbody2D* body) const
    {
        return GetSolverInverseInertia(body);
    }

    float PhysicsWorld2D::Cross(const Vector2& a, const Vector2& b) const
    {
        return Cross2D(a, b);
    }

    Vector2 PhysicsWorld2D::GetConstraintPointVelocity(const Vector2& linearVelocity, float angularVelocity, const Vector2& leverArm) const
    {
        return GetVelocityAtPoint(linearVelocity, angularVelocity, leverArm);
    }

    PhysicsWorld2D::ClosestSegmentPoints2D PhysicsWorld2D::ClosestPointsBetweenSegments(const Vector2& a0, const Vector2& a1, const Vector2& b0, const Vector2& b1) const
    {
        ClosestSegmentPoints2D result;

        const Vector2 d1 = a1 - a0;

        const Vector2 d2 = b1 - b0;

        const Vector2 r = a0 - b0;

        const float a = Vector2::Dot(d1, d1);

        const float e = Vector2::Dot(d2, d2);

        const float f = Vector2::Dot(d2, r);

        constexpr float epsilon = 0.000001f;

        float s = 0.0f;

        float t = 0.0f;

        // BOTH SEGMENTS ARE POINTS

        if (a <= epsilon && e <= epsilon)
        {
            result.PointA = a0;

            result.PointB = b0;

            return result;
        }

        // A IS A POINT

        if (a <= epsilon)
        {
            s = 0.0f;

            t = std::clamp(f / e, 0.0f, 1.0f);
        }
        else 
        {
            const float c = Vector2::Dot(d1, r);

            // B IS A POINT

            if (e <= epsilon)
            {
                t = 0.0f;

                s = std::clamp(-c / a, 0.0f, 1.0f);
            }
            else 
            {
                const float b = Vector2::Dot(d1, d2);

                const float denominator = a * e - b * b;

                if (std::abs(denominator) > epsilon)
                {
                    s = std::clamp((b * f - c * e) / denominator, 0.0f, 1.0f);
                }
                else 
                {
                    // Nearly parallel segements.

                    s = 0.0f;
                }

                t = (b * s + f) / e;

                if (t < 0.0f)
                {
                    t = 0.0f;

                    s = std::clamp(-c / a, 0.0f, 1.0f);
                }
                else if (t > 1.0f)
                {
                    t = 1.0f;

                    s = std::clamp((b - c) / a, 0.0f, 1.0f);
                }
            }
        }

        result.PointA = a0 + d1 * s;

        result.PointB = b0 + d2 * t;

        return result;
    }

    Vector2 PhysicsWorld2D::GetPolygonSupportPoint(const Polygon2D& polygon, const Vector2& direction) const
    {
        if (polygon.Vertices.empty())
        {
            return{0.0f, 0.0f};
        }

        std::size_t bestIndex = 0;

        float bestProjection = Vector2::Dot(polygon.Vertices[0], direction);

        for (std::size_t i = 1; i < polygon.Vertices.size(); ++i)
        {
            const float projection = Vector2::Dot(polygon.Vertices[i], direction);

            if (projection > bestProjection)
            {
                bestProjection = projection;

                bestIndex = i;
            }
        }

        return polygon.Vertices[bestIndex];
    }

    void PhysicsWorld2D::ProjectPolygon(const Polygon2D& polygon, const Vector2& axis, float& outMin, float& outMax) const
    {
        if (polygon.Vertices.empty())
        {
            outMin = 0.0f;

            outMax = 0.0f;

            return;
        }

        outMin = Vector2::Dot(polygon.Vertices[0], axis);

        outMax = outMin;

        for (std::size_t i = 1; i < polygon.Vertices.size(); ++i)
        {
            const float projection = Vector2::Dot(polygon.Vertices[i], axis);

            outMin = std::min(outMin, projection);

            outMax = std::max(outMax, projection);
        }
    }

    Vector2 PhysicsWorld2D::GetPolygonCenter(const Polygon2D& polygon) const
    {
        if (polygon.Vertices.empty())
        {
            return{0.0f, 0.0f};
        }

        Vector2 center{0.0f, 0.0f};

        for (const Vector2& vertex : polygon.Vertices)
        {
            center += vertex;
        }

        center *= 1.0f / static_cast<float>(polygon.Vertices.size());

        return center;
    }

    bool PhysicsWorld2D::TestPolygonAxis(const Polygon2D& a, const Polygon2D& b, const Vector2& axis, float& outOverlap) const
    {
        constexpr float epsilon = 0.000001f;

        if (axis.LengthSquared() <= epsilon)
        {
            outOverlap = 0.0f;

            return true;
        }

        float minA = 0.0f;
        float maxA = 0.0f;

        float minB = 0.0f;
        float maxB = 0.0f;

        ProjectPolygon(a, axis, minA, maxA);

        ProjectPolygon(b, axis, minB, maxB);

        outOverlap = std::min(maxA, maxB) - std::max(minA, minB);

        return outOverlap > 0.0f;
    }

    PhysicsWorld2D::PolygonSATResult2D PhysicsWorld2D::TestPolygonSAT(const Polygon2D& a, const Polygon2D& b) const
    {
        PolygonSATResult2D result;

        if (a.Vertices.size() < 3 || b.Vertices.size() < 3)
        {
            return result;
        }

        float minimumOverlap = std::numeric_limits<float>::max();

        Vector2 minimumAxis{0.0f, 0.0f};

        bool minimumAxisFromA = true;

        std::size_t minimumAxisIdex = 0;

        // AXES FROM A

        for (std::size_t i = 0; i < a.Normals.size(); ++i)
        {
            const Vector2& axis = a.Normals[i];

            if (axis.LengthSquared() <= 0.000001f)
            {
                continue;
            }

            float overlap = 0.0f;

            if (!TestPolygonAxis(a, b, axis, overlap))
            {
                return result;
            }

            if (overlap < minimumOverlap)
            {
                minimumOverlap = overlap;

                minimumAxis = axis;

                minimumAxisFromA = true;

                minimumAxisIdex = i;
            }
        }

        // AXES FROM B

        for (std::size_t i = 0; i < b.Normals.size(); ++i)
        {
            const Vector2& axis = b.Normals[i];

            if (axis.LengthSquared() <= 0.000001f)
            {
                continue;
            }

            float overlap = 0.0f;

            if (!TestPolygonAxis(a, b, axis, overlap))
            {
                return result;
            }

            if (overlap < minimumOverlap)
            {
                minimumOverlap = overlap;

                minimumAxis = axis;

                minimumAxisFromA = false;

                minimumAxisIdex = i;
            }
        }

        if (minimumOverlap == std::numeric_limits<float>::max())
        {
            return result;
        }

        // FORCE NORMAL A -> B

        const Vector2 centerA = GetPolygonCenter(a);

        const Vector2 centerB = GetPolygonCenter(b);

        if (Vector2::Dot(centerB - centerA, minimumAxis) < 0.0f)
        {
            minimumAxis *= -1.0f;
        }

        result.Overlapping = true;

        result.Axis = minimumAxis;

        result.Overlap = minimumOverlap;

        result.AxisFromA = minimumAxisFromA;

        result.AxisIndex = minimumAxisIdex;

        return result;
    }

    PhysicsWorld2D::PolygonEdge2D PhysicsWorld2D::GetPolygonEdge(const Polygon2D& polygon, std::size_t edgeIndex) const
    {
        PolygonEdge2D edge;

        if (polygon.Vertices.empty())
        {
            return edge;
        }

        const std::size_t count = polygon.Vertices.size();

        edgeIndex %= count;

        const std::size_t nestIndex = (edgeIndex + 1) % count;

        edge.A = polygon.Vertices[edgeIndex];

        edge.B = polygon.Vertices[nestIndex];


        if (edgeIndex < polygon.Normals.size())
        {
            edge.Normal = polygon.Normals[edgeIndex];
        }

        edge.Index = edgeIndex;

        return edge;
    }

    PhysicsWorld2D::PolygonEdge2D PhysicsWorld2D::FindIncidentPolygonEdge(const Polygon2D& polygon, const Vector2& referenceNormal) const
    {
        PolygonEdge2D result;

        if (polygon.Normals.empty())
        {
            return result;
        }

        std::size_t bestIndex = 0;

        float smallestDot = Vector2::Dot(polygon.Normals[0], referenceNormal);

        for (std::size_t i = 1; i < polygon.Normals.size(); ++i)
        {
            const float dot = Vector2::Dot(polygon.Normals[i], referenceNormal);

            if (dot < smallestDot)
            {
                smallestDot = dot;

                bestIndex = i;
            }
        }

        return GetPolygonEdge(polygon, bestIndex);
    }

    std::size_t PhysicsWorld2D::BuildPolygonContactPoints(const Polygon2D& polygonA, const Polygon2D& polygonB, const PolygonSATResult2D& sat, Vector2 outContacts[2]) const
    {
        // REFERENCE / INCIDENT

        const Polygon2D& reference = sat.AxisFromA ? polygonA : polygonB;

        const Polygon2D& incident = sat.AxisFromA ? polygonB : polygonA;

        if (reference.Vertices.empty() || incident.Vertices.empty())
        {
            return 0;
        }

        // REFERENCE EDGE

        const PolygonEdge2D referenceEdge = GetPolygonEdge(reference, sat.AxisIndex);

        Vector2 referenceNormal = referenceEdge.Normal;

        // Make the normal point:
        //
        // reference -> incident

        const Vector2 referenceCenter = GetPolygonCenter(reference);

        const Vector2 incidentCenter = GetPolygonCenter(incident);

        if (Vector2::Dot(incidentCenter - referenceCenter, referenceNormal) < 0.0f)
        {
            referenceNormal *= -1.0f;
        }

        // REFERENCE TANGENT

        Vector2 referenceTangent = referenceEdge.B - referenceEdge.A;

        const float tangentLengthSquared = referenceTangent.LengthSquared();

        constexpr float epsilon = 0.000001f;

        if (tangentLengthSquared <= epsilon)
        {
            return 0;
        }

        const float tangentLength = std::sqrt(tangentLengthSquared);

        referenceTangent *= 1.0f / tangentLength;

        const Vector2 referenceFaceCenter = (referenceEdge.A + referenceEdge.B) * 0.5f;

        const float referenceHalfLength = tangentLength * 0.5f;

        // INCIDENT EDGE

        const PolygonEdge2D incidentEdge = FindIncidentPolygonEdge(incident, referenceNormal);

        // CLIP AGAINST SIDE PLANES

        Vector2 clipped[2];

        const std::size_t clippedCount = ClipSegmentToSpan(incidentEdge.A, incidentEdge.B, referenceFaceCenter, referenceTangent, referenceHalfLength, clipped);

        if (clippedCount == 0)
        {
            return 0;
        }

        // KEEP POINTS BEHIND REFERENCE FACE

        std::size_t contactCount = 0;

        constexpr float contactTolerance = 0.001f;

        for (std::size_t i = 0; i < clippedCount; ++i)
        {
            const float separation = Vector2::Dot(clipped[i] - referenceFaceCenter, referenceNormal);

            if (separation > contactTolerance)
            {
                continue;
            }

            outContacts[contactCount++] = clipped[i] - referenceNormal * (separation * 0.5f);

            if (contactCount == 2)
            {
                break;
            }
        }

        return contactCount;
    }

    Polygon2D PhysicsWorld2D::OrientedBoxToPolygon(const OrientedBox2D& box) const
    {
        Polygon2D polygon;

        polygon.Vertices.reserve(4);

        polygon.Normals.reserve(4);

        for (std::size_t i = 0; i < 4; ++i)
        {
            polygon.Vertices.push_back(box.Vertices[i]);
        }

        constexpr float epsilon = 0.000001f;

        for (std::size_t i = 0; i < 4; ++i)
        {
            const Vector2& current = polygon.Vertices[i];

            const Vector2& next = polygon.Vertices[(i + 1) % 4];

            const Vector2 edge = next - current;

            Vector2 normal{-edge.Y, edge.X};

            const float lengthSquared = normal.LengthSquared();

            if (lengthSquared > epsilon)
            {
                normal *= 1.0f / std::sqrt(lengthSquared);
            }
            else 
            {
                normal = {0.0f, 0.0f};
            }

            polygon.Normals.push_back(normal);
        }

        return polygon;
    }


    Vector2 PhysicsWorld2D::FindClosestPolygonVertex(const Polygon2D& polygon, const Vector2& point) const
    {
        if (polygon.Vertices.empty())
        {
            return{0.0f, 0.0f};
        }

        std::size_t closestIndex = 0;

        float closestDistanceSquared = (polygon.Vertices[0] - point).LengthSquared();

        for (std::size_t i = 1; i < polygon.Vertices.size(); ++i)
        {
            const float distanceSquared = (polygon.Vertices[i] - point).LengthSquared();

            if (distanceSquared < closestDistanceSquared)
            {
                closestDistanceSquared = distanceSquared;

                closestIndex = i;
            }
        }

        return polygon.Vertices[closestIndex];
    }

    void PhysicsWorld2D::ProjectCircle(const Vector2& center, float radius, const Vector2& axis, float& outMin, float& outMax) const
    {
        const float projection = Vector2::Dot(center, axis);

        outMin = projection - radius;

        outMax = projection + radius;
    }

    Vector2 PhysicsWorld2D::GetBodyStepMotion(const Rigidbody2D* body, const TransformComponent* transform) const
    {
        if (!body || !transform)
        {
            return{0.0f, 0.0f};
        }

        if (body->IsStatic())
        {
            return{0.0f, 0.0f};
        }

        const Vector2 currentPosition = transform->GetWorldTransform().Position;

        return currentPosition - body->GetPreviousPosition();
    }

    PhysicsWorld2D::SweptAxisResult2D PhysicsWorld2D::SweepIntervalsOnAxis(float minA, float maxA, float minB, float maxB, float relativeSpeed, const Vector2& axis) const
    {
        SweptAxisResult2D result;

        constexpr float epsilon = 0.000001f;

        // NO RELATIVE MOTION ON THIS AXIS

        if (std::abs(relativeSpeed) <= epsilon)
        {
            // 
            if (maxA < minB || maxB < minA)
            {
                result.Valid = false;

                return result;
            }

            // Already overlapping on this axis for the entire interval.

            result.EntryTime = -std::numeric_limits<float>::infinity();

            result.ExitTime = std::numeric_limits<float>::infinity();

            return result;
        }

        // TIMES AT WHICH INTERVAL BOUNDARIES MEET

        const float t1 = (minB - maxA) / relativeSpeed;

        const float t2 = (maxB - minA) / relativeSpeed;

        result.EntryTime = std::min(t1, t2);

        result.ExitTime = std::max(t1, t2);

        // TARGET OUTWARD NORMAL

        if (relativeSpeed > 0.0f)
        {
            result.Normal = axis * -1.0f;
        }
        else 
        {
            result.Normal = axis;
        }

        return result;
    }

    const PhysicsSettings2D& PhysicsWorld2D::GetSettings() const
    {
        return m_Settings;
    }

    void PhysicsWorld2D::SetSettings(const PhysicsSettings2D& settings)
    {
        m_Settings = settings;

        ValidateSettings();
    }

    PhysicsSettings2D& PhysicsWorld2D::GetSettings()
    {
        return m_Settings;
    }

    void PhysicsWorld2D::ValidateSettings()
    {
        m_Settings.FixedDeltaTime = std::max(m_Settings.FixedDeltaTime, 0.000001f);

        m_Settings.MaxSubSteps = std::max<std::size_t>(1, m_Settings.MaxSubSteps);

        m_Settings.VelocityIterations = std::max<std::size_t>(1, m_Settings.VelocityIterations);

        m_Settings.PositionIterations = std::max<std::size_t>(1, m_Settings.PositionIterations);

        m_Settings.PositionSlop = std::max(0.0f, m_Settings.PositionSlop);

        m_Settings.PositionCorrectionPercent = std::clamp(m_Settings.PositionCorrectionPercent, 0.0f, 1.0f);

        m_Settings.RestitutionVelocityThreshold = std::max(0.0f, m_Settings.RestitutionVelocityThreshold);

        m_Settings.SleepLinearSpeedThreshold = std::max(0.0f, m_Settings.SleepLinearSpeedThreshold);

        m_Settings.SleepAngularSpeedThreshold = std::max(0.0f, m_Settings.SleepAngularSpeedThreshold);

        m_Settings.TimeToSleep = std::max(0.0f, m_Settings.TimeToSleep);

        m_Settings.CollisionWakeSpeed = std::max(0.0f, m_Settings.CollisionWakeSpeed);

        m_Settings.SpatialCellSize = std::max(1.0f, m_Settings.SpatialCellSize);

        m_Settings.MaxCCDImpacts = std::max<std::size_t>(1, m_Settings.MaxCCDImpacts);

        m_Settings.CCDTimeEpsilon = std::max(0.0f, m_Settings.CCDTimeEpsilon);

        m_Settings.CCDSeparation = std::max(0.0f, m_Settings.CCDSeparation);
    }

    const PhysicsStats2D& PhysicsWorld2D::GetStats() const
    {
        return m_Stats;
    }

    const std::vector<PhysicsCCDDebug2D>& PhysicsWorld2D::GetCCDDebugRecords() const
    {
        return m_CCDDebugRecords;
    }

    void PhysicsWorld2D::ResetStepStats(float deltaTime)
    {
        const std::uint64_t previousStepIndex = m_Stats.StepIndex;

        m_Stats = PhysicsStats2D{} ;

        m_Stats.StepIndex = previousStepIndex + 1;

        m_Stats.StepDeltaTime = deltaTime;
    }

    void PhysicsWorld2D::CountBodiesForStats()
    {
        m_Stats.DynamicBodies = 0;

        m_Stats.KinematicBodies = 0;

        m_Stats.StaticBodies = 0;

        m_Stats.SleepingBodies = 0;

        if (!m_Scene)
        {
            return;
        }

        const auto countEntity = 
           [&](auto&& self, Entity* entity) -> void
           {
                if (!entity)
                {
                    return;
                }

                Rigidbody2D* body = entity->GetComponent<Rigidbody2D>();

                if (body)
                {
                    switch (body->GetBodyType())
                    {
                        case BodyType2D::Static:
                            ++m_Stats.StaticBodies;
                            break;

                        case BodyType2D::Kinematic:
                            ++m_Stats.KinematicBodies;
                            break;

                        case BodyType2D::Dynamic:
                            ++m_Stats.DynamicBodies;

                            if (body->IsSleeping())
                            {
                                ++m_Stats.SleepingBodies;
                            }
                            break;
                    }
                }

                for (const EntityHandle& childHandle : entity->GetChildren())
                {
                    self(self, childHandle.Get());
                }
           };
        
        for (Entity* root : m_Scene->GetRootEntities())
        {
            countEntity(countEntity, root);
        }
    }

    void PhysicsWorld2D::FinalizeStepStats()
    {
        m_Stats.ActiveColliders = m_ActiveColliders.size();


        m_Stats.SpatialCells = m_SpatialGrid.size();


        m_Stats.BroadPhaseProxies = m_BroadPhaseProxies.size();


        m_Stats.CandidatePairs = m_CandidatePairs.size();


        m_Stats.Islands = m_Islands.size();


        m_Stats.Joints = m_Joints.size();


        m_Stats.CachedContactPairs = m_ContactCache.size();


        m_Stats.ActiveJointConstraints = 0;


        for (Joint2D* joint : m_Joints)
        {
            if (joint &&joint->IsEnabled())
            {
                ++m_Stats.ActiveJointConstraints;
            }
        }


        CountBodiesForStats();
    }

    void PhysicsWorld2D::PrintPhysicsStats() const
    {
        std::cout
            << "\n=== Physics Stats ===\n"

            << "Step: "
            << m_Stats.StepIndex
            << '\n'

            << "Colliders: "
            << m_Stats.ActiveColliders
            << '\n'

            << "Dynamic bodies: "
            << m_Stats.DynamicBodies
            << '\n'

            << "Sleeping bodies: "
            << m_Stats.SleepingBodies
            << '\n'

            << "Spatial cells: "
            << m_Stats.SpatialCells
            << '\n'

            << "Candidate pairs: "
            << m_Stats.CandidatePairs
            << '\n'

            << "Narrow tests: "
            << m_Stats.NarrowPhaseTests
            << '\n'

            << "Manifolds: "
            << m_Stats.GeneratedManifolds
            << '\n'

            << "Contact points: "
            << m_Stats.ContactPoints
            << '\n'

            << "Islands: "
            << m_Stats.Islands
            << '\n'

            << "Joints: "
            << m_Stats.ActiveJointConstraints
            << '\n'

            << "CCD bodies: "
            << m_Stats.CCDBodies
            << '\n'

            << "CCD tests: "
            << m_Stats.CCDNarrowPhaseTests
            << '\n'

            << "CCD hits: "
            << m_Stats.CCDHits
            << '\n'

            << "CCD resolved: "
            << m_Stats.CCDResolvedImpacts
            << '\n'

            << "=====================\n";
    }

    float PhysicsWorld2D::GetBroadPhaseRejectionRatio() const
    {
        if (m_Stats.CandidatePairs == 0)
        {
            return 0.0f;
        }

        const float narrowRatio = static_cast<float>(m_Stats.NarrowPhaseTests) / static_cast<float>(m_Stats.CandidatePairs);

        return 1.0f - narrowRatio;
    }

    const PhysicsDebugDrawSettings2D& PhysicsWorld2D::GetDebugDrawSettings() const
    {
        return m_DebugDrawSettings;
    }

    PhysicsDebugDrawSettings2D& PhysicsWorld2D::GetDebugDrawSettings()
    {
        return m_DebugDrawSettings;
    }

    PhysicsDebugSnapshot2D PhysicsWorld2D::GetDebugSnapshot() const
    {
        PhysicsDebugSnapshot2D snapshot;

        snapshot.Settings = m_Settings;

        snapshot.Stats = m_Stats;

        snapshot.DrawSettings = m_DebugDrawSettings;

        return snapshot;
    }
}