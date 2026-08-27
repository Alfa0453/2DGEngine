#include "PhysicsWorld2D.h"

#include "BoxCollider2D.h"
#include "BroadPhaseProxy2D.h"
#include "CircleCollider2D.h"
#include "Collider2D.h"
#include "ColliderPair2D.h"
#include "CollisionEvents2D.h"
#include "CollisionManifold2D.h"
#include "OrientedBox2D.h"
#include "OverlapHit2D.h"
#include "PhysicsQueryContext2D.h"
#include "PhysicsQueryFilter2D.h"
#include "RaycastHit2D.h"
#include "Rigidbody2D.h"
#include "ShapeCastHit2D.h"
#include "SpatialCell2D.h"
#include "CollisionDetectionMode2D.h"
#include "SweepHit2D.h"
#include "SweptAABBHit2D.h"

#include "../Scene/Scene.h"
#include "../Scene/Entity.h"
#include "../Math/Bounds2D.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>

namespace Engine
{
    PhysicsWorld2D::PhysicsWorld2D(Scene* scene)
        : m_Scene(scene)
    {
    }

    void PhysicsWorld2D::SetScene(Scene* scene)
    {
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

        while (m_Accumulator >= m_FixedDeltaTime && subSteps < m_MaxSubSteps)
        {
            Step(m_FixedDeltaTime);

            m_Accumulator -= m_FixedDeltaTime;

            ++subSteps;
        }

        if (subSteps == m_MaxSubSteps)
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

        // =====================================
        // 1. Tentative integration
        // =====================================
        
        IntegrateBodies(deltaTime);

        // =====================================
        // 2. Pre-CCD spatial structure
        // =====================================
        
        CollectColliders();

        BuildSpatialGrid();

        // =====================================
        // 3. CCD
        // =====================================

        m_SweptTriggerPairsThisStep.clear();

        RunContinuousCollisionPass(deltaTime);

        // =====================================
        // 4. CCD changed transforms.
        // Rebuild final broad-phase state.
        // =====================================
        
        CollectColliders();

        BuildSpatialGrid();

        GenerateCondidatePairs();

        // =====================================
        // 5. Regular discrete contacts
        // =====================================

        m_CurrentOverlaps.clear();

        m_CurrentContacts.clear();


        ProcessCandidatePairs();

        // Wake sleeping bodies
        WakeBodiesFromContacts();

        // Solve velocity constraints.
        SolveVelocityContacts();

        // Solve penetration constraints.
        SolvePositionContacts();

        // Sleep evaluation
        UpdateSleepStates(deltaTime);

        // Collision events
        PublishPairEvents();

        // Store overlap state
        m_PreviousOverlaps = m_CurrentOverlaps;
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

        for (const EntityHandle& childHandle : entity->GetChildren())
        {
            CollectCollidersRecursive(childHandle.Get());
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
            const Vector2 pointA = GetOBBSurpportPoint(boxA, manifold.Normal);

            const Vector2 pointB = GetOBBSurpportPoint(boxB, manifold.Normal * -1.0f);

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

        const float distanceSquared = delta.LengthSqured();

        const float combinedRadius = radiusA + radiusB;

        const float combinedRadiusSquared = combinedRadius * combinedRadius;

        if (distanceSquared >= combinedRadiusSquared)
        {
            return false;
        }

        manifold.A = &a;

        manifold.B = &b;

        manifold.IsTrigger = a.IsTrigger() || b.IsTrigger();

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

    bool PhysicsWorld2D::BoxVsCircle(BoxCollider2D& box, CircleCollider2D& circle, CollisionManifold2D& maniflod) const
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

        const float distanceSquared = localDelta.LengthSqured();

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

        maniflod.A = &box;

        maniflod.B = &circle;

        maniflod.IsTrigger = box.IsTrigger() || circle.IsTrigger();

        maniflod.ClearContacts();

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

            maniflod.Normal = OBBLocalDirectionToWorld(orientedBox, localNormal);

            maniflod.Penetration = radius - distance;

            if (maniflod.Penetration <= 0.0f)
            {
                return false;
            }

            // -----------------------------------------------------
            // Convert closest point back to world space.
            // -----------------------------------------------------

            maniflod.AddContactPoint(OBBLocalPointToWorld(orientedBox, localClosestPoint));

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

        maniflod.Normal = OBBLocalDirectionToWorld(orientedBox, localNormal);


        // =========================================================
        // PENETRATION
        // =========================================================

        maniflod.Penetration = radius + nearest;

        if (maniflod.Penetration <= 0.0f)
        {
            return false;
        }


        // =========================================================
        // LOCAL CONTACT -> WORLD CONTACT
        // =========================================================

        maniflod.AddContactPoint(OBBLocalPointToWorld(orientedBox, localContact));


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

        outOverlap = std::min(maxA, maxB) - std::max(minA, minB);

        return outOverlap > 0.0f;
    }


    Vector2 PhysicsWorld2D::GetOBBSurpportPoint(const OrientedBox2D& box, const Vector2& direction) const
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

    std::size_t PhysicsWorld2D::GetAciveColliderCount() const
    {
        return m_ActiveColliders.size();
    }

    std::size_t PhysicsWorld2D::GetCurrentOverlapCount() const
    {
        return m_CurrentOverlaps.size();
    }

    void PhysicsWorld2D::SetGravity(const Vector2& gravity)
    {
        m_Gravity = gravity;
    }

    const Vector2& PhysicsWorld2D::GetGravity() const
    {
        return m_Gravity;
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

        acceleration += m_Gravity * body.GetGravityScale();

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
        // Move entity
        // -------------------------------

        transform.Translate(velocity * deltaTime);

        // -------------------------------
        // Forces last one step
        // -------------------------------

        body.ClearForces();
    }

    void PhysicsWorld2D::IntegrateKinematicBody(Rigidbody2D& body, TransformComponent& transform, float deltaTime)
    {
        transform.Translate(body.GetVelocity() * deltaTime);

        body.ClearForces();
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

        constexpr float slop = 0.01f;

        constexpr float percent = 0.35f;

        const float correctedPenetration = std::max(manifold.Penetration - slop, 0.0f);

        const Vector2 correction = manifold.Normal * (correctedPenetration * percent / totalInverseMass);

        if (inverseMassA > 0.0f)
        {
            transformA.Translate(correction * -inverseMassA);
        }

        if (inverseMassB > 0.0f)
        {
            transformB.Translate(correction * inverseMassB);
        }
    }

    void PhysicsWorld2D::ApplyVelocityResponse(const CollisionManifold2D& manifold, Rigidbody2D* bodyA, Rigidbody2D* bodyB)
    {
        // =========================================================
        // GET SOLVER INVERSE MASSES
        // =========================================================

        const float inverseMassA = GetSolverInverseMass(bodyA);

        const float inverseMassB = GetSolverInverseMass(bodyB);

        const float totalInverseMass = inverseMassA + inverseMassB;

        // If neither body can move, there is nothing to solve.
        if (totalInverseMass <= 0.0f)
        {
            return;
        }

        // =========================================================
        // GET CURRENT VELOCITIES
        // =========================================================

        Vector2 velocityA = bodyA ? bodyA->GetVelocity() : Vector2{0.0f, 0.0f};

        Vector2 velocityB = bodyB ? bodyB->GetVelocity() : Vector2{0.0f, 0.0f};

        // =========================================================
        // CALCULATE RELATIVE VELOCITY
        // =========================================================

        Vector2 relativeVelocity = velocityB - velocityA;

        // =========================================================
        // FIND VELOCITY ALONG COLLISION NORMAL
        // =========================================================

        const float velocityAlongNormal = Vector2::Dot(relativeVelocity, manifold.Normal);

        // =========================================================
        // DON'T SOLVE IF OBJECTS ARE ALREADY SEPARATING
        // =========================================================

        if (velocityAlongNormal > 0.0f)
        {
            return;
        }

        // =========================================================
        // COMBINE RESTITUTION
        // =========================================================

        float restitution = CombineRestitution(*manifold.A, *manifold.B);

        // =========================================================
        // REMOVE BOUNCE AT VERY LOW IMPACT SPEEDS
        // =========================================================

        constexpr float restitutionVelocityThreshold = 20.0f;

        if (std::abs(velocityAlongNormal) < restitutionVelocityThreshold)
        {
            restitution = 0.0f;
        }

        // =========================================================
        // CALCULATE NORMAL IMPULSE MAGNITUDE
        // =========================================================

        const float normalImpulseMagnitude = -(1.0f + restitution) * velocityAlongNormal / totalInverseMass;

        // =========================================================
        // CREATE NORMAL IMPULSE VECTOR
        // =========================================================

        const Vector2 normalImpulse = manifold.Normal * normalImpulseMagnitude;

        // =========================================================
        // APPLY NORMAL IMPULSE TO LOCAL VELOCITIES
        // =========================================================

        if (bodyA && inverseMassA > 0.0f)
        {
            velocityA -= normalImpulse * inverseMassA;
        }

        if (bodyB && inverseMassB > 0.0f)
        {
            velocityB += normalImpulse * inverseMassB;
        }

        // =========================================================
        // RECALCULATE RELATIVE VELOCITY
        // =========================================================

        // The normal impulse just changed velocityA and velocityB.
        // Friction must use these new velocities.

        relativeVelocity = velocityB - velocityA;

        // =========================================================
        // CALCULATE THE TANGENT
        // =========================================================

        // Remove the part of relative velocity that lies along the collision normal.
        // What remains is movement ACROSS the surface.

        Vector2 tangent = relativeVelocity - manifold.Normal * Vector2::Dot(relativeVelocity, manifold.Normal);

        // =========================================================
        // CHECK WHETHER A USEFUL TANGENT EXISTS
        // =========================================================

        const float tangentLengthSquared = tangent.LengthSqured();

        if (tangentLengthSquared > 0.000001f)
        {
            // =====================================================
            // NORMALIZE THE TANGENT
            // =====================================================

            tangent *= 1.0f / std::sqrt(tangentLengthSquared);

            // =====================================================
            // CALCULATE REQUIRED TANGENTIAL IMPULSE
            // =====================================================

            const float tangentImpulseMagnitude = -Vector2::Dot(relativeVelocity, tangent) / totalInverseMass;

            // =====================================================
            // COMBINE STATIC FRICTION
            // =====================================================

            const float staticFriction = CombineStaticFriction(*manifold.A, *manifold.B);

            // =====================================================
            // COMBINE DYNAMIC FRICTION
            // =====================================================

            const float dynamicFrictin = CombineDynamicFriction(*manifold.A, *manifold.B);

            Vector2 frictionImpulse;

            // STATIC FRICTION

            // can static friction completely stop the sliding?

            if (std::abs(tangentImpulseMagnitude) <= normalImpulseMagnitude * staticFriction)
            {
                frictionImpulse = tangent * tangentImpulseMagnitude;
            }

            // DYNAMIC FRICTION

            // Static friction wasn't strong enough, so the surfaces are sliding.

            else {
                const float frictionMagnitude = normalImpulseMagnitude * dynamicFrictin;

                const float fricionDirection = tangentImpulseMagnitude < 0.0f ? -1.0f : 1.0f;

                frictionImpulse = tangent * frictionMagnitude * fricionDirection;
            }

            // APPLY FRICTION IMPULSE TO A

            if (bodyA && inverseMassA > 0.0f)
            {
                velocityA -= frictionImpulse * inverseMassA;
            }

            if (bodyB && inverseMassB > 0.0f)
            {
                velocityB += frictionImpulse * inverseMassB;
            }
        }

        // WRITE FINAL VELOCITIES BACK TO THE RIGIDBODIES

        if (bodyA)
        {
            bodyA->SetVelocityFromPhysics(velocityA);
        }

        if (bodyB)
        {
            bodyB->SetVelocityFromPhysics(velocityB);
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
        m_VelocityIterations = std::max<std::size_t>(1, iterations);
    }

    std::size_t PhysicsWorld2D::GetVelocityIterations() const
    {
        return m_VelocityIterations;
    }

    void PhysicsWorld2D::SetPositionIterations(std::size_t iterations)
    {
        m_PositionIterations = std::max<std::size_t>(1, iterations);
    }

    std::size_t PhysicsWorld2D::GetPositionIterations() const
    {
        return m_PositionIterations;
    }

    void PhysicsWorld2D::SolveVelocityContact(const CollisionManifold2D& manifold)
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

        if (inverseMassA + inverseMassB <= 0.0f)
        {
            return;
        }

        ApplyVelocityResponse(manifold, bodyA, bodyB);
    }

    void PhysicsWorld2D::SolveVelocityContacts()
    {
        for (std::size_t iteration = 0; iteration < m_VelocityIterations; ++iteration)
        {
            for (const CollisionManifold2D& manifold : m_CurrentContacts)
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
        for (std::size_t iteration = 0; iteration < m_PositionIterations; ++iteration)
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

    void PhysicsWorld2D::WakeBodiesFromContacts()
    {
        const float wakeSpeedSquared = m_CollisionWakeSpeed * m_CollisionWakeSpeed;

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

            const Vector2 velocityA = bodyA ? bodyA->GetVelocity() : Vector2{0.0f, 0.0f};

            const Vector2 velocityB = bodyB ? bodyB->GetVelocity() : Vector2{0.0f, 0.0f};

            const Vector2 relativeVelocity = velocityB - velocityA;

            if (relativeVelocity.LengthSqured() < wakeSpeedSquared)
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
            const float thresholdSquared = m_SleepLinearSpeedThreshold * m_SleepLinearSpeedThreshold;

            if (body->GetVelocity().LengthSqured() <= thresholdSquared)
            {
                body->AddSleepTime(deltaTime);

                if (body->GetSleepTimer() >= m_TimeToSleep)
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
        m_SpatialCellSize = std::max(1.0f, cellSize);
    }

    float PhysicsWorld2D::GetSpatialCellSize() const
    {
        return m_SpatialCellSize;
    }

    SpatialCell2D PhysicsWorld2D::WorldToCell(const Vector2& worldPosition) const
    {
        return 
        {
            static_cast<std::int32_t>(std::floor(worldPosition.X / m_SpatialCellSize)),

            static_cast<std::int32_t>(std::floor(worldPosition.Y / m_SpatialCellSize))
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

    void PhysicsWorld2D::GenerateCondidatePairs()
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

            if (!GenerateManifold(*a, *b, manifold))
            {
                continue;
            }

            // Real contact

            m_CurrentOverlaps.insert(pair);

            m_CurrentContacts.push_back(manifold);
        }
    }

    std::size_t PhysicsWorld2D::GetSpatialCellCount() const
    {
        return m_SpatialGrid.size();
    }

    std::size_t PhysicsWorld2D::GetCandidatePirCount() const
    {
        return m_CandidatePairs.size();
    }

    const std::vector<Collider2D*>& PhysicsWorld2D::GetactiveColliders() const
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

    bool PhysicsWorld2D::FindEarliestContinuousHit(Collider2D* movingCollider, const Bounds2D& startBounds, const Vector2& motion, SweepHit2D& outHit, Collider2D*& outOtherCollider)
    {
        if (!movingCollider)
        {
            return false;
        }

        // -----------------------------------------
        // No movement = nothing to sweep.
        // -----------------------------------------

        if (motion.LengthSqured() <= 0.000001f)
        {
            return false;
        }

        // -----------------------------------------
        // Build AABB covering the entire motion.
        // -----------------------------------------

        const Bounds2D sweptBounds = BuildSweptBounds(startBounds, motion);

        // -----------------------------------------
        // Spatial query.
        // -----------------------------------------

        std::vector<Collider2D*> candidates;

        PhysicsQueryFilter2D filter;

        filter.Ignore = movingCollider;

        // We want solid CCD targets here.
        filter.IncludeTriggers = false;

        filter.LayerMask = movingCollider->GetMask();

        QueryBounds(sweptBounds, filter, candidates);

        // -----------------------------------------
        // Find earliest actual shape hit.
        // -----------------------------------------

        bool foundHit = false;

        float earliestTime = 1.0f;

        Collider2D* earliestCollider = nullptr;

        SweepHit2D earliestHit;

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

            Entity* otherOwner = other->GetOwner();

            if (!otherOwner)
            {
                continue;
            }

            Rigidbody2D* otherBody = otherOwner->GetComponent<Rigidbody2D>();

            // -------------------------------------
            // Current CCD scope:
            // only Static or collider-only targets.
            // -------------------------------------
            if (otherBody && !otherBody->IsStatic())
            {
                continue;
            }

            if (movingCollider->IsTrigger() || other->IsTrigger())
            {
                continue;
            }

            const SweepHit2D hit = SweepColliderAgainstCollider(*movingCollider, startBounds, motion, *other);

            if (!hit.Hit)
            {
                continue;
            }

            // Prevent near-zero repeated hits.
            if (hit.Time < m_CCDTimeEpsilon)
            {
                continue;
            }

            if (hit.Time < earliestTime)
            {
                earliestTime = hit.Time;

                earliestHit = hit;

                earliestCollider = other;

                foundHit = true;
            }
        }

        if (!foundHit)
        {
            return false;
        }

        outHit = earliestHit;

        outOtherCollider = earliestCollider;

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

        float remainingFraction = 1.0f;

        // Go back to the actual start.
        transform->SetWorldPosition(currentPosition);

        Bounds2D startBounds = movingCollider->GetWorldBounds();

        for (std::size_t impactIndex = 0; impactIndex < m_MaxCCDImpacts; ++impactIndex)
        {
            if (remainingMotion.LengthSqured() <= 0.000001f)
            {
                break;
            }

            // Swept riggers will be handled here.
            PublishSweptTriggers(movingCollider, startBounds, remainingMotion);

            SweepHit2D hit;

            Collider2D* other = nullptr;

            if (!FindEarliestContinuousHit(movingCollider, startBounds, remainingMotion, hit, other))
            {
                currentPosition += remainingMotion;

                transform->SetWorldPosition(currentPosition);

                break;
            }

            // Move to impact.

            currentPosition += remainingMotion * hit.Time;

            currentPosition += hit.Normal * m_CCDSeparation;

            transform->SetWorldPosition(currentPosition);

            // Target body.

            Entity* otherOwner = other ? other->GetOwner() : nullptr;

            Rigidbody2D* otherBody = otherOwner ? otherOwner->GetComponent<Rigidbody2D>() : nullptr;

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

            // Existing material solver.

            ApplyVelocityResponse(manifold, body, otherBody);

            // Remaining portion of the fixed step.

            remainingFraction *= (1.0f - hit.Time);

            if (remainingFraction <= m_CCDTimeEpsilon)
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

        const float normalLengthSquared = normal.LengthSqured();

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

        if (motion.LengthSqured() <= 0.000001f)
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

        if (startDelta.LengthSqured() <= radius * radius)
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
            const float targetX = boxBounds.Max.X - radius;

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
            const float targetY = boxBounds.Max.Y - radius;

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
        // Box vs Box
        // =========================================================

        if (movingShape == ColliderShape2D::Box && targetShape == ColliderShape2D::Box)
        {
            const SweptAABBHit2D boxHit = SweptAABB(movingStartBounds, motion, target.GetWorldBounds());

            SweepHit2D result;

            result.Hit = boxHit.Hit;

            result.Time = boxHit.Time;

            result.Normal = boxHit.Normal;

            return result;
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
        // Circle vs Box
        // =========================================================

        if (movingShape == ColliderShape2D::Circle && targetShape == ColliderShape2D::Box)
        {
            auto& movingCircle = static_cast<CircleCollider2D&>(moving);

            return
                SweepCircleVsBox(
                    movingCircle.GetWorldCenter(),
                    movingCircle.GetWorldRadius(),
                    motion,
                    target.GetWorldBounds()
                );
        }

        // =========================================================
        // Box vs Circle
        // =========================================================

        if (movingShape == ColliderShape2D::Box && targetShape == ColliderShape2D::Circle)
        {
            auto& targetCircle = static_cast<CircleCollider2D&>(target);

            SweepHit2D result = 
                SweepCircleVsBox(
                    targetCircle.GetWorldCenter(),
                    targetCircle.GetWorldRadius(),
                    motion * -1.0f,
                    movingStartBounds
                );

            if (result.Hit)
            {
                result.Normal *= -1.0f;
            }

            return result;
        }

        return SweepHit2D{};
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

        const float directionLengthSquared = rayDirection.LengthSqured();

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

        const float directionLengthSquared = rayDirection.LengthSqured();

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

        const float lengthSquared = castDirection.LengthSqured();

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

        const float lengthSquared = castDirection.LengthSqured();

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

        const float lengthSquared = normal.LengthSqured();

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

        return delta.LengthSqured() <= radius * radius;
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
        }

        return false;
    }

    bool PhysicsWorld2D::CirclesOverlap(const Vector2& centerA, float radiusA, const Vector2& centerB, float radiusB) const
    {
        const Vector2 delta = centerB - centerA;

        const float combinedRadius = radiusA + radiusB;

        return delta.LengthSqured() <= combinedRadius * combinedRadius;
    }

    bool PhysicsWorld2D::CircleOverlapsAABB(const Vector2& center, float radius, const Bounds2D& bounds) const
    {
        const Vector2 closest{std::clamp(center.X, bounds.Min.X, bounds.Max.X), std::clamp(center.Y, bounds.Min.Y, bounds.Max.Y)};

        const Vector2 delta = center - closest;

        return delta.LengthSqured() <= radius * radius;
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
}