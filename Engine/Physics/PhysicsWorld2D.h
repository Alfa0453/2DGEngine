#pragma once

#include "ColliderPair2D.h"
#include "OverlapHit2D.h"
#include "PhysicsDebugStatus2D.h"
#include "PhysicsQueryFilter2D.h"
#include "RaycastHit2D.h"
#include "Rigidbody2D.h"
#include "CollisionManifold2D.h"
#include "ShapeCastHit2D.h"
#include "SpatialCell2D.h"
#include "SweepHit2D.h"
#include "SweptAABBHit2D.h"
#include "PhysicsQueryContext2D.h"
#include "BroadPhaseProxy2D.h"
#include "CachedContactPair2D.h"
#include "PhysicsIsland2D.h"
#include "Joint2D.h"
#include "Polygon2D.h"
#include "PhysicsSettings2D.h"
#include "PhysicsStats2D.h"
#include "PhysicsDebugDrawSettings2D.h"
#include "PhysicsDebugSnapshot2D.h"
#include "PhysicsCCDDebug2D.h"

#include "../Math/Vector2.h"
#include "../Math/Bounds2D.h"
#include "../Scene/TransformComponent.h"

#include <unordered_set>
#include <unordered_map>
#include <vector>

using SpatialBucket2D = std::vector<Engine::Collider2D*>;

namespace Engine
{
    class Scene;

    class Collider2D;
    class BoxCollider2D;
    class CircleCollider2D;
    class CapsuleCollider2D;
    class PolygonCollider2D;

    struct OrientedBox2D;

    struct Capsule2D;

    class PhysicsWorld2D
    {
    public:
        
        explicit PhysicsWorld2D(Scene* scene = nullptr);

        void Update(float deltaTime);

        void SetScene(Scene* scene);

        Scene* GetScene() const;

        void Step(float deltaTime);

        std::size_t GetActiveColliderCount() const;

        std::size_t GetCurrentOverlapCount() const;

        void SetGravity(const Vector2& gravity);

        const Vector2& GetGravity() const;

        void IntegrateBodies(float deltaTime);

        void IntegrateEntityRecursive(Entity* entity, float deltaTime);

        void IntegrateDynamicBody(Rigidbody2D& body, TransformComponent& transform, float deltaTime);

        void IntegrateKinematicBody(Rigidbody2D& body, TransformComponent& transform, float deltaTime);

        std::size_t GetSpatialCellCount() const;

        std::size_t GetCandidatePairCount() const;

        const std::vector<Collider2D*>& GetActiveColliders() const;

        const std::vector<CollisionManifold2D>& GetCurrentContacts() const;

        const std::unordered_map<SpatialCell2D, SpatialBucket2D, SpatialCell2DHash>& GetSpatialGrid() const;

        void SetSpatialCellSize(float cellSize);

        float GetSpatialCellSize() const;

        void SetVelocityIterations(std::size_t iterations);

        std::size_t GetVelocityIterations() const;

        void SetPositionIterations(std::size_t iterations);

        std::size_t GetPositionIterations() const;

        PhysicsDebugStatus2D GetDebugStatus() const;

        bool Raycast(const Vector2& origin, const Vector2& direction, float maxDistance, RaycastHit2D& outHit, const PhysicsQueryFilter2D& filter = PhysicsQueryFilter2D{}) const;

        std::size_t RaycastAll(const Vector2& origin, const Vector2& direction, float maxDistance, std::vector<RaycastHit2D>& outHits, const PhysicsQueryFilter2D& filter = PhysicsQueryFilter2D{}) const;

        std::size_t OverlapPoint(const Vector2& point, std::vector<OverlapHit2D>& outHits, const PhysicsQueryFilter2D& filer = PhysicsQueryFilter2D{}) const;

        std::size_t OverlapCircle(const Vector2& center, float radius, std::vector<OverlapHit2D>& outHits, const PhysicsQueryFilter2D& filter = PhysicsQueryFilter2D{}) const;

        std::size_t OverlapBox(const Bounds2D& bounds, std::vector<OverlapHit2D>& outHits, const PhysicsQueryFilter2D& filter = PhysicsQueryFilter2D{}) const;

        bool CircleCast(const Vector2& origin, float radius, const Vector2& direction, float maxDistance, ShapeCastHit2D& outHit, const PhysicsQueryFilter2D& filter = PhysicsQueryFilter2D{}) const;

        bool CircleCast(const Vector2& origin, float radius, const Vector2& direction, float maxDistance, ShapeCastHit2D& outHit, PhysicsQueryContext2D& context, const PhysicsQueryFilter2D& filter = PhysicsQueryFilter2D{}) const;

        bool BoxCast(const Bounds2D& startBounds, const Vector2& direction, float maxDistance, ShapeCastHit2D& outHit, const PhysicsQueryFilter2D& filter = PhysicsQueryFilter2D{}) const;

        bool BoxCast(const Bounds2D& startBounds, const Vector2& direction, float maxDistance, ShapeCastHit2D& outHit, PhysicsQueryContext2D& context, const PhysicsQueryFilter2D& filter = PhysicsQueryFilter2D{}) const;

        std::size_t GetIslandCount() const;

        void AddJoint(Joint2D* joint);

        void RemoveJoint(Joint2D* joint);

        const std::vector<Joint2D*>& GetJoints() const;

        float GetConstraintInverseMass(const Rigidbody2D* body) const;

        float GetConstraintInverseInertia(const Rigidbody2D* body) const;

        float Cross(const Vector2& a, const Vector2& b) const;

        Vector2 GetConstraintPointVelocity(const Vector2& linearVelocity, float angularVelocity, const Vector2& leverArm) const;

        const PhysicsSettings2D& GetSettings() const;

        void SetSettings(const PhysicsSettings2D& settings);

        PhysicsSettings2D& GetSettings();

        const PhysicsStats2D& GetStats() const;

        void PrintPhysicsStats() const;

        float GetBroadPhaseRejectionRatio() const;

        const PhysicsDebugDrawSettings2D& GetDebugDrawSettings() const;

        PhysicsDebugDrawSettings2D& GetDebugDrawSettings();

        PhysicsDebugSnapshot2D GetDebugSnapshot() const;

        const std::vector<PhysicsCCDDebug2D>& GetCCDDebugRecords() const;

    private:

        struct RayShapeHit2D
        {
            bool Hit = false;

            float Distance = 0.0f;

            Vector2 Normal{0.0f, 0.0f};
        };

        struct ClosestSegmentPoints2D
        {
            Vector2 PointA{0.0f, 0.0f};

            Vector2 PointB{0.0f, 0.0f};
        };

        struct PolygonSATResult2D
        {
            bool Overlapping = false;

            Vector2 Axis{0.0f, 0.0f};

            float Overlap = 0.0f;

            float AxisFromA = 0.0f;

            std::size_t AxisIndex = 0;
        };

        struct PolygonEdge2D
        {
            Vector2 A{0.0f, 0.0f};

            Vector2 B{0.0f, 0.0f};

            Vector2 Normal{0.0f, 0.0f};

            std::size_t Index = 0;
        };

        struct SweptAxisResult2D
        {
            bool Valid = true;

            float EntryTime = 0.0f;

            float ExitTime = 1.0f;

            Vector2 Normal{0.0f, 0.0f};
        };

        void ValidateSettings();

        void CollectColliders();

        void CollectCollidersRecursive(Entity* entity);

        void CollectMovingCCDCandidates(Collider2D* movingCollider, const Bounds2D& movingSweptBounds, std::vector<Collider2D*>& inOutCandidates) const;

        bool ShouldTestPair(Collider2D* a, Collider2D* b) const;

        bool BroadPhaseOverlap(const Collider2D& a, const Collider2D& b) const;

        bool GenerateManifold(Collider2D& a, Collider2D& b, CollisionManifold2D& manifold) const;

        // Returns true when a solid manifold involving a one-way (pass-through)
        // collider should be kept. A one-way collider only remains solid when
        // the other body is being pushed out along that collider's one-way
        // axis; otherwise the pair is discarded and the bodies pass through.
        bool ShouldBlockOneWayContact(const CollisionManifold2D& manifold) const;

        bool BoxVsBox(BoxCollider2D& a, BoxCollider2D& b, CollisionManifold2D& manifold) const;

        bool CircleVsCircle(CircleCollider2D& a, CircleCollider2D& b, CollisionManifold2D& manifold) const;

        bool BoxVsCircle(BoxCollider2D& box, CircleCollider2D& circle, CollisionManifold2D& manifold) const;

        bool CapsuleVsCircle(CapsuleCollider2D& capsuleCollider, CircleCollider2D& circle, CollisionManifold2D& manifold) const;

        bool CapsuleVsCapsule(CapsuleCollider2D& a, CapsuleCollider2D& b, CollisionManifold2D& manifold) const;

        bool PolygonVsPolygon(PolygonCollider2D& a, PolygonCollider2D& b, CollisionManifold2D& manifold) const;

        bool PolygonVsBox(PolygonCollider2D& polygonCollider, BoxCollider2D& boxCollider, CollisionManifold2D& manifold) const;

        bool PolygonVsCircle(PolygonCollider2D& polygonCollider, CircleCollider2D& circle, CollisionManifold2D& manifold) const;

        bool CapsuleVsBox(CapsuleCollider2D& capsuleCollider, BoxCollider2D& boxCollider, CollisionManifold2D& manifold) const;

        bool CapsuleVsPolygon(CapsuleCollider2D& capsuleCollider, PolygonCollider2D& polygonCollider, CollisionManifold2D& manifold) const;

        // Shared narrow-phase core for a capsule versus any convex polygon
        // (a box is converted to a polygon first). On overlap it fills the
        // manifold's Normal (pointing from the capsule toward the polygon),
        // Penetration and up to two contact points; the callers set A/B and
        // the trigger flag. Returns false when the shapes are separated.
        bool BuildCapsulePolygonManifold(const Capsule2D& capsule, const Polygon2D& polygon, CollisionManifold2D& manifold) const;

        void ProjectOrientedBox(const OrientedBox2D& box, const Vector2& axis, float& outMin, float& outMax) const;

        bool TestOBBAxis(const OrientedBox2D& a, const OrientedBox2D& b, const Vector2& axis, float& outOverlap) const;

        Vector2 GetOBBSupportPoint(const OrientedBox2D& box, const Vector2& direction) const;

        std::size_t ClipSegmentToSpan(const Vector2& p0, const Vector2& p1, const Vector2& origin, const Vector2& tangent, float halfLength, Vector2 outPoints[2]) const;

        std::size_t BuildOBBContactPoints(const OrientedBox2D& a, const OrientedBox2D& b, const Vector2& normal, int minimumAxisIndex, Vector2 outContacts[2]) const;

        Vector2 WorldPointToOBBLocal(const OrientedBox2D& box, const Vector2& worldPoint) const;

        Vector2 OBBLocalPointToWorld(const OrientedBox2D& box, const Vector2& localPoint) const;

        Vector2 OBBLocalDirectionToWorld(const OrientedBox2D& box, const Vector2& localDirection) const;

        void PublishPairEvents();

        void PublishBegin(const ColliderPair2D& pair);

        void PublishStay(const ColliderPair2D& pair);

        void PublishEnd(const ColliderPair2D& pair);

        void ApplyPositionalCorrection(const CollisionManifold2D& manifold, Rigidbody2D* bodyA, Rigidbody2D* bodyB, TransformComponent& transformA, TransformComponent& transformB);

        void ApplyVelocityResponse(CollisionManifold2D& manifold, Rigidbody2D* bodyA, Rigidbody2D* bodyB);
    
        void SolveVelocityContacts();

        void SolvePositionContacts();

        void SolveVelocityContact(CollisionManifold2D& manifold);

        float CombineRestitution(const Collider2D& a, const Collider2D& b) const;

        float CombineStaticFriction(const Collider2D& a, const Collider2D& b) const;

        float CombineDynamicFriction(const Collider2D& a, const Collider2D& b) const;

        bool RefreshManifold(CollisionManifold2D& manifold) const;

        float GetSolverInverseMass(const Rigidbody2D* body) const;

        float GetSolverInverseInertia(const Rigidbody2D* body) const;

        float Cross2D(const Vector2& a, const Vector2& b) const;

        Vector2 AngularCrossVector(float angularVelocity, const Vector2& vector) const;

        Vector2 GetVelocityAtPoint(const Vector2& linearVelocity, float angularVelocity, const Vector2& leverArm) const;

        void WakeBodiesFromContacts();

        // Legacy pre-island sleep path.
        // No longer called by PhysicsWorld2D::Step().
        void UpdateSleepStates(float deltaTime);

        void UpdateEntitySleepRecursive(Entity* entity, float deltaTime);

        SpatialCell2D WorldToCell(const Vector2& worldPosition) const;

        void BuildSpatialGrid();

        void GenerateCandidatePairs();

        void ProcessCandidatePairs();

        SweptAABBHit2D SweptAABB(const Bounds2D& movingStartBounds, const Vector2& relativeMotion, const Bounds2D& targetBounds) const;

        void RunContinuousCollisionPass(float deltaTime);

        bool FindEarliestContinuousHit(Collider2D* movingCollider, const Bounds2D& movingStartBounds, const Vector2& movingMotion, float remainingFraction, SweepHit2D& outHit, Collider2D*& outOtherCollider, Vector2& outOtherMotion);

        Bounds2D ReconstructStartBounds(const Collider2D& collider, const Rigidbody2D& body, const TransformComponent& transform) const;

        Bounds2D BuildSweptBounds(const Bounds2D& startBounds, const Vector2& motion) const;

        void ProcessContinuousBody(Collider2D* movingCollider, Rigidbody2D* body, TransformComponent* transform, float deltaTime);

        void PublishSweptTriggers(Collider2D* movingCollider, const Bounds2D& startBounds, const Vector2& motion);

        SweepHit2D SweepCircleVsCircle(const Vector2& startCenterA, float radiusA, const Vector2& motion, const Vector2& centerB, float radiusB) const;

        SweepHit2D SweepCircleVsBox(const Vector2& startCenter, float radius, const Vector2& motion, const Bounds2D& boxBounds) const;

        SweepHit2D SweepColliderAgainstCollider(Collider2D& moving, const Bounds2D& movingStartBounds, const Vector2& motion, Collider2D& target) const;

        SweepHit2D SweepColliderAgainstMovingCollider(Collider2D& moving, const Bounds2D& movingStartBounds, const Vector2& movingMotion, Collider2D& target, const Bounds2D& targetStartBounds, const Vector2& targetMotion) const;

        SweepHit2D SweepCircleVsOBB(const Vector2& startCenter, float radius, const Vector2& motion, const OrientedBox2D& box) const;

        SweepHit2D SweepOBBVsOBB(const OrientedBox2D& movingStartBox, const Vector2& relativeMotion, const OrientedBox2D& targetStartBox) const;

        OrientedBox2D TranslateOrientedBox(const OrientedBox2D& box, const Vector2& translation) const;

        void ConsiderSweepCandidate(float time, const Vector2& normal, SweepHit2D& bestHit) const;

        void QueryBounds(const Bounds2D& bounds, const PhysicsQueryFilter2D& filter, std::vector<Collider2D*>& outResults) const;

        RayShapeHit2D RaycastAABB(const Vector2& origin, const Vector2& direction, float maxDistance, const Bounds2D& bounds) const;

        RayShapeHit2D RaycastCircle(const Vector2& origin, const Vector2& direction, float maxDistance, const Vector2& center, float radius) const;

        RayShapeHit2D RaycastCollider(const Vector2& origin, const Vector2& direction, float maxDistance, const Collider2D& collider) const;

        bool PointInsideAABB(const Vector2& point, const Bounds2D& bounds) const;

        bool PointInsideCircle(const Vector2& point, const Vector2& center, float radius) const;

        bool PointOverlapsCollider(const Vector2& point, const Collider2D& collider) const;

        bool CirclesOverlap(const Vector2& centerA, float radiusA, const Vector2& centerB, float radiusB) const;

        bool CircleOverlapsAABB(const Vector2& center, float radius, const Bounds2D& bounds) const;

        bool CircleOverlapsCollider(const Vector2& center, float radius, const Collider2D& collider) const;

        bool AABBsOverlap(const Bounds2D& a, const Bounds2D& b) const;

        bool BoxOverlapsCollider(const Bounds2D& queryBox, const Collider2D& collider) const;

        const std::vector<Collider2D*>& QueryBoundsToContext(const Bounds2D& bounds, const PhysicsQueryFilter2D& filter, PhysicsQueryContext2D& context) const;

        SweepHit2D SweepCircleQueryAgainstCollider(const Vector2& startCenter, float radius, const Vector2& motion, const Collider2D& target) const;

        SweepHit2D SweepBoxQueryAgainstCollider(const Bounds2D& startBounds, const Vector2& motion, const Collider2D& target) const;

        void GetCellsForBounds(const Bounds2D& bounds, std::vector<SpatialCell2D>& outCells) const;

        void InsertProxyIntoGrid(BroadPhaseProxy2D& proxy) const;

        void RemoveProxyFromGrid(const BroadPhaseProxy2D& proxy) const;

        BroadPhaseProxy2D BuildProxy(Collider2D* collider) const;

        bool IsProxyDirty(const BroadPhaseProxy2D& proxy) const;

        void UpdateBroadPhaseProxy(BroadPhaseProxy2D& proxy) const;

        void SynchronizeBroadPhaseForQueries() const;

        void PrepareVelocityContacts();

        void PrepareVelocityContact(CollisionManifold2D& manifold);

        void WarmStartVelocityContacts();

        void WarmStartVelocityContact(CollisionManifold2D& manifold);

        void RestoreCachedContactImpulses();

        void RestoreCachedContactImpulses(CollisionManifold2D& manifold);

        void StoreContactCache();

        void StoreContactCache(const CollisionManifold2D& manifold);

        void RemoveStaleCachedContacts();

        int FindClosestCachedContact(const CachedContactPair2D& cachedPair, const Vector2& point, bool used[2]) const;

        void BuildIslands();

        bool IsIslandDynamicBody(const Rigidbody2D* body) const;

        void PrepareVelocityIsland(PhysicsIsland2D& island);

        void WarmStartVelocityIsland(PhysicsIsland2D& island);

        void SolveVelocityIsland(PhysicsIsland2D& island);

        void SolvePositionIsland(PhysicsIsland2D& island);

        Rigidbody2D* GetColliderBody(Collider2D* collider) const;

        void UpdateIslandSleepStates(float deltaTime);

        void UpdateIslandSleepState(PhysicsIsland2D& island, float deltaTime);

        void WakeIsland(PhysicsIsland2D& island);

        void SleepIsland(PhysicsIsland2D& island);

        bool IsBodyQuietForSleep(const Rigidbody2D& body) const;

        void UpdateIsolatedBodySleepRecursive(Entity* entity, float deltaTime, const std::unordered_set<Rigidbody2D*>& islandBodies);

        bool IslandNeedsWake(const PhysicsIsland2D& island) const;

        void PropagateIslandWakeStates();

        void PrepareJoints(PhysicsIsland2D& island, float deltaTime);

        void WarmStartJoints(PhysicsIsland2D& island);

        void SolveJointVelocities(PhysicsIsland2D& island);

        ClosestSegmentPoints2D ClosestPointsBetweenSegments(const Vector2& a0, const Vector2& a1, const Vector2& b0, const Vector2& b1) const;

        Vector2 GetPolygonSupportPoint(const Polygon2D& polygon, const Vector2& direction) const;

        void ProjectPolygon(const Polygon2D& polygon, const Vector2& axis, float& outMin, float& outMax) const;

        Vector2 GetPolygonCenter(const Polygon2D& polygon) const;

        bool TestPolygonAxis(const Polygon2D& a, const Polygon2D& b, const Vector2& axis, float& outOverlap) const;

        PolygonSATResult2D TestPolygonSAT(const Polygon2D& a, const Polygon2D& b) const;

        PolygonEdge2D GetPolygonEdge(const Polygon2D& polygon, std::size_t edgeIndex) const;

        PolygonEdge2D FindIncidentPolygonEdge(const Polygon2D& polygon, const Vector2& referenceNormal) const;

        std::size_t BuildPolygonContactPoints(const Polygon2D& polygonA, const Polygon2D& polygonB, const PolygonSATResult2D& sat, Vector2 outContacts[2]) const;

        Polygon2D OrientedBoxToPolygon(const OrientedBox2D& box) const;

        Vector2 FindClosestPolygonVertex(const Polygon2D& polygon, const Vector2& point) const;

        void ProjectCircle(const Vector2& center, float radius, const Vector2& axis, float& outMin, float& outMax) const;

        Vector2 GetBodyStepMotion(const Rigidbody2D* body, const TransformComponent* transform) const;

        SweptAxisResult2D SweepIntervalsOnAxis(float minA, float maxA, float minB, float maxB, float relativeSpeed, const Vector2& axis) const;

        void ResetStepStats(float deltaTime);

        void FinalizeStepStats();

        void CountBodiesForStats();

    private:

        PhysicsSettings2D m_Settings;

        PhysicsDebugDrawSettings2D m_DebugDrawSettings;

        PhysicsStats2D m_Stats;
        
        Scene* m_Scene = nullptr;

        float m_Accumulator = 0.0f;

        std::vector<Collider2D*> m_ActiveColliders;

        std::vector<CollisionManifold2D> m_CurrentContacts;

        std::unordered_set<ColliderPair2D, ColliderPair2DHash> m_PreviousOverlaps;

        std::unordered_set<ColliderPair2D, ColliderPair2DHash> m_CurrentOverlaps;

        mutable std::unordered_map<SpatialCell2D, SpatialBucket2D, SpatialCell2DHash> m_SpatialGrid;

        mutable std::unordered_map<Collider2D*, BroadPhaseProxy2D> m_BroadPhaseProxies;

        std::unordered_set<ColliderPair2D, ColliderPair2DHash> m_CandidatePairs;

        std::unordered_set<ColliderPair2D, ColliderPair2DHash> m_SweptTriggerPairsThisStep;

        std::unordered_map<ColliderPair2D, CachedContactPair2D, ColliderPair2DHash> m_ContactCache;

        std::unordered_set<ColliderPair2D, ColliderPair2DHash> m_CCDResolvePairsThisStep;

        std::vector<PhysicsIsland2D> m_Islands;

        std::vector<Joint2D*> m_Joints;

        std::vector<PhysicsCCDDebug2D> m_CCDDebugRecords;
    };
}