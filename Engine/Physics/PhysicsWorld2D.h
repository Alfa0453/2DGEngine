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

    struct OrientedBox2D;

    class PhysicsWorld2D
    {
    public:
        
        explicit PhysicsWorld2D(Scene* scene = nullptr);

        void Update(float deltaTime);

        void SetScene(Scene* scene);

        Scene* GetScene() const;

        void Step(float deltaTime);

        std::size_t GetAciveColliderCount() const;

        std::size_t GetCurrentOverlapCount() const;

        void SetGravity(const Vector2& gravity);

        const Vector2& GetGravity() const;

        void IntegrateBodies(float deltaTime);

        void IntegrateEntityRecursive(Entity* entity, float deltaTime);

        void IntegrateDynamicBody(Rigidbody2D& body, TransformComponent& transform, float deltaTime);

        void IntegrateKinematicBody(Rigidbody2D& body, TransformComponent& transform, float deltaTime);

        std::size_t GetSpatialCellCount() const;

        std::size_t GetCandidatePirCount() const;

        const std::vector<Collider2D*>& GetactiveColliders() const;

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

    private:

        struct RayShapeHit2D
        {
            bool Hit = false;

            float Distance = 0.0f;

            Vector2 Normal{0.0f, 0.0f};
        };

        void CollectColliders();

        void CollectCollidersRecursive(Entity* entity);

        bool ShouldTestPair(Collider2D* a, Collider2D* b) const;

        bool BroadPhaseOverlap(const Collider2D& a, const Collider2D& b) const;

        bool GenerateManifold(Collider2D& a, Collider2D& b, CollisionManifold2D& manifold) const;

        bool BoxVsBox(BoxCollider2D& a, BoxCollider2D& b, CollisionManifold2D& manifold) const;

        bool CircleVsCircle(CircleCollider2D& a, CircleCollider2D& b, CollisionManifold2D& manifold) const;

        bool BoxVsCircle(BoxCollider2D& box, CircleCollider2D& circle, CollisionManifold2D& maniflod) const;

        void ProjectOrientedBox(const OrientedBox2D& box, const Vector2& axis, float& outMin, float& outMax) const;

        bool TestOBBAxis(const OrientedBox2D& a, const OrientedBox2D& b, const Vector2& axis, float& outOverlap) const;

        Vector2 GetOBBSurpportPoint(const OrientedBox2D& box, const Vector2& direction) const;

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

        void ApplyVelocityResponse(const CollisionManifold2D& manifold, Rigidbody2D* bodyA, Rigidbody2D* bodyB);
    
        void SolveVelocityContacts();

        void SolvePositionContacts();

        void SolveVelocityContact(const CollisionManifold2D& manifold);

        float CombineRestitution(const Collider2D& a, const Collider2D& b) const;

        float CombineStaticFriction(const Collider2D& a, const Collider2D& b) const;

        float CombineDynamicFriction(const Collider2D& a, const Collider2D& b) const;

        bool RefreshManifold(CollisionManifold2D& manifold) const;

        float GetSolverInverseMass(const Rigidbody2D* body) const;

        void WakeBodiesFromContacts();

        void UpdateSleepStates(float deltaTime);

        void UpdateEntitySleepRecursive(Entity* entity, float deltaTime);

        SpatialCell2D WorldToCell(const Vector2& worldPosition) const;

        void BuildSpatialGrid();

        void GenerateCondidatePairs();

        void ProcessCandidatePairs();

        SweptAABBHit2D SweptAABB(const Bounds2D& movingStartBounds, const Vector2& relativeMotion, const Bounds2D& targetBounds) const;

        void RunContinuousCollisionPass(float deltaTime);

        bool FindEarliestContinuousHit(Collider2D* movingCollider, const Bounds2D& startBounds, const Vector2& motion, SweepHit2D& outHit, Collider2D*& outOtherCollider);

        Bounds2D ReconstructStartBounds(const Collider2D& collider, const Rigidbody2D& body, const TransformComponent& transform) const;

        Bounds2D BuildSweptBounds(const Bounds2D& startBounds, const Vector2& motion) const;

        void ProcessContinuousBody(Collider2D* movingCollider, Rigidbody2D* body, TransformComponent* transform, float deltaTime);

        void PublishSweptTriggers(Collider2D* movingCollider, const Bounds2D& startBoKsunds, const Vector2& motion);

        SweepHit2D SweepCircleVsCircle(const Vector2& startCenterA, float radiusA, const Vector2& motion, const Vector2& centerB, float radiusB) const;

        SweepHit2D SweepCircleVsBox(const Vector2& startCenter, float radius, const Vector2& motion, const Bounds2D& boxBounds) const;

        SweepHit2D SweepColliderAgainstCollider(Collider2D& moving, const Bounds2D& movingStartBounds, const Vector2& motion, Collider2D& target) const;

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

    private:
        
        Scene* m_Scene = nullptr;

        Vector2 m_Gravity{0.0f, 980.0f};

        float m_FixedDeltaTime = 1.0f / 60.0f;

        float m_Accumulator = 0.0f;

        std::size_t m_MaxSubSteps = 8;

        std::vector<Collider2D*> m_ActiveColliders;

        std::vector<CollisionManifold2D> m_CurrentContacts;

        std::unordered_set<ColliderPair2D, ColliderPair2DHash> m_PreviousOverlaps;

        std::unordered_set<ColliderPair2D, ColliderPair2DHash> m_CurrentOverlaps;

        std::size_t m_VelocityIterations = 8;

        std::size_t m_PositionIterations = 3;

        float m_CollisionWakeSpeed = 20.0f;

        float m_SleepLinearSpeedThreshold = 5.0f;

        float m_TimeToSleep = 0.5f;

        mutable std::unordered_map<SpatialCell2D, SpatialBucket2D, SpatialCell2DHash> m_SpatialGrid;

        mutable std::unordered_map<Collider2D*, BroadPhaseProxy2D> m_BroadPhaseProxies;

        float m_SpatialCellSize = 128.0f;

        std::unordered_set<ColliderPair2D, ColliderPair2DHash> m_CandidatePairs;

        std::size_t m_MaxCCDImpacts = 4;

        float m_CCDTimeEpsilon = 0.0001f;

        float m_CCDSeparation = 0.001f;

        std::unordered_set<ColliderPair2D, ColliderPair2DHash> m_SweptTriggerPairsThisStep;
    };
}