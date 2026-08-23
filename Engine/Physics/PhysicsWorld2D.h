#pragma once

#include "ColliderPair2D.h"
#include "../Math/Vector2.h"
#include "../Scene/TransformComponent.h"
#include "Rigidbody2D.h"

#include <unordered_set>
#include <vector>

namespace Engine
{
    class Scene;

    class Collider2D;
    class BoxCollider2D;

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

    private:

        void CollectColliders();

        void CollectCollidersRecursive(Entity* entity);

        bool ShouldTestPair(Collider2D* a, Collider2D* b) const;

        bool BroadPhaseOverlap(const Collider2D& a, const Collider2D& b) const;

        bool NarrowPhaseOverlap(const Collider2D& a, const Collider2D& b) const;

        bool BoxVsBox(const BoxCollider2D& a, const BoxCollider2D& b) const;

        void PublishPairEvents();

        void PublishBegin(const ColliderPair2D& pair);

        void PublishStay(const ColliderPair2D& pair);

        void PublishEnd(const ColliderPair2D& pair);

    private:
        
        Scene* m_Scene = nullptr;

        Vector2 m_Gravity{0.0f, 980.0f};

        float m_FixedDeltaTime = 1.0f / 60.0f;

        float m_Accumulator = 0.0f;

        std::size_t m_MaxSubSteps = 8;

        std::vector<Collider2D*> m_ActiveColliders;

        std::unordered_set<ColliderPair2D, ColliderPair2DHash> m_PreviousOverlaps;

        std::unordered_set<ColliderPair2D, ColliderPair2DHash> m_CurrentOverlaps;
    };
}