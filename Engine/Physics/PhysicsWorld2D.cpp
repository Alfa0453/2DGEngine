#include "PhysicsWorld2D.h"

#include "../Scene/Scene.h"
#include "../Scene/Entity.h"
#include "BoxCollider2D.h"
#include "../Math/Bounds2D.h"
#include "CollisionEvents2D.h"

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

        // -------------------------------
        // Move physics bodies
        // -------------------------------

        IntegrateBodies(deltaTime);

        // ---------------------------------
        // Collect current colliders
        // ---------------------------------

        CollectColliders();

        // ---------------------------------
        // Start fresh overlap set
        // ---------------------------------

        m_CurrentOverlaps.clear();

        // ---------------------------------
        // Generate pairs
        // ---------------------------------

        for (std::size_t i = 0; i < m_ActiveColliders.size(); ++i)
        {
            for (std::size_t j = i + 1; j < m_ActiveColliders.size(); ++j)
            {
                Collider2D* a = m_ActiveColliders[i];

                Collider2D* b = m_ActiveColliders[j];

                // -------------------------
                // Filtering
                // -------------------------

                if (!ShouldTestPair(a, b))
                {
                    continue;
                }

                // -------------------------
                // Broad phase
                // -------------------------

                if (!BroadPhaseOverlap(*a, *b))
                {
                    continue;
                }

                // -------------------------
                // Narrow phase
                // -------------------------

                if (!NarrowPhaseOverlap(*a, *b))
                {
                    continue;
                }

                // -------------------------
                // Real overlap
                // -------------------------

                m_CurrentOverlaps.insert(ColliderPair2D::Make(a, b));
            }
        }

        // ---------------------------------
        // Generate Begin/Stay/End
        // ---------------------------------

        PublishPairEvents();

        // ---------------------------------
        // Current becomes previous
        // ---------------------------------

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

        BoxCollider2D* box = entity->GetComponent<BoxCollider2D>();

        if (box && box->IsEnabled())
        {
            const Vector2& size = box->GetSize();

            if (size.X > 0.0f && size.Y > 0.0f)
            {
                m_ActiveColliders.push_back(box);
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
                    IntegrateDynamicBody(*body, *transform, deltaTime);
                    break;
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

    bool PhysicsWorld2D::BoxVsBox(const BoxCollider2D& a, const BoxCollider2D& b) const
    {
        const Bounds2D boundsA = a.GetWorldBounds();

        const Bounds2D boundsB = b.GetWorldBounds();

        return boundsA.Intersects(boundsB);
    }

    bool PhysicsWorld2D::NarrowPhaseOverlap(const Collider2D& a, const Collider2D& b) const
    {
        if (a.GetShape() == ColliderShape2D::Box && b.GetShape() == ColliderShape2D::Box)
        {
            return BoxVsBox(static_cast<const BoxCollider2D&>(a), static_cast<const BoxCollider2D&>(b));
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

        body.SetVelocity(velocity);

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
}