#include "Rigidbody2D.h"

#include "BoxCollider2D.h"
#include "OrientedBox2D.h"
#include "CapsuleCollider2D.h"
#include "CircleCollider2D.h"
#include "PolygonCollider2D.h"

#include "../Scene/Entity.h"
#include "../Scene/TransformComponent.h"

#include <algorithm>
#include <cstdlib>
#include <cmath>

namespace Engine
{
    void Rigidbody2D::SetBodyType(BodyType2D type)
    {
        m_BodyType = type;

        m_IsSleeping = false;

        m_SleepTimer = 0.0f;

        RecalculateInverseMass();
    }

    BodyType2D Rigidbody2D::GetBodyType() const
    {
        return m_BodyType;
    }

    void Rigidbody2D::SetVelocity(const Vector2& velocity)
    {
        if (IsDynamic())
        {
            Wake();
        }

        m_Velocity = velocity;
    }

    const Vector2& Rigidbody2D::GetVelocity() const
    {
        return m_Velocity;
    }

    void Rigidbody2D::AddVelocity(const Vector2& deltaVelocity)
    {
        if (!IsDynamic())
        {
            return;
        }

        Wake();

        m_Velocity += deltaVelocity;
    }

    void Rigidbody2D::SetMass(float mass)
    {
        m_Mass = std::max(0.0001f, mass);

        RecalculateInverseMass();
    }

    float Rigidbody2D::GetMass() const
    {
        return m_Mass;
    }

    float Rigidbody2D::GetInverseMass() const
    {
        return m_InverseMass;
    }

    void Rigidbody2D::SetGravityScale(float gravityScale)
    {
        if (IsDynamic() && gravityScale != m_GravityScale)
        {
            Wake();
        }
        
        m_GravityScale = gravityScale;
    }

    float Rigidbody2D::GetGravityScale() const
    {
        return m_GravityScale;
    }

    void Rigidbody2D::SetLinearDamping(float damping)
    {
        m_LinearDamping = std::max(0.0f, damping);
    }

    float Rigidbody2D::GetLinearDamping() const
    {
        return m_LinearDamping;
    }

    void Rigidbody2D::AddForce(const Vector2& force)
    {
        if (!IsDynamic())
        {
            return;
        }

        if (force.X == 0.0f && force.Y == 0.0f)
        {
            return;
        }

        Wake();

        m_AccumulatedForce += force;
    }

    void Rigidbody2D::AddImpulse(const Vector2& impulse)
    {
        if (!IsDynamic())
        {
            return;
        }

        if (impulse.X == 0.0f && impulse.Y == 0.0f)
        {
            return;
        }

        Wake();

        m_Velocity += impulse * m_InverseMass;
    }

    void Rigidbody2D::AddForceAtPosition(const Vector2& force, const Vector2& worldPoint)
    {
        if (!IsDynamic())
        {
            return;
        }

        if (force.X == 0.0f && force.Y == 0.0f)
        {
            return;
        }

        // Linear part acts through the center of mass exactly like AddForce.

        AddForce(force);

        // Angular part: torque = cross(r, force), where r is the lever arm
        // from the center of mass to the application point. AddTorque already
        // no-ops when the body has fixed rotation, so no extra guard is needed.

        Entity* owner = GetOwner();

        if (!owner)
        {
            return;
        }

        TransformComponent* transform = owner->GetComponent<TransformComponent>();

        if (!transform)
        {
            return;
        }

        const Vector2 center = transform->GetWorldTransform().Position;

        const Vector2 leverArm = worldPoint - center;

        const float torque = leverArm.X * force.Y - leverArm.Y * force.X;

        AddTorque(torque);
    }

    void Rigidbody2D::AddImpulseAtPosition(const Vector2& impulse, const Vector2& worldPoint)
    {
        if (!IsDynamic())
        {
            return;
        }

        if (impulse.X == 0.0f && impulse.Y == 0.0f)
        {
            return;
        }

        Wake();

        // Linear response.

        m_Velocity += impulse * m_InverseMass;

        // Angular response: delta angular velocity = inverseInertia * cross(r, impulse).
        // GetInverseInertia() already returns 0 for fixed-rotation bodies, so
        // this term vanishes automatically when rotation is locked.

        Entity* owner = GetOwner();

        if (!owner)
        {
            return;
        }

        TransformComponent* transform = owner->GetComponent<TransformComponent>();

        if (!transform)
        {
            return;
        }

        const Vector2 center = transform->GetWorldTransform().Position;

        const Vector2 leverArm = worldPoint - center;

        const float angularImpulse = leverArm.X * impulse.Y - leverArm.Y * impulse.X;

        m_AngularVelocity += angularImpulse * GetInverseInertia();
    }

    void Rigidbody2D::ClearForces()
    {
        m_AccumulatedForce = {0.0f, 0.0f};
    }

    const Vector2& Rigidbody2D::GetAccumulatedForce() const
    {
        return m_AccumulatedForce;
    }

    bool Rigidbody2D::IsStatic() const
    {
        return m_BodyType == BodyType2D::Static;
    }

    bool Rigidbody2D::IsKinematic() const
    {
        return m_BodyType == BodyType2D::Kinematic;
    }

    bool Rigidbody2D::IsDynamic() const
    {
        return m_BodyType == BodyType2D::Dynamic;
    }

    void Rigidbody2D::RecalculateInverseMass()
    {
        if (!IsDynamic())
        {
            m_InverseMass = 0.0f;

            return;
        }

        m_InverseMass = 1.0f / m_Mass;
    }

    bool Rigidbody2D::IsSleeping() const
    {
        return m_IsSleeping;
    }

    bool Rigidbody2D::CanSleep() const
    {
        return IsDynamic() && m_AllowSleep;
    }

    void Rigidbody2D::SetAllowSleep(bool allowSleep)
    {
        m_AllowSleep = allowSleep;

        if (!m_AllowSleep)
        {
            Wake();
        }
    }

    float Rigidbody2D::GetSleepTimer() const
    {
        return m_SleepTimer;
    }

    void Rigidbody2D::Sleep()
    {
        if (!CanSleep())
        {
            return;
        }

        m_IsSleeping = true;

        m_SleepTimer = 0.0f;

        m_Velocity = {0.0f, 0.0f};

        m_AccumulatedForce = {0.0f, 0.0f};

        m_AngularVelocity = 0.0f;

        m_AccumulatedTorque = 0.0f;
    }

    void Rigidbody2D::Wake()
    {
        m_IsSleeping = false;

        m_SleepTimer = 0.0f;
    }

    void Rigidbody2D::AddSleepTime(float deltaTime)
    {
        m_SleepTimer += deltaTime;
    }

    void Rigidbody2D::ResetSleepTimer()
    {
        m_SleepTimer = 0.0f;
    }

    void Rigidbody2D::SetVelocityFromPhysics(const Vector2& velocity)
    {
        m_Velocity = velocity;
    }

    void Rigidbody2D::SetCollisionDetectionMode(CollisionDetectionMode2D mode)
    {
        m_CollisionDetectionMode = mode;

        if (IsDynamic())
        {
            Wake();
        }
    }

    CollisionDetectionMode2D Rigidbody2D::GetCollisionDetectionMode() const
    {
        return m_CollisionDetectionMode;
    }

    void Rigidbody2D::SetPreviousPosition(const Vector2& position)
    {
        m_PreviousPosition = position;
    }

    const Vector2& Rigidbody2D::GetPreviousPosition() const
    {
        return m_PreviousPosition;
    }

    void Rigidbody2D::SetFixedRotation(bool fixedRotation)
    {
        if (m_FixedRotation == fixedRotation)
        {
            return;
        }

        m_FixedRotation = fixedRotation;

        // Locking rotation must remove any spin the body currently carries,
        // otherwise it would coast forever (there is no longer any inertia
        // term for damping or contacts to act on).

        if (m_FixedRotation)
        {
            m_AngularVelocity = 0.0f;

            m_AccumulatedTorque = 0.0f;
        }

        if (IsDynamic())
        {
            Wake();
        }
    }

    bool Rigidbody2D::IsFixedRotation() const
    {
        return m_FixedRotation;
    }

    void Rigidbody2D::SetAngularVelocity(float angularVelocity)
    {
        // A fixed-rotation body is not allowed to spin, regardless of who
        // requests it. Force the stored value to zero and bail out.

        if (m_FixedRotation)
        {
            m_AngularVelocity = 0.0f;

            return;
        }

        if (!IsDynamic())
        {
            m_AngularVelocity = angularVelocity;

            return;
        }

        if (angularVelocity == m_AngularVelocity)
        {
            return;
        }

        Wake();

        m_AngularVelocity = angularVelocity;
    }

    float Rigidbody2D::GetAngularVelocity() const
    {
        return m_AngularVelocity;
    }

    void Rigidbody2D::AddAngularVelocity(float deltaAngularVelocity)
    {
        if (!IsDynamic() || m_FixedRotation)
        {
            return;
        }

        if (std::abs(deltaAngularVelocity) <= 0.000001f)
        {
            return;
        }

        Wake();

        m_AngularVelocity += deltaAngularVelocity;
    }

    void Rigidbody2D::SetAngularDamping(float damping)
    {
        m_AngularDamping = std::max(0.0f, damping);
    }

    float Rigidbody2D::GetAngularDamping() const
    {
        return m_AngularDamping;
    }

    void Rigidbody2D::AddTorque(float torque)
    {
        // Torque has no effect on a rotation-locked body.

        if (!IsDynamic() || m_FixedRotation)
        {
            return;
        }

        if (std::abs(torque) <= 0.000001f)
        {
            return;
        }

        Wake();

        m_AccumulatedTorque += torque;
    }

    float Rigidbody2D::GetAccumulatedTorque() const
    {
        return m_AccumulatedTorque;
    }

    void Rigidbody2D::ClearTorque()
    {
        m_AccumulatedTorque = 0.0f;
    }

    void Rigidbody2D::SetAngularVelocityFromPhysics(float angularVelocity)
    {
        m_AngularVelocity = angularVelocity;
    }

    float Rigidbody2D::GetMomentOfInertia() const
    {
        const float mass = GetMass();

        if (mass <= 0.0f)
        {
            return 0.0f;
        }

        Entity* owner = GetOwner();

        if (!owner)
        {
            return 0.0f;
        }

        if (BoxCollider2D* box = owner->GetComponent<BoxCollider2D>())
        {
            const OrientedBox2D obb = box->GetWorldOrientedBox();

            const float width = obb.HalfExtents.X * 2.0f;

            const float height = obb.HalfExtents.Y * 2.0f;

            float inertia = mass * (width * width + height * height) / 12.0f;

            if (TransformComponent* transform = owner->GetComponent<TransformComponent>())
            {
                const Vector2 bodyCenter = transform->GetWorldTransform().Position;

                const Vector2 offset = obb.Center - bodyCenter;

                inertia += mass * offset.LengthSquared();
            }

            return inertia;
        }

        if (CircleCollider2D* circle = owner->GetComponent<CircleCollider2D>())
        {
            const float radius = circle->GetWorldRadius();

            float inertia = 0.5f * mass * radius * radius;

            if (TransformComponent* transform = owner->GetComponent<TransformComponent>())
            {
                const Vector2 bodyCenter = transform->GetWorldTransform().Position;

                const Vector2 circleCenter = circle->GetWorldCenter();

                const Vector2 offset = circleCenter - bodyCenter;

                inertia += mass * offset.LengthSquared();
            }

            return inertia;
        }

        if (CapsuleCollider2D* capsule = owner->GetComponent<CapsuleCollider2D>())
        {
            if (TransformComponent* transform = owner->GetComponent<TransformComponent>())
            {
                const Transform2D& world = transform->GetWorldTransform();

                const float radiusScale = std::max(std::abs(world.Scale.X), std::abs(world.Scale.Y));

                const float halfHeightScale = std::abs(world.Scale.Y);

                const float worldRadius = capsule->GetRadius() * radiusScale;

                const float worldHalfHeight = capsule->GetHalfHeight() * halfHeightScale;

                return CalculateCapsuleInertia(mass, worldRadius, worldHalfHeight);
            }
            else
            {
                return CalculateCapsuleInertia(mass, capsule->GetRadius(), capsule->GetHalfHeight());
            }   
        }

        if (PolygonCollider2D* polygon = owner->GetComponent<PolygonCollider2D>())
        {
            return CalculatePolygonInertia(mass, *polygon);
        }

        return 0.0f;
    }

    float Rigidbody2D::GetInverseInertia() const
    {
        // Static and kinematic bodies do not respond rotationally to physical impulse.

        if (!IsDynamic())
        {
            return 0.0f;
        }

        // A fixed-rotation body has, by definition, infinite rotational
        // inertia. Returning a zero inverse inertia is what makes every
        // rotational term in the contact solver, the joint solver and the
        // integrator collapse to zero, so the body can never be spun.

        if (m_FixedRotation)
        {
            return 0.0f;
        }

        const float inertia = GetMomentOfInertia();

        constexpr float epsilon = 0.000001f;

        if (inertia <= epsilon)
        {
            return 0.0f;
        }

        return 1.0f / inertia;
    }

    float Rigidbody2D::CalculateCapsuleInertia(float mass, float radius, float halfHeight)
    {
        constexpr float Pi = 3.14159265358979323846f;

        radius = std::max(radius, 0.0f);

        halfHeight = std::max(halfHeight, 0.0f);

        if (mass <= 0.0f || radius <= 0.0f)
        {
            return 0.0f;
        }

        const float rectangleWidth = 2.0f * radius;

        const float rectangleHeight = 2.0f * halfHeight;

        const float rectangleArea = rectangleWidth * rectangleHeight;

        const float circleArea = Pi * radius * radius;

        const float totalArea = rectangleArea + circleArea;

        if (totalArea <= 0.000001f)
        {
            return 0.0f;
        }

        const float rectangleMass = mass * (rectangleArea / totalArea);

        const float semicircleMass = mass * (circleArea * 0.5f / totalArea);

        const float rectangleInertia = rectangleMass * (rectangleWidth * rectangleWidth + rectangleHeight * rectangleHeight) / 12.0f;

        const float centroidOffset = 4.0f * radius / (3.0f * Pi);

        const float distanceFromCenter = halfHeight + centroidOffset;

        const float semicircleCentroidInertia = semicircleMass * radius * radius * (0.5f - 16.0f / (9.0f * Pi * Pi));

        const float semicircleInertia = semicircleCentroidInertia + semicircleMass * distanceFromCenter * distanceFromCenter;

        return rectangleInertia + 2.0f * semicircleInertia;
    }

    float Rigidbody2D::CalculatePolygonInertia(float mass, const PolygonCollider2D& polygon)
    {
        const auto& vertices = polygon.GetVertices();

        if (mass <= 0.0f || vertices.size() < 3)
        {
            return 0.0f;
        }

        const Vector2 offset = polygon.GetOffset();

        float twiceSignedArea = 0.0f;

        float inertiaAccumulator = 0.0f;

        for (std::size_t i = 0; i < vertices.size(); ++i)
        {
            const Vector2& a = vertices[i];

            const Vector2& b = vertices[(i + 1) % vertices.size()] + offset;

            const float cross = a.X * b.Y - b.X * a.Y;

            twiceSignedArea += cross;

            const float aa = Vector2::Dot(a, a);

            const float ab = Vector2::Dot(a, b);

            const float bb = Vector2::Dot(b, b);

            inertiaAccumulator += cross * (aa + ab + bb);
        }

        const float signedArea = twiceSignedArea * 0.5f;

        constexpr float epsilon = 0.000001f;

        if (std::abs(signedArea) <= epsilon)
        {
            return 0.0f;
        }

        const float area = std::abs(signedArea);

        const float density = mass / area;

        const float inertia = density * inertiaAccumulator / 12.0f;

        return std::abs(inertia);
    }
}