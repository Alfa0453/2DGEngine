# Physics 2D — New Features Setup Guide

This document covers four capabilities added to the physics engine:

1. Rotation lock (fixed rotation) on `Rigidbody2D`
2. Off-center force / impulse application on `Rigidbody2D`
3. One-way (pass-through) platforms and conveyor surface velocity on `Collider2D`
4. Motors and limits on `RevoluteJoint2D` and `PrismaticJoint2D`

All angles are in **radians** and all angular velocities in **radians/second** at
the physics API level (transforms store rotation in degrees internally; the joints
convert for you). Gravity in this engine points along **+Y (downward)**, so "up" is
`-Y`. This matters for the one-way platform default axis.

---

## 1. Rotation lock (fixed rotation)

Locks a body's rotational degree of freedom. The body still translates and collides
normally, but nothing — contacts, joints, torque, or direct API calls — can spin it.
This is the standard setup for player characters, top-down actors, and any body that
must stay upright.

```cpp
rigidbody->SetFixedRotation(true);   // lock; clears any existing spin
bool locked = rigidbody->IsFixedRotation();
```

How it works: while locked, `GetInverseInertia()` returns `0`, which zeroes every
rotational term in the contact solver, the joint solver, and the integrator.
`SetAngularVelocity`, `AddAngularVelocity`, and `AddTorque` become no-ops. Calling
`SetFixedRotation(false)` restores normal rotational dynamics (inertia is recomputed
from the collider shape as before).

---

## 2. Off-center force and impulse

`AddForce` / `AddImpulse` act through the center of mass and never rotate the body.
To apply a force or impulse at a specific world point — an explosion, a thruster,
recoil, a directional hit — use the positional variants. They add the correct torque
from the lever arm automatically.

```cpp
// Continuous force at a world point (accumulated for this step):
rigidbody->AddForceAtPosition(Vector2{0.0f, -500.0f}, hitWorldPoint);

// Instantaneous impulse at a world point:
rigidbody->AddImpulseAtPosition(Vector2{250.0f, 0.0f}, contactWorldPoint);
```

The torque contribution is `cross(r, force)` where `r` is the vector from the body's
world center of mass to the point. On a fixed-rotation body the torque part is
suppressed automatically, so only the linear push applies.

---

## 3. One-way platforms and conveyor surfaces

Both live on `Collider2D`, so any collider (box, circle, capsule, polygon) can use them.

### One-way (jump-through) platforms

A one-way collider only produces a **solid** contact when the other body is on the
solid side defined by its one-way axis. From every other direction the contact is
discarded and the body passes through. Triggers still fire from all sides.

```cpp
platformCollider->SetOneWay(true);
platformCollider->SetOneWayAxis(Vector2{0.0f, -1.0f}); // default: solid from above ("up")
platformCollider->SetOneWayThreshold(0.5f);            // cosine tolerance, default 0.5 (~60°)
```

The contact is kept solid only when the collision normal pushing the other body away
from the platform aligns with the one-way axis by more than the threshold. The default
axis `(0, -1)` means "you can rest on top and land from above, but you rise through it
from below and walk through the sides." Lower the threshold toward `0` to make the
solid arc wider; raise it toward `1` to require a near-vertical landing.

This test is velocity-independent, so a body resting motionless on top of a one-way
platform stays supported correctly.

### Conveyor / moving-surface velocity

A non-zero surface velocity makes friction drive touching bodies toward a target
world-space velocity instead of toward rest — a conveyor belt or moving walkway.
Only the component along the contact tangent has an effect.

```cpp
beltCollider->SetSurfaceVelocity(Vector2{120.0f, 0.0f}); // pushes bodies to the right
beltCollider->SetSurfaceVelocity(Vector2{0.0f, 0.0f});   // ordinary surface (default)
```

The drive strength is bounded by the surface's friction: a low-friction belt moves
bodies slowly, a high-friction belt grips them to belt speed quickly.

---

## 4. Joint motors and limits

Motors and limits were added to both `RevoluteJoint2D` (angular) and
`PrismaticJoint2D` (linear). Both are off by default, so existing joints behave
exactly as before until you enable them.

### Revolute joint (hinge)

```cpp
// Motor: drive relative angular velocity toward a target, bounded by max torque.
hinge->EnableMotor(true);
hinge->SetMotorSpeed(3.14f);        // radians/second (positive = B rotates + relative to A)
hinge->SetMaxMotorTorque(5000.0f);  // torque budget

// Limits: clamp the relative angle to a range, measured from the joint's
// orientation at construction (angle 0 = the pose when the joint was created).
hinge->EnableLimit(true);
hinge->SetLimits(-0.5f, 0.5f);      // radians [lower, upper]

// Introspection:
float angle = hinge->GetJointAngle();   // radians, relative to reference
float speed = hinge->GetJointSpeed();   // radians/second
float torque = hinge->GetMotorTorque(1.0f / fixedDeltaTime);
```

Use the motor for powered hinges (turrets, motorized doors, wheels) and limits for
swinging doors, levers, and ragdoll joints. Motor and limit can be combined: a motor
that runs a hinge into its limit will hold there against the torque budget.

### Prismatic joint (slider)

```cpp
// Motor: drive translation speed along the joint axis, bounded by max force.
slider->EnableMotor(true);
slider->SetMotorSpeed(200.0f);      // world units/second along the axis
slider->SetMaxMotorForce(8000.0f);  // force budget

// Limits: clamp the axial translation (see GetTranslation) to a range.
slider->EnableLimit(true);
slider->SetLimits(0.0f, 300.0f);    // world units [lower, upper]

float force = slider->GetMotorForce(1.0f / fixedDeltaTime);
float t = slider->GetTranslation(); // current translation along the axis
```

Use it for elevators, sliding doors, pistons, and moving platforms — e.g. an elevator
is a prismatic joint with a vertical axis, a motor to move it, and limits at the top
and bottom of the shaft.

### Notes on the solver

- Motor and limit impulses are solved on the joint's shared axis each velocity
  iteration and are **not** warm-started across sub-steps; with the default 8 velocity
  iterations they converge within a step. The primary joint constraint keeps its
  existing warm start.
- Limits use a speculative velocity bias to begin arresting motion just before the
  limit is reached (anti-tunneling); any residual overshoot is pushed out by the
  position solver.
- `SetLimits` accepts its arguments in any order and stores them as `lower <= upper`.

---

## Build / verification

No new files, includes, or CMake changes are required — all four features live inside
the existing `Engine/Physics` translation units. Every modified unit
(`Rigidbody2D`, `Collider2D`, `PhysicsWorld2D`, `RevoluteJoint2D`, `PrismaticJoint2D`)
was verified to compile to object files cleanly with `-std=c++17`.
