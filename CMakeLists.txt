cmake_minimum_required(VERSION 3.20)

project(2DGEngine
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------
# Third-party libraries
# ---------------------------------------------------------

add_subdirectory(ThirdParty/SDL)

# Disable vendored dependencies for SDL_image to use system packages
set(SDLIMAGE_VENDORED OFF CACHE BOOL "Use vendored dependencies" FORCE)
add_subdirectory(ThirdParty/SDL_image)

# ---------------------------------------------------------
# Runtime executable
# ---------------------------------------------------------

add_executable(2DGRuntime
    Runtime/Main.cpp

    Runtime/Game/Components/PlayerControllerComponent.cpp
    Runtime/Game/Components/PlayerControllerComponent.h

    Runtime/Game/Components/RotatorComponent.h

    Runtime/Game/Components/LifetimeComponent.cpp
    Runtime/Game/Components/LifetimeComponent.h

    Engine/Core/Application.cpp
    Engine/Core/Application.h

    Engine/Core/Window.cpp
    Engine/Core/Window.h

    Engine/Core/EngineTime.cpp
    Engine/Core/EngineTime.h

    Engine/Events/EventSubscription.h
    Engine/Events/EventBus.h
    Engine/Events/EventBus.cpp

    Engine/Graphics/AnimationFrame2D.h
    Engine/Graphics/AnimationClip2D.cpp
    Engine/Graphics/AnimationClip2D.h
    Engine/Graphics/SpriteFrame2D.h
    Engine/Graphics/AnimatorCondition2D.h
    Engine/Graphics/AnimatorConditionMode.h
    Engine/Graphics/AnimatorParameter2D.h
    Engine/Graphics/AnimatorParameterType.h
    Engine/Graphics/AnimatorState2D.h
    Engine/Graphics/AnimatorTransition2D.h
    Engine/Graphics/AnimationEvent2D.h

    Engine/Graphics/Renderer2D.cpp
    Engine/Graphics/Renderer2D.h

    Engine/Graphics/Texture2D.cpp
    Engine/Graphics/Texture2D.h

    Engine/Graphics/Sprite.cpp
    Engine/Graphics/Sprite.h

    Engine/Graphics/Camera2D.cpp
    Engine/Graphics/Camera2D.h

    Engine/Graphics/SortingLayer.h
    Engine/Graphics/SpriteFrame2D.h
    Engine/Graphics/SpriteRegion.h

    Engine/Graphics/PrimitiveShape2D.h

    Engine/Graphics/PrimitiveTextureFactory2D.cpp
    Engine/Graphics/PrimitiveTextureFactory2D.h

    Engine/Input/Input.cpp
    Engine/Input/Input.h

    Engine/Math/Vector2.h
    Engine/Math/Rect.h
    Engine/Math/Color.h

    Engine/Math/Transform2D.cpp
    Engine/Math/Transform2D.h

    Engine/Math/Bounds2D.cpp
    Engine/Math/Bounds2D.h

    Engine/Scene/Scene.cpp
    Engine/Scene/Scene.h

    Engine/Scene/Component.cpp
    Engine/Scene/Component.h

    Engine/Scene/Entity.cpp
    Engine/Scene/Entity.h

    Engine/Scene/EntityHandle.cpp
    Engine/Scene/EntityHandle.h

    Engine/Scene/SpriteRendererComponent.cpp
    Engine/Scene/SpriteRendererComponent.h

    Engine/Scene/TransformComponent.cpp
    Engine/Scene/TransformComponent.h

    Engine/Scene/AnimatorComponent.cpp
    Engine/Scene/AnimatorComponent.h

    Engine/Scene/Primitive2DFactory.cpp
    Engine/Scene/Primitive2DFactory.h

    Engine/Physics/CollisionLayer2D.h
    Engine/Physics/CollisionEvents2D.h
    Engine/Physics/ColliderPair2D.h
    Engine/Physics/CachedContact2D.h
    Engine/Physics/CachedContactPair2D.h
    Engine/Physics/BodyType2D.h

    Engine/Physics/SpatialCell2D.h
    Engine/Physics/SweptAABBHit2D.h
    Engine/Physics/CollisionDetectionMode2D.h

    Engine/Physics/PhysicsDebugRenderer2D.cpp
    Engine/Physics/PhysicsDebugRenderer2D.h
    Engine/Physics/PhysicsDebugStatus2D.h

    Engine/Physics/SweepHit2D.h
    Engine/Physics/PhysicsQueryFilter2D.h
    Engine/Physics/PhysicsQueryContext2D.h
    
    Engine/Physics/ShapeCastHit2D.h
    Engine/Physics/OverlapHit2D.h
    Engine/Physics/RaycastHit2D.h

    Engine/Physics/OrientedBox2D.h
    Engine/Physics/BroadPhaseProxy2D.h

    Engine/Physics/Collider2D.h
    Engine/Physics/Collider2D.cpp

    Engine/Physics/BoxCollider2D.h
    Engine/Physics/BoxCollider2D.cpp

    Engine/Physics/CircleCollider2D.cpp
    Engine/Physics/CircleCollider2D.h

    Engine/Physics/PhysicsWorld2D.h
    Engine/Physics/PhysicsWorld2D.cpp

    Engine/Physics/Rigidbody2D.cpp
    Engine/Physics/Rigidbody2D.h

    Engine/Physics/CollisionManifold2D.h
    Engine/Physics/PhysicsMaterial2D.h
)

target_include_directories(2DGRuntime
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    Engine/Core
    Engine/Events
    Engine/Graphics
    Engine/Input
    Engine/Math
    Engine/Scene
    Engine/Physics

    Runtime/Game/Components
)

target_link_libraries(2DGRuntime
    PRIVATE
    SDL3::SDL3
    SDL3_image::SDL3_image
)

# Copy DLLs to output directory
add_custom_command(TARGET 2DGRuntime POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:SDL3::SDL3>
    $<TARGET_FILE_DIR:2DGRuntime>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:SDL3_image::SDL3_image>
    $<TARGET_FILE_DIR:2DGRuntime>
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/Content
    $<TARGET_FILE_DIR:2DGRuntime>/Content
)