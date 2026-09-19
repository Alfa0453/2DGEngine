add_engine_subsystem(
    Core

    PUBLIC_DEPENDENCIES
        Engine::Math
        Engine::Events
        Engine::Input
        Engine::Graphics
        Engine::Audio
        Engine::Scene
        Engine::Physics

        SDL3::SDL3
)