include(EngineSubsystem)

# ============================================================
# Shared engine build settings
# ============================================================

add_library(
    Engine_ProjectOptions
    INTERFACE
)

add_library(
    Engine::ProjectOptions
    ALIAS
    Engine_ProjectOptions
)

target_compile_features(
    Engine_ProjectOptions
    INTERFACE
    cxx_std_20
)

target_include_directories(
    Engine_ProjectOptions
    INTERFACE
    ${CMAKE_SOURCE_DIR}
)

set_target_properties(
    Engine_ProjectOptions
    PROPERTIES
    FOLDER "Engine"
)

# ============================================================
# Engine subsystems
# ============================================================

add_subdirectory(Math)

add_subdirectory(Events)

add_subdirectory(Input)

add_subdirectory(Graphics)

add_subdirectory(Audio)

add_subdirectory(Scene)

add_subdirectory(Physics)

add_subdirectory(Core)
