cmake_minimum_required(VERSION 3.20)

project(
    2DGEngine
    VERSION 0.1.0
    LANGUAGES CXX
)

# ============================================================
# IDE organization
# ============================================================

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ============================================================
# CMake helper modules
# ============================================================

list(
    APPEND
    CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
)

include(RuntimeDependencies)

# ---------------------------------------------------------------------------
# Third-party libraries
# ---------------------------------------------------------------------------
add_subdirectory(ThirdParty/SDL)

set(SDLIMAGE_VENDORED OFF CACHE BOOL "Use vendored dependencies" FORCE)

add_subdirectory(ThirdParty/SDL_image)

# ============================================================
# Engine
# ============================================================

add_subdirectory(Engine)

# ============================================================
# Runtime
# ============================================================

add_subdirectory(Runtime)

# ============================================================
# Tests
# ============================================================

include(CTest)

if(BUILD_TESTING)

    add_subdirectory(Tests)

endif()
