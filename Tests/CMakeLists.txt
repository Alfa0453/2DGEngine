file(
    GLOB_RECURSE
    TEST_SOURCES

    CONFIGURE_DEPENDS

    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)


add_executable(
    2DGTests
    ${TEST_SOURCES}
)


target_link_libraries(
    2DGTests

    PRIVATE

    Engine::Core
)


set_target_properties(
    2DGTests
    PROPERTIES
    FOLDER "Tests"
)


copy_2dg_runtime_dependencies(
    2DGTests
)


add_test(
    NAME
        2DGTests

    COMMAND
        2DGTests
)