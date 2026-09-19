file(
    GLOB_RECURSE
    RUNTIME_SOURCES

    CONFIGURE_DEPENDS

    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)


add_executable(
    2DGRuntime
    ${RUNTIME_SOURCES}
)


target_include_directories(
    2DGRuntime

    PRIVATE

    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Game/Components
)


target_link_libraries(
    2DGRuntime

    PRIVATE

    Engine::Core
)


set_target_properties(
    2DGRuntime
    PROPERTIES
    FOLDER "Runtime"
)


copy_2dg_runtime_dependencies(
    2DGRuntime
)


add_custom_command(
    TARGET 2DGRuntime
    POST_BUILD

    COMMAND
        ${CMAKE_COMMAND}
        -E
        copy_directory
        "${CMAKE_SOURCE_DIR}/Content"
        "$<TARGET_FILE_DIR:2DGRuntime>/Content"
)