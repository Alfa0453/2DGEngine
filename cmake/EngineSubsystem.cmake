include_guard(GLOBAL)


function(add_engine_subsystem NAME)

    set(options)

    set(oneValueArgs)

    set(multiValueArgs
        PUBLIC_DEPENDENCIES
        PRIVATE_DEPENDENCIES
    )


    cmake_parse_arguments(
        SUBSYSTEM
        "${options}"
        "${oneValueArgs}"
        "${multiValueArgs}"
        ${ARGN}
    )


    file(
        GLOB_RECURSE
        SUBSYSTEM_SOURCES

        CONFIGURE_DEPENDS

        "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
    )


    set(
        TARGET_NAME
        "Engine_${NAME}"
    )


    add_library(
        ${TARGET_NAME}
        STATIC
        ${SUBSYSTEM_SOURCES}
    )


    add_library(
        Engine::${NAME}
        ALIAS
        ${TARGET_NAME}
    )


    target_include_directories(
        ${TARGET_NAME}

        PUBLIC

        ${CMAKE_CURRENT_SOURCE_DIR}
    )


    target_link_libraries(
        ${TARGET_NAME}

        PUBLIC

        Engine::ProjectOptions
        ${SUBSYSTEM_PUBLIC_DEPENDENCIES}

        PRIVATE

        ${SUBSYSTEM_PRIVATE_DEPENDENCIES}
    )


    set_target_properties(
        ${TARGET_NAME}

        PROPERTIES

        FOLDER
        "Engine/${NAME}"
    )

endfunction()