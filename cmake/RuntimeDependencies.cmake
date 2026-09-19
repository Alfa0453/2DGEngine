function(copy_2dg_runtime_dependencies TARGET_NAME)

    add_custom_command(
        TARGET ${TARGET_NAME}
        POST_BUILD

        COMMAND
            ${CMAKE_COMMAND}
            -E
            copy_if_different
            $<TARGET_FILE:SDL3::SDL3>
            $<TARGET_FILE_DIR:${TARGET_NAME}>

        COMMAND
            ${CMAKE_COMMAND}
            -E
            copy_if_different
            $<TARGET_FILE:SDL3_image::SDL3_image>
            $<TARGET_FILE_DIR:${TARGET_NAME}>
    )

endfunction()