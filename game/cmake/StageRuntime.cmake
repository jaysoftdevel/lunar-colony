cmake_minimum_required(VERSION 3.20)

# Debug features are optional in the single game runtime.
file(MAKE_DIRECTORY "${RUNTIME_DIR}")
file(LOCK "${RUNTIME_DIR}/../.runtime-stage.lock" GUARD PROCESS TIMEOUT 60)

function(stage_file source relative_destination)
    file(REAL_PATH "${source}" input)
    set(destination "${RUNTIME_DIR}/${relative_destination}")
    if(EXISTS "${destination}")
        file(REAL_PATH "${destination}" output)
        if(input STREQUAL output)
            return()
        endif()
    endif()
    configure_file("${input}" "${destination}" COPYONLY)
endfunction()

if(DEBUG_TEMPLATE)
    get_filename_component(template_name "${DEBUG_TEMPLATE}" NAME)
    stage_file("${DEBUG_TEMPLATE}" "debug/${template_name}")
else()
    file(REMOVE "${RUNTIME_DIR}/debug/0-54-0-debug-start.gmsave")
endif()

if(UI_FONT_NOTICE)
    stage_file("${UI_FONT_NOTICE}" "licenses/HUD-Font.txt")
endif()
