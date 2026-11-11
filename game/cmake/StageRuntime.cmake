cmake_minimum_required(VERSION 3.20)

# Release and Debug can stage into the same runtime directory.
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
endif()

if(OPENAL_LIBRARY AND EXISTS "${OPENAL_LIBRARY}")
    stage_file("${OPENAL_LIBRARY}" "${OPENAL_DESTINATION}")
    if(OPENAL_NOTICE AND EXISTS "${OPENAL_NOTICE}")
        stage_file("${OPENAL_NOTICE}" "licenses/OpenAL-Soft.txt")
    endif()
endif()

if(UI_FONT_NOTICE)
    stage_file("${UI_FONT_NOTICE}" "licenses/HUD-Font.txt")
endif()
