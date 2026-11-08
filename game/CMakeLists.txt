# A save or preference can never enable debug tools in a Release build.
add_compile_definitions(GAME_DEBUG_FEATURES=$<IF:$<CONFIG:Debug>,1,0>)

include(cmake/GameClusters.cmake)

set(DEBUG_BLUEPRINT_FIXTURE "${CMAKE_CURRENT_SOURCE_DIR}/debug-templates/0-40-32-debug-start.gmsave")
if(EXISTS "${DEBUG_BLUEPRINT_FIXTURE}")
    set(staged_debug_template "$<$<CONFIG:Debug>:${DEBUG_BLUEPRINT_FIXTURE}>")
endif()

add_executable(game src/main.cpp)
target_include_directories(game PRIVATE include)
target_link_libraries(game PRIVATE game_engine Threads::Threads rvo2)
target_compile_definitions(game PRIVATE GAME_VERSION="${GAME_VERSION_FULL}")
if(MSVC)
    target_compile_options(game PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(game PRIVATE -Wall -Wextra -Wpedantic)
endif()

foreach(cluster IN LISTS GAME_CLUSTER_TARGETS)
    target_sources(game PRIVATE $<TARGET_OBJECTS:${cluster}>)
endforeach()

if(GAME_BUILD_NAVIGATION_TESTS)
    add_executable(navigation_regression tests/navigation_regression.cpp)
    target_include_directories(navigation_regression PRIVATE include)
    target_link_libraries(navigation_regression PRIVATE game_engine Threads::Threads rvo2)
    foreach(cluster IN LISTS GAME_CLUSTER_TARGETS)
        target_sources(navigation_regression PRIVATE $<TARGET_OBJECTS:${cluster}>)
    endforeach()
    add_executable(ui_regression tests/ui_regression.cpp)
    target_include_directories(ui_regression PRIVATE include)
    target_link_libraries(ui_regression PRIVATE game_engine Threads::Threads rvo2)
    foreach(cluster IN LISTS GAME_CLUSTER_TARGETS)
        target_sources(ui_regression PRIVATE $<TARGET_OBJECTS:${cluster}>)
    endforeach()
    if(staged_debug_template)
        target_compile_definitions(ui_regression PRIVATE "$<$<CONFIG:Debug>:GAME_EXPECT_STAGED_DEBUG_TEMPLATE=1>")
        add_custom_target(stage_ui_test_runtime
            COMMAND "${CMAKE_COMMAND}" "-DRUNTIME_DIR=$<TARGET_FILE_DIR:ui_regression>"
                "-DDEBUG_TEMPLATE=${staged_debug_template}"
                -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/StageRuntime.cmake"
            VERBATIM
        )
        add_dependencies(ui_regression stage_ui_test_runtime)
    endif()
    add_test(NAME game_build_order_ui COMMAND ui_regression build-order)
    add_test(NAME game_activity_log_ui COMMAND ui_regression activity-log)
    add_test(NAME game_build_mode COMMAND ui_regression build-mode)
    add_test(NAME game_graphics_quality COMMAND ui_regression graphics-quality)
    function(add_debug_regression name)
        add_test(NAME "${name}" ${ARGN} CONFIGURATIONS Debug)
    endfunction()
    add_debug_regression(ui_resolution_scaling COMMAND ui_regression)
    add_debug_regression(navigation_fault_recovery COMMAND navigation_regression fault-recovery)
    add_debug_regression(navigation_fleet COMMAND navigation_regression)
    add_debug_regression(navigation_geometry COMMAND navigation_regression geometry)
    add_debug_regression(navigation_pressure_gates COMMAND navigation_regression gates)
    add_debug_regression(navigation_storage_extensions COMMAND navigation_regression extensions)
    set_tests_properties(navigation_storage_extensions PROPERTIES TIMEOUT 300)
    add_debug_regression(navigation_workshops COMMAND navigation_regression workshops)
    set_tests_properties(navigation_workshops PROPERTIES TIMEOUT 300)
    add_debug_regression(colony_interaction COMMAND navigation_regression interaction)
    add_debug_regression(colony_ore_carrier COMMAND navigation_regression ore-carrier)
    add_debug_regression(colony_conveyor_ore COMMAND navigation_regression conveyor-ore)
    add_debug_regression(colony_structure_routes COMMAND navigation_regression structure-routes)
    add_debug_regression(colony_oxygen COMMAND navigation_regression oxygen)
    add_debug_regression(colony_progression COMMAND navigation_regression progression)
    add_debug_regression(colony_robot_battery_bootstrap COMMAND navigation_regression battery-bootstrap)
    add_debug_regression(colony_hydrogen_solar COMMAND navigation_regression hydrogen-solar)
    add_debug_regression(colony_hydrogen_refueling COMMAND navigation_regression hydrogen-refueling)
    add_debug_regression(colony_hydroponics COMMAND navigation_regression hydroponics)
    add_debug_regression(navigation_global_routes COMMAND navigation_regression global-routes)
    add_debug_regression(debug_robot_orders COMMAND navigation_regression debug-orders)
    add_debug_regression(colony_airlock_cycle COMMAND navigation_regression airlock-cycle)
    add_debug_regression(colony_crew_autonomy COMMAND navigation_regression crew-autonomy)
    add_debug_regression(colony_crew_waypoints COMMAND navigation_regression crew-waypoints)
    set_tests_properties(colony_crew_autonomy PROPERTIES TIMEOUT 180)
    add_debug_regression(colony_airlock_connections COMMAND navigation_regression airlock)
    add_debug_regression(construction_reordered_builds COMMAND navigation_regression build-order)
    add_debug_regression(construction_queue_flow COMMAND navigation_regression queue-flow)
    add_debug_regression(construction_priority COMMAND navigation_regression priority)
    add_debug_regression(colony_supply_network COMMAND navigation_regression network)
    add_debug_regression(colony_supply COMMAND navigation_regression supply)
    add_debug_regression(colony_power_switches COMMAND navigation_regression power-switches)
    add_debug_regression(navigation_utilities COMMAND navigation_regression utilities)
    # Optional regression fixture. Building the game and tests requires only sources.
    if(EXISTS "${DEBUG_BLUEPRINT_FIXTURE}")
        add_debug_regression(navigation_fault_construction COMMAND navigation_regression fault-construction "${DEBUG_BLUEPRINT_FIXTURE}")
        add_debug_regression(navigation_construction COMMAND navigation_regression construction "${DEBUG_BLUEPRINT_FIXTURE}")
        add_debug_regression(navigation_late_corridor COMMAND navigation_regression late "${DEBUG_BLUEPRINT_FIXTURE}")
        set_tests_properties(navigation_fault_construction navigation_construction PROPERTIES TIMEOUT 300)
    endif()
endif()

set(GAME_BIN_DIR "${CMAKE_BINARY_DIR}/bin" CACHE PATH "Directory containing both game executables")
set_target_properties(game PROPERTIES
    OUTPUT_NAME_DEBUG "game-debug"
    # The generator expression keeps multi-config generators from adding a
    # configuration subdirectory, so both variants share the save directory.
    RUNTIME_OUTPUT_DIRECTORY "$<1:${GAME_BIN_DIR}>")

if(WIN32)
    set(GAME_OPENAL_RUNTIME "${PROJECT_SOURCE_DIR}/build/windows/bin/OpenAL32.dll" CACHE FILEPATH
        "OpenAL Soft Win64 DLL staged beside the game")
    set(openal_destination "OpenAL32.dll")
else()
    find_file(GAME_OPENAL_RUNTIME NAMES libopenal.so.1 PATHS ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})
    set(openal_destination "lib/libopenal.so.1")
    set_property(TARGET game APPEND PROPERTY BUILD_RPATH "$ORIGIN/lib")
    if(NOT CMAKE_CROSSCOMPILING)
        set(openal_notice "/usr/share/doc/libopenal1/copyright")
    endif()
endif()

# Stage loose runtime files on incremental builds as well as fresh builds.
add_custom_target(stage_game_runtime
    COMMAND "${CMAKE_COMMAND}" "-DRUNTIME_DIR=$<TARGET_FILE_DIR:game>"
        "-DDEBUG_TEMPLATE=${staged_debug_template}"
        "-DOPENAL_LIBRARY=${GAME_OPENAL_RUNTIME}"
        "-DOPENAL_DESTINATION=${openal_destination}"
        "-DOPENAL_NOTICE=${openal_notice}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/StageRuntime.cmake"
    VERBATIM
)
add_dependencies(game stage_game_runtime)

if(WIN32)
    set(GAME_D3D12_AGILITY_DIR "" CACHE PATH
        "Optional DirectX 12 Agility SDK 1.620.1 x64 runtime directory")
    if(DEFINED ENV{GAME_D3D12_AGILITY_DIR})
        set(GAME_D3D12_AGILITY_DIR "$ENV{GAME_D3D12_AGILITY_DIR}")
    endif()
    if(GAME_D3D12_AGILITY_DIR)
        set(d3d12_core "${GAME_D3D12_AGILITY_DIR}/D3D12Core.dll")
        set(d3d12_layers "${GAME_D3D12_AGILITY_DIR}/D3D12SDKLayers.dll")
        if(NOT EXISTS "${d3d12_core}")
            message(FATAL_ERROR "GAME_D3D12_AGILITY_DIR must contain the x64 Agility SDK 1.620.1 D3D12Core.dll")
        endif()
        set(d3d12_runtime_files "${d3d12_core}")
        if(EXISTS "${d3d12_layers}")
            list(APPEND d3d12_runtime_files "${d3d12_layers}")
        endif()
        target_compile_definitions(game PRIVATE GAME_D3D12_AGILITY_VERSION=620)
        add_custom_target(stage_d3d12_agility
            COMMAND "${CMAKE_COMMAND}" -E make_directory "${GAME_BIN_DIR}/D3D12"
            COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${d3d12_runtime_files} "${GAME_BIN_DIR}/D3D12"
            VERBATIM
        )
        add_dependencies(game stage_d3d12_agility)
    endif()
endif()

if(GAME_BUILD_NAVIGATION_TESTS)
    add_executable(lander_regression tests/lander_regression.cpp)
    target_include_directories(lander_regression PRIVATE include)
    target_compile_definitions(lander_regression PRIVATE GAME_LEGACY_LANDER_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
    target_link_libraries(lander_regression PRIVATE game_engine Threads::Threads rvo2)
    foreach(cluster IN LISTS GAME_CLUSTER_TARGETS)
        target_sources(lander_regression PRIVATE $<TARGET_OBJECTS:${cluster}>)
    endforeach()
    add_test(NAME lunar_lander_deployment COMMAND lander_regression)
    set_tests_properties(lunar_lander_deployment PROPERTIES TIMEOUT 180)
endif()

if(GAME_BUILD_NAVIGATION_TESTS)
    add_executable(trader_regression tests/trader_regression.cpp)
    target_include_directories(trader_regression PRIVATE include)
    target_link_libraries(trader_regression PRIVATE game_engine Threads::Threads rvo2)
    foreach(cluster IN LISTS GAME_CLUSTER_TARGETS)
        target_sources(trader_regression PRIVATE $<TARGET_OBJECTS:${cluster}>)
    endforeach()
    add_test(NAME lunar_trader_logistics COMMAND trader_regression CONFIGURATIONS Debug)
    set_tests_properties(lunar_trader_logistics PROPERTIES TIMEOUT 180)
endif()
