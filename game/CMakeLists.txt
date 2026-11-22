# A save or preference can never enable debug tools in a Release build.
add_compile_definitions(GAME_DEBUG_FEATURES=$<IF:$<CONFIG:Debug>,1,0>)

include(cmake/Runtime.cmake)
include(cmake/GameClusters.cmake)

set(DEBUG_BLUEPRINT_FIXTURE "${CMAKE_CURRENT_SOURCE_DIR}/debug-templates/0-54-0-debug-start.gmsave")
set(NAVIGATION_CONSTRUCTION_FIXTURE "${CMAKE_CURRENT_SOURCE_DIR}/debug-templates/0-40-32-debug-start.gmsave")
if(EXISTS "${DEBUG_BLUEPRINT_FIXTURE}")
    set(staged_debug_template "$<$<CONFIG:Debug>:${DEBUG_BLUEPRINT_FIXTURE}>")
endif()

add_executable(game src/main.cpp)
target_include_directories(game PRIVATE include)
target_link_libraries(game PRIVATE colony_runtime Threads::Threads rvo2)
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
    target_link_libraries(navigation_regression PRIVATE colony_runtime Threads::Threads rvo2)
    foreach(cluster IN LISTS GAME_CLUSTER_TARGETS)
        target_sources(navigation_regression PRIVATE $<TARGET_OBJECTS:${cluster}>)
    endforeach()
    add_executable(ui_regression tests/ui_regression.cpp)
    target_include_directories(ui_regression PRIVATE include)
    target_compile_definitions(ui_regression PRIVATE
        GAME_LEGACY_PLANNING_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/schema29-stage1-contracts.gmsave"
        GAME_SOURCE_ROOT="${PROJECT_SOURCE_DIR}")
    target_link_libraries(ui_regression PRIVATE colony_runtime Threads::Threads rvo2)
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
    add_test(NAME game_fov_station_interiors COMMAND ui_regression fov-interiors)
    add_test(NAME game_runtime_paths COMMAND ui_regression runtime-paths)
    add_test(NAME game_windows_build_driver COMMAND "${JSD_GUARD_PYTHON}" -B "${CMAKE_CURRENT_SOURCE_DIR}/tests/windows_build_test.py")
    add_test(NAME navigation_collision_radius COMMAND navigation_regression collision-radius)
    add_test(NAME navigation_rover_steering COMMAND navigation_regression rover-steering)
    add_test(NAME navigation_operation_replan COMMAND navigation_regression operation-replan)
    add_test(NAME navigation_lander_depot_routes COMMAND navigation_regression carrier-depot
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/schema48-lander-depot.gmsave")
    set_tests_properties(navigation_lander_depot_routes PROPERTIES TIMEOUT 90)
    add_test(NAME navigation_stage_package_clearance COMMAND navigation_regression stage-packages)
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
    if(EXISTS "${NAVIGATION_CONSTRUCTION_FIXTURE}")
        add_debug_regression(navigation_fault_construction COMMAND navigation_regression fault-construction "${NAVIGATION_CONSTRUCTION_FIXTURE}")
        add_debug_regression(navigation_construction COMMAND navigation_regression construction "${NAVIGATION_CONSTRUCTION_FIXTURE}")
        add_debug_regression(navigation_late_corridor COMMAND navigation_regression late "${NAVIGATION_CONSTRUCTION_FIXTURE}")
        set_tests_properties(navigation_fault_construction navigation_construction PROPERTIES TIMEOUT 300)
    endif()
endif()

set(GAME_BIN_DIR "${CMAKE_BINARY_DIR}/bin" CACHE PATH "Directory containing both game executables")
set_target_properties(game PROPERTIES
    OUTPUT_NAME_DEBUG "game-debug"
    # Keep both variants and their staged runtime files in the same bin directory.
    RUNTIME_OUTPUT_DIRECTORY "$<1:${GAME_BIN_DIR}>")

if(NOT WIN32)
    set_property(TARGET game APPEND PROPERTY BUILD_RPATH "$ORIGIN/lib")
endif()

# Stage loose runtime files on incremental builds as well as fresh builds.
add_custom_target(stage_game_runtime
    COMMAND "${CMAKE_COMMAND}" "-DRUNTIME_DIR=$<TARGET_FILE_DIR:game>"
        "-DDEBUG_TEMPLATE=${staged_debug_template}"
        "-DUI_FONT_NOTICE=${PROJECT_SOURCE_DIR}/third_party/ui_font/OFL.txt"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/StageRuntime.cmake"
    VERBATIM
)
add_dependencies(game stage_game_runtime)

if(GAME_BUILD_NAVIGATION_TESTS)
    add_executable(lander_regression tests/lander_regression.cpp)
    target_include_directories(lander_regression PRIVATE include)
    target_compile_definitions(lander_regression PRIVATE GAME_LEGACY_LANDER_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
    target_link_libraries(lander_regression PRIVATE colony_runtime Threads::Threads rvo2)
    foreach(cluster IN LISTS GAME_CLUSTER_TARGETS)
        target_sources(lander_regression PRIVATE $<TARGET_OBJECTS:${cluster}>)
    endforeach()
    add_test(NAME lunar_lander_deployment COMMAND lander_regression)
    set_tests_properties(lunar_lander_deployment PROPERTIES TIMEOUT 180)
endif()

if(GAME_BUILD_NAVIGATION_TESTS)
    add_executable(trader_regression tests/trader_regression.cpp)
    target_include_directories(trader_regression PRIVATE include)
    target_link_libraries(trader_regression PRIVATE colony_runtime Threads::Threads rvo2)
    foreach(cluster IN LISTS GAME_CLUSTER_TARGETS)
        target_sources(trader_regression PRIVATE $<TARGET_OBJECTS:${cluster}>)
    endforeach()
    add_test(NAME lunar_trader_logistics COMMAND trader_regression CONFIGURATIONS Debug)
    set_tests_properties(lunar_trader_logistics PROPERTIES TIMEOUT 180)
endif()

if(GAME_BUILD_NAVIGATION_TESTS)
    get_property(game_tests DIRECTORY PROPERTY TESTS)
    foreach(test IN LISTS game_tests)
        set_property(TEST "${test}" APPEND PROPERTY ENVIRONMENT "GAME_SAVE_DIR=${CMAKE_CURRENT_BINARY_DIR}/test-saves/${test}")
    endforeach()
endif()

# JSD is linked statically. Its source-built SDL runtime stays app-local.
if(WIN32)
    set(jsd_runtime_dir "$<TARGET_FILE_DIR:game>")
    set(jsd_sdl_name "$<TARGET_FILE_NAME:SDL2>")
else()
    set(jsd_runtime_dir "$<TARGET_FILE_DIR:game>/lib")
    set(jsd_sdl_name "$<TARGET_SONAME_FILE_NAME:SDL2>")
endif()
add_custom_target(stage_jsd_runtime
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${jsd_runtime_dir}" "$<TARGET_FILE_DIR:game>/licenses"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "$<TARGET_FILE:SDL2>" "${jsd_runtime_dir}/${jsd_sdl_name}"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${JSD_GAME_ENGINE_DIR}/LICENSE.txt" "$<TARGET_FILE_DIR:game>/licenses/JSD-engine.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${JSD_GAME_ENGINE_DIR}/third_party/SDL2/LICENSE.txt" "$<TARGET_FILE_DIR:game>/licenses/SDL2.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${JSD_GAME_ENGINE_DIR}/third_party/JoltPhysics/LICENSE" "$<TARGET_FILE_DIR:game>/licenses/Jolt.txt"
    DEPENDS SDL2
    VERBATIM
)
add_dependencies(game stage_jsd_runtime)

if(GAME_BUILD_NAVIGATION_TESTS AND GAME_BUILD_RENDER_TESTS)
    add_executable(jsd_scene_regression tests/jsd_scene_regression.cpp)
    target_include_directories(jsd_scene_regression PRIVATE include "${CMAKE_CURRENT_SOURCE_DIR}/src/runtime")
    target_link_libraries(jsd_scene_regression PRIVATE colony_runtime Threads::Threads rvo2)
    foreach(cluster IN LISTS GAME_CLUSTER_TARGETS)
        target_sources(jsd_scene_regression PRIVATE $<TARGET_OBJECTS:${cluster}>)
    endforeach()
    add_test(NAME jsd_game_scenes COMMAND jsd_scene_regression)
    set_tests_properties(jsd_game_scenes PROPERTIES TIMEOUT 180 ENVIRONMENT "GAME_SAVE_DIR=${CMAKE_CURRENT_BINARY_DIR}/test-saves/jsd-game-scenes")
endif()

if(GAME_BUILD_NAVIGATION_TESTS AND GAME_BUILD_RENDER_TESTS)
    add_executable(jsd_surface_regression tests/jsd_surface_regression.cpp)
    target_link_libraries(jsd_surface_regression PRIVATE JSD::GameEngineStatic)
    add_test(NAME jsd_aperiodic_materials COMMAND jsd_surface_regression)
    set_tests_properties(jsd_aperiodic_materials PROPERTIES TIMEOUT 180)
endif()

# Tests live in their own configuration directories on Windows. Each executable
# must receive the matching source-built SDL DLL, not just the main game binary.
if(WIN32)
    add_custom_target(stage_colony_windows_runtime)
    add_dependencies(stage_colony_windows_runtime SDL2)
    get_property(colony_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
    foreach(target IN LISTS colony_targets)
        get_target_property(type ${target} TYPE)
        if(type STREQUAL "EXECUTABLE")
            add_custom_command(TARGET stage_colony_windows_runtime POST_BUILD
                COMMAND "${CMAKE_COMMAND}" -E make_directory "$<TARGET_FILE_DIR:${target}>"
                COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                    "$<TARGET_FILE:SDL2>" "$<TARGET_FILE_DIR:${target}>"
                VERBATIM)
            add_dependencies(${target} stage_colony_windows_runtime)
        endif()
    endforeach()
endif()
