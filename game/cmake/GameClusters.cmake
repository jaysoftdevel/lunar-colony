# v0.40 implementation clusters
#
# Keep these source groups deliberately explicit. They are both build targets
# and context boundaries: future work should start from the smallest cluster(s)
# named in IMPLEMENTATION_CLUSTERS.md rather than loading the entire game.

function(configure_game_cluster target)
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${target} PRIVATE game_engine)
    target_compile_definitions(${target} PRIVATE GAME_VERSION="${GAME_VERSION_FULL}")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endfunction()

add_library(game_cluster_app OBJECT
    src/game_app.cpp
    src/screens/main_menu.cpp
    src/screens/new_game.cpp
    src/screens/save_load.cpp
    src/screens/settings.cpp
    src/screens/credits.cpp
)
configure_game_cluster(game_cluster_app)

add_library(game_cluster_interaction OBJECT
    src/playfield/playfield_view.cpp
    src/systems/gameplay_controller.cpp
    src/systems/world_selection.cpp
    src/systems/camera_controller.cpp
    src/systems/first_person_controller.cpp
    src/systems/first_person_mode.cpp
)
configure_game_cluster(game_cluster_interaction)

add_library(game_cluster_rendering OBJECT
    src/playfield/world_renderer.cpp
    src/playfield/builder_robot_renderer.cpp
)
configure_game_cluster(game_cluster_rendering)

add_library(game_cluster_ui OBJECT
    src/ui/status_bar.cpp
    src/ui/debug_frame.cpp
    src/ui/build_bar.cpp
    src/ui/view_selector.cpp
    src/ui/build_icons.cpp
    src/ui/selection_panel.cpp
    src/ui/construction_status_frames.cpp
    src/ui/debug_crew_paths.cpp
)
configure_game_cluster(game_cluster_ui)

add_library(game_cluster_construction OBJECT
    src/features/build_catalog.cpp
    src/features/habitat_placement.cpp
    src/features/resource_placement.cpp
    src/features/utility_builder.cpp
    src/features/walkway_builder.cpp
    src/features/corridor_builder.cpp
    src/systems/construction_costs.cpp
    src/systems/construction_system.cpp
    src/systems/resource_management.cpp
)
configure_game_cluster(game_cluster_construction)

add_library(game_cluster_robots OBJECT
    src/systems/builder_robot.cpp
)
configure_game_cluster(game_cluster_robots)

add_library(game_cluster_pathfinding OBJECT
    src/robots/path_planner.cpp
)
configure_game_cluster(game_cluster_pathfinding)

add_library(game_cluster_simulation OBJECT
    src/systems/colony_status.cpp
    src/systems/save_game.cpp
)
configure_game_cluster(game_cluster_simulation)

add_library(game_cluster_world_core OBJECT
    src/world/playfield.cpp
    src/world/habitat_mesh.cpp
    src/world/resource_mesh.cpp
    src/world/construction_sequence.cpp
)
configure_game_cluster(game_cluster_world_core)

add_library(game_cluster_habitats OBJECT
    src/world/habitats/module.cpp
    src/world/habitats/airlock.cpp
    src/world/habitats/airlock_inner.cpp
    src/world/habitats/oxygen_dome.cpp
    src/world/habitats/oxygen_dome_inner.cpp
    src/world/habitats/cantine_dome.cpp
    src/world/habitats/cantine_dome_inner.cpp
    src/world/habitats/bio_dome.cpp
    src/world/habitats/bio_dome_inner.cpp
    src/world/habitats/crew_quarters.cpp
    src/world/habitats/crew_quarters_inner.cpp
    src/world/habitats/processing_facility.cpp
    src/world/habitats/processing_facility_inner.cpp
    src/world/habitats/factory.cpp
    src/world/habitats/factory_inner.cpp
    src/world/habitats/manufactory.cpp
    src/world/habitats/manufactory_inner.cpp
    src/world/habitats/sick_bay.cpp
    src/world/habitats/sick_bay_inner.cpp
    src/world/habitats/laboratory.cpp
    src/world/habitats/laboratory_inner.cpp
)
configure_game_cluster(game_cluster_habitats)

add_library(game_cluster_resources OBJECT
    src/world/resources/solar_panel.cpp
    src/world/resources/water_driller.cpp
    src/world/resources/power_storage.cpp
    src/world/resources/water_storage.cpp
    src/world/resources/ore_driller.cpp
    src/world/resources/water_purifier.cpp
    src/world/resources/antenna.cpp
    src/world/resources/landing_pad.cpp
    src/world/resources/storage_dome.cpp
)
configure_game_cluster(game_cluster_resources)

add_library(game_cluster_networks OBJECT
    src/world/corridor/module.cpp
    src/world/corridor_network.cpp
    src/world/corridor/geometry.cpp
    src/world/corridor/fittings.cpp
    src/world/corridor/entrances.cpp
    src/world/corridor/routing.cpp
    src/world/corridor/factory.cpp
    src/world/corridor/fan_optimizer.cpp
    src/world/utility_network.cpp
    src/world/walkway_network.cpp
)
configure_game_cluster(game_cluster_networks)

set(GAME_CLUSTER_TARGETS
    game_cluster_app
    game_cluster_interaction
    game_cluster_rendering
    game_cluster_ui
    game_cluster_construction
    game_cluster_robots
    game_cluster_pathfinding
    game_cluster_simulation
    game_cluster_world_core
    game_cluster_habitats
    game_cluster_resources
    game_cluster_networks
)
