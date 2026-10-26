add_executable(game
    src/main.cpp
    src/game_app.cpp

    src/screens/main_menu.cpp
    src/screens/new_game.cpp
    src/screens/settings.cpp
    src/screens/credits.cpp

    src/playfield/playfield_view.cpp
    src/playfield/world_renderer.cpp

    src/ui/status_bar.cpp
    src/ui/debug_frame.cpp
    src/ui/debug_crew_paths.cpp
    src/ui/build_bar.cpp
    src/ui/view_selector.cpp
    src/ui/build_icons.cpp
    src/ui/selection_panel.cpp

    src/features/build_catalog.cpp
    src/features/habitat_placement.cpp
    src/features/resource_placement.cpp
    src/features/utility_builder.cpp
    src/features/walkway_builder.cpp
    src/features/corridor_builder.cpp

    src/systems/gameplay_controller.cpp
    src/systems/world_selection.cpp
    src/systems/camera_controller.cpp
    src/systems/colony_status.cpp
    src/systems/construction_system.cpp

    src/world/playfield.cpp
    src/world/habitat_mesh.cpp
    src/world/resource_mesh.cpp
    src/world/construction_sequence.cpp
    src/world/utility_network.cpp
    src/world/walkway_network.cpp
    src/world/habitats/airlock.cpp
    src/world/habitats/oxygen_dome.cpp
    src/world/habitats/cantine_dome.cpp
    src/world/habitats/bio_dome.cpp
    src/world/habitats/crew_quarters.cpp
    src/world/habitats/processing_facility.cpp
    src/world/habitats/factory.cpp
    src/world/habitats/manufactory.cpp
    src/world/habitats/sick_bay.cpp
    src/world/habitats/laboratory.cpp
    src/world/habitats/interiors.cpp

    src/world/resources/solar_panel.cpp
    src/world/resources/water_driller.cpp
    src/world/resources/power_storage.cpp
    src/world/resources/water_storage.cpp
    src/world/resources/ore_driller.cpp
    src/world/resources/water_purifier.cpp
    src/world/resources/antenna.cpp
    src/world/resources/landing_pad.cpp

    src/world/corridor_network.cpp
    src/world/corridor/geometry.cpp
    src/world/corridor/fittings.cpp
    src/world/corridor/entrances.cpp
    src/world/corridor/routing.cpp
    src/world/corridor/factory.cpp
    src/world/corridor/fan_optimizer.cpp
)
target_include_directories(game PRIVATE include)
target_link_libraries(game PRIVATE game_engine)
target_compile_definitions(game PRIVATE GAME_VERSION="${PROJECT_VERSION}")
target_compile_options(game PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(game PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
