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
    src/ui/build_bar.cpp
    src/ui/view_selector.cpp
    src/ui/build_icons.cpp
    src/ui/selection_panel.cpp

    src/features/build_catalog.cpp
    src/features/habitat_placement.cpp
    src/features/corridor_builder.cpp

    src/systems/gameplay_controller.cpp
    src/systems/world_selection.cpp
    src/systems/camera_controller.cpp
    src/systems/colony_status.cpp

    src/world/playfield.cpp
    src/world/habitat_mesh.cpp
    src/world/habitats/airlock.cpp
    src/world/habitats/oxygen_dome.cpp
    src/world/habitats/cantine_dome.cpp
    src/world/habitats/bio_dome.cpp
    src/world/habitats/crew_quarters.cpp

    src/world/corridor_network.cpp
    src/world/corridor/geometry.cpp
    src/world/corridor/fittings.cpp
    src/world/corridor/entrances.cpp
    src/world/corridor/routing.cpp
    src/world/corridor/factory.cpp
)
target_include_directories(game PRIVATE include)
target_link_libraries(game PRIVATE game_engine)
target_compile_definitions(game PRIVATE GAME_VERSION="${PROJECT_VERSION}")
target_compile_options(game PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(game PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
