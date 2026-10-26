add_executable(game
    src/main.cpp
    src/game_app.cpp
    src/world/playfield.cpp
    src/systems/camera_controller.cpp
    src/systems/colony_status.cpp
    src/features/build_catalog.cpp
)
target_include_directories(game PRIVATE include)
target_link_libraries(game PRIVATE game_engine)
target_compile_definitions(game PRIVATE GAME_VERSION="${PROJECT_VERSION}")
target_compile_options(game PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(game PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
