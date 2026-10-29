include(cmake/GameClusters.cmake)

add_executable(game src/main.cpp)
target_include_directories(game PRIVATE include)
target_link_libraries(game PRIVATE game_engine Threads::Threads)
target_compile_definitions(game PRIVATE GAME_VERSION="${GAME_VERSION_FULL}")
target_compile_options(game PRIVATE -Wall -Wextra -Wpedantic)

foreach(cluster IN LISTS GAME_CLUSTER_TARGETS)
    target_sources(game PRIVATE $<TARGET_OBJECTS:${cluster}>)
endforeach()

set_target_properties(game PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
