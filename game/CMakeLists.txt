include(cmake/GameClusters.cmake)

add_executable(game src/main.cpp)
target_include_directories(game PRIVATE include)
target_link_libraries(game PRIVATE game_engine Threads::Threads rvo2)
target_compile_definitions(game PRIVATE GAME_VERSION="${GAME_VERSION_FULL}")
target_compile_options(game PRIVATE -Wall -Wextra -Wpedantic)

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
    add_test(NAME navigation_fleet COMMAND navigation_regression)
    add_test(NAME navigation_geometry COMMAND navigation_regression geometry)
    add_test(NAME navigation_construction COMMAND navigation_regression construction "${CMAKE_CURRENT_SOURCE_DIR}/debug-templates/0-40-32-debug-start.gmsave")
    add_test(NAME navigation_late_corridor COMMAND navigation_regression late "${CMAKE_CURRENT_SOURCE_DIR}/debug-templates/0-40-32-debug-start.gmsave")
    set_tests_properties(navigation_construction PROPERTIES TIMEOUT 300)
endif()

set_target_properties(game PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Debug-only new-game construction template. Keep it next to the executable so
# the runtime can locate it regardless of the current working directory.
set(DEBUG_BLUEPRINT_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/debug-templates/0-40-32-debug-start.gmsave")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/bin/debug")
configure_file("${DEBUG_BLUEPRINT_TEMPLATE}"
               "${CMAKE_BINARY_DIR}/bin/debug/0-40-32-debug-start.gmsave"
               COPYONLY)
