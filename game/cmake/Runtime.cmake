# Game-specific bindings, presentation and preferences. JSD owns the reusable
# renderer, platform, math types, worker implementation, audio mixer and device.
add_library(colony_runtime STATIC
    src/runtime/platform_jsd.cpp
    src/runtime/renderer_jsd.cpp
    src/runtime/audio_jsd.cpp
    src/runtime/jobs.cpp
    src/runtime/graphics_settings.cpp
    src/ui/widgets.cpp
)
target_include_directories(colony_runtime PUBLIC include)
target_link_libraries(colony_runtime PUBLIC JSD::GameEngineStatic Threads::Threads)
target_compile_definitions(colony_runtime PUBLIC
    GAME_PLATFORM_NAME="${GAME_PLATFORM_NAME}"
    GAME_RENDER_BACKEND="${GAME_RENDER_BACKEND}"
)
if(MSVC)
    target_compile_options(colony_runtime PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(colony_runtime PRIVATE -Wall -Wextra -Wpedantic)
endif()
if(GAME_BUILD_NAVIGATION_TESTS)
    add_executable(jsd_bridge_regression tests/jsd_bridge_regression.cpp)
    target_include_directories(jsd_bridge_regression PRIVATE src/runtime)
    target_link_libraries(jsd_bridge_regression PRIVATE colony_runtime)
    add_test(NAME jsd_camera_material_contract COMMAND jsd_bridge_regression)
    add_executable(jsd_runtime_regression tests/jsd_runtime_regression.cpp)
    target_link_libraries(jsd_runtime_regression PRIVATE colony_runtime)
    add_test(NAME jsd_runtime_services COMMAND jsd_runtime_regression)
    set_tests_properties(jsd_runtime_services PROPERTIES TIMEOUT 30 ENVIRONMENT "SDL_AUDIODRIVER=dummy")
    option(GAME_BUILD_RENDER_TESTS "Enable GPU/EGL rendering integration tests" OFF)
    if(GAME_BUILD_RENDER_TESTS)
        add_test(NAME jsd_renderer_integration COMMAND jsd_bridge_regression --render)
        set_tests_properties(jsd_renderer_integration PROPERTIES TIMEOUT 180)
    endif()
    if(UNIX)
        add_test(NAME jsd_build_arguments COMMAND "${JSD_GUARD_PYTHON}" -B "${CMAKE_CURRENT_SOURCE_DIR}/tests/build_arguments_test.py")
    endif()
endif()
