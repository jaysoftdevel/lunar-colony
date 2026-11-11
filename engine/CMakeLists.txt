if(WIN32)
    add_library(game_engine STATIC
        src/platform_win32.cpp
        src/renderer_d3d12.cpp
        src/ui.cpp
        src/task_pool.cpp
        src/audio_openal.cpp
    )
else()
    find_package(X11 REQUIRED)
    add_library(game_engine STATIC
        src/platform_x11.cpp
        src/renderer_gl.cpp
        src/lunar_renderer.cpp
        src/lunar_bvh.cpp
        src/lunar_geometry.cpp
        src/lunar_pages.cpp
        src/ui.cpp
        src/task_pool.cpp
        src/audio_openal.cpp
    )
endif()
target_sources(game_engine PRIVATE src/cpu_affinity.cpp src/graphics_settings.cpp)
target_include_directories(game_engine PUBLIC include)
target_compile_definitions(game_engine PUBLIC
    GAME_PLATFORM_NAME="${GAME_PLATFORM_NAME}"
    GAME_RENDER_BACKEND="${GAME_RENDER_BACKEND}"
)
if(WIN32)
    target_compile_definitions(game_engine PRIVATE UNICODE _UNICODE _WIN32_WINNT=0x0A00 WINVER=0x0A00)
    target_link_libraries(game_engine PUBLIC Threads::Threads d3d12 dxgi d3dcompiler user32 gdi32 shell32)
    if(MSVC)
        target_compile_options(game_engine PRIVATE /W4 /permissive- /EHsc)
    endif()
else()
    target_link_libraries(game_engine PUBLIC X11::X11 dl Threads::Threads)
    target_compile_options(game_engine PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(GAME_BUILD_NAVIGATION_TESTS)
    add_executable(task_pool_regression tests/task_pool_regression.cpp)
    target_link_libraries(task_pool_regression PRIVATE game_engine)
    add_test(NAME engine_task_pool COMMAND task_pool_regression)
    set_tests_properties(engine_task_pool PROPERTIES TIMEOUT 30)
endif()
