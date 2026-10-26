find_package(X11 REQUIRED)
add_library(game_engine STATIC
    src/platform_x11.cpp
    src/renderer_gl.cpp
    src/ui.cpp
)
target_include_directories(game_engine PUBLIC include)
target_link_libraries(game_engine PUBLIC X11::X11 dl)
target_compile_options(game_engine PRIVATE -Wall -Wextra -Wpedantic)
