#pragma once
#include "game/runtime/graphics_preferences.hpp"

namespace game {
struct Preferences {
    game::runtime::GraphicsSettings graphics;
    bool mesh_layout{};
    bool music_enabled{true};
    int first_person_fov{68};
};

Preferences load_preferences();
bool save_preferences(const Preferences& preferences);
} // namespace game
