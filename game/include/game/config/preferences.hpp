#pragma once
#include "engine/graphics_settings.hpp"

namespace game {
struct Preferences {
    eng::GraphicsSettings graphics;
    bool mesh_layout{};
    bool music_enabled{true};
};

Preferences load_preferences();
bool save_preferences(const Preferences& preferences);
} // namespace game
