#pragma once
#include "game/runtime/graphics_preferences.hpp"

namespace game {
struct Preferences {
    game::runtime::GraphicsSettings graphics;
    bool mesh_layout{};
    bool music_enabled{true};
    int first_person_fov{68};
    int autosave_minutes{10}; // 0 disables; otherwise real active-play minutes.
    bool show_fps{};
    int ui_scale_index{};
    bool faded_hulls{};
    int sound_volume_percent{72};
    bool debug_frame_enabled{true};
    int display_width{},display_height{};
};

Preferences load_preferences();
bool save_preferences(const Preferences& preferences);
} // namespace game
