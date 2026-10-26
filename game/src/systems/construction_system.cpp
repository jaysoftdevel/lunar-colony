#include "game/game_app.hpp"

namespace game {

void GameApp::update_construction(float dt){
    for(auto& habitat:placed_habitats_) habitat.construction.update(dt);
    for(auto& resource:placed_resources_) resource.construction.update(dt);
    corridor_network_.update_construction(dt);
    utility_network_.update_construction(dt);
}

} // namespace game
