#include "game/game_app.hpp"
#include "game/world/interior_station.hpp"
#include "game/util/world_math.hpp"
namespace game {
const HabitatMesh& GameApp::habitat_runtime_mesh(std::size_t index) const {
    const auto& habitat=placed_habitats_[index];const auto& source=habitat_models_.mesh(habitat);
    std::vector<interior_layout::Reservation> reserved;
    for(const auto& corridor:corridor_network_.connections())for(const auto* endpoint:{&corridor.endpoint_a,&corridor.endpoint_b}){
        if(endpoint->kind!=CorridorEndpointKind::Habitat||endpoint->index!=index)continue;
        auto wall=world_math::rotate_y(endpoint->door.wall_position-habitat.position,-habitat.rotation_degrees);wall.y=source.walkable_floor_y;
        auto normal=world_math::rotate_y(endpoint->door.normal,-habitat.rotation_degrees);normal.y=0;
        if(game::runtime::length(normal)<.01f)continue;
        reserved.push_back({wall,game::runtime::normalize(normal)});
    }
    if(habitat_layout_cache_.size()<placed_habitats_.size())habitat_layout_cache_.resize(placed_habitats_.size());
    return interior_layout::resolve(habitat_layout_cache_[index],source,std::move(reserved),hydroponics_bay_count(habitat.size));
}
const ResourceMesh& GameApp::resource_runtime_mesh(std::size_t index) const {
    const auto& item=placed_resources_[index];const auto& source=resource_models_.mesh(item);
    if(source.stations.empty())return source;
    std::vector<interior_layout::Reservation> reserved;
    for(const auto& corridor:corridor_network_.connections())for(const auto* e:{&corridor.endpoint_a,&corridor.endpoint_b}){
        if(e->kind!=CorridorEndpointKind::Resource||e->index!=index)continue;
        auto wall=world_math::rotate_y(e->door.wall_position-item.position,-item.rotation_degrees);wall.y=source.walkable_floor_y;
        auto normal=world_math::rotate_y(e->door.normal,-item.rotation_degrees);normal.y=0;
        if(game::runtime::length(normal)>.01f)reserved.push_back({wall,game::runtime::normalize(normal)});
    }
    if(resource_layout_cache_.size()<placed_resources_.size())resource_layout_cache_.resize(placed_resources_.size());
    return interior_layout::resolve(resource_layout_cache_[index],source,std::move(reserved));
}

}
