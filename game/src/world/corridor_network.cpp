#include "game/world/corridor_network.hpp"
#include "game/world/corridor/detail.hpp"
#include <limits>

namespace game {
using namespace eng;
using namespace corridor_detail;

void CorridorNetwork::connect_new_habitat(std::size_t new_index,
                                          const std::vector<PlacedHabitat>& habitats,
                                          const HabitatModelLibrary& library){
    if(auto c=preview_connection(new_index,habitats,library)) connections_.push_back(std::move(*c));
}

std::optional<CorridorConnection> CorridorNetwork::preview_connection(
    std::size_t new_index,const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library) const {
    if(new_index>=habitats.size()) return std::nullopt;

    const Vec3 newcomer=habitats[new_index].position;
    float best=std::numeric_limits<float>::max();
    CorridorBuildAnchor target{};
    bool found=false;

    // Existing habitats remain valid automatic targets.
    for(std::size_t i=0;i<habitats.size();++i){
        if(i==new_index) continue;
        const float d=distance_xz(newcomer,habitats[i].position);
        if(d<best){
            best=d; found=true;
            target={CorridorEndpointKind::Habitat,i,habitats[i].position};
        }
    }

    // Existing corridors are also first-class targets. A newly placed habitat
    // therefore joins the nearest part of the colony network, not necessarily
    // the nearest building.
    for(std::size_t i=0;i<connections_.size();++i){
        const Vec3 p=nearest_path_point(connections_[i].path,newcomer);
        const float d=distance_xz(newcomer,p);
        if(d<best){
            best=d; found=true;
            target={CorridorEndpointKind::Corridor,i,p};
        }
    }

    if(!found) return std::nullopt;
    CorridorBuildAnchor source{CorridorEndpointKind::Habitat,new_index,newcomer};
    return make_generic(source,target,CorridorConnectionKind::Automatic,habitats,library,connections_);
}

std::optional<CorridorEndpoint> CorridorNetwork::preview_anchor(
    const CorridorBuildAnchor& anchor,Vec3 other,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library) const {
    if(anchor.kind==CorridorEndpointKind::Corridor && anchor.index>=connections_.size()) return std::nullopt;
    if(anchor.kind==CorridorEndpointKind::Habitat && anchor.index>=habitats.size()) return std::nullopt;
    return resolve_anchor(anchor,other,habitats,library,connections_);
}

std::optional<CorridorConnection> CorridorNetwork::preview_built_corridor(
    const CorridorBuildAnchor& a,const CorridorBuildAnchor& b,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library) const {
    if(a.kind==CorridorEndpointKind::Corridor && a.index>=connections_.size()) return std::nullopt;
    if(b.kind==CorridorEndpointKind::Corridor && b.index>=connections_.size()) return std::nullopt;
    if(a.kind==CorridorEndpointKind::Habitat && a.index>=habitats.size()) return std::nullopt;
    if(b.kind==CorridorEndpointKind::Habitat && b.index>=habitats.size()) return std::nullopt;
    if(a.kind==b.kind && a.index==b.index && a.kind!=CorridorEndpointKind::OpenDoor) return std::nullopt;
    return make_generic(a,b,CorridorConnectionKind::BuiltSection,habitats,library,connections_);
}

bool CorridorNetwork::build_corridor(const CorridorBuildAnchor& a,const CorridorBuildAnchor& b,
                                     const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library){
    auto c=preview_built_corridor(a,b,habitats,library);
    if(!c) return false;
    connections_.push_back(std::move(*c));
    return true;
}

std::optional<CorridorConnection> CorridorNetwork::preview_corridor_link(
    std::size_t corridor_a,std::size_t corridor_b,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library) const {
    if(corridor_a>=connections_.size() || corridor_b>=connections_.size() || corridor_a==corridor_b) return std::nullopt;
    const auto pair=closest_pair(connections_[corridor_a].path,connections_[corridor_b].path);
    if(distance_xz(pair.first,pair.second)<CORRIDOR_HALF_WIDTH*1.25f) return std::nullopt;
    CorridorBuildAnchor a{CorridorEndpointKind::Corridor,corridor_a,pair.first};
    CorridorBuildAnchor b{CorridorEndpointKind::Corridor,corridor_b,pair.second};
    auto c=make_generic(a,b,CorridorConnectionKind::NetworkLink,habitats,library,connections_);
    return c;
}

bool CorridorNetwork::connect_corridors(std::size_t corridor_a,std::size_t corridor_b,
                                        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library){
    auto c=preview_corridor_link(corridor_a,corridor_b,habitats,library);
    if(!c) return false;
    connections_.push_back(std::move(*c));
    return true;
}

Vec3 CorridorNetwork::nearest_point_on_corridor(std::size_t corridor_index,Vec3 p) const {
    if(corridor_index>=connections_.size()) return p;
    return nearest_path_point(connections_[corridor_index].path,p);
}

CorridorBuildAnchor CorridorNetwork::corridor_anchor(std::size_t corridor_index,Vec3 near_point) const {
    return {CorridorEndpointKind::Corridor,corridor_index,nearest_point_on_corridor(corridor_index,near_point)};
}

std::size_t CorridorNetwork::habitat_connection_count(std::size_t habitat_index) const {
    std::size_t count=0;
    for(const auto& c:connections_){
        if(endpoint_uses_habitat(c.endpoint_a,habitat_index) || endpoint_uses_habitat(c.endpoint_b,habitat_index)) ++count;
    }
    return count;
}

float CorridorNetwork::connection_length(std::size_t corridor_index) const {
    if(corridor_index>=connections_.size()) return 0.0f;
    return path_length(connections_[corridor_index].path);
}


} // namespace game
