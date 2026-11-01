#include "game/game_app.hpp"
#include "game/world/gate_area.hpp"
namespace game {
bool GameApp::gate_route_clear(const std::vector<eng::Vec3>& path,float margin) const {
    std::vector<gate_area::Area> pads;
    for(std::size_t i=0;i<placed_habitats_.size();++i){auto a=gate_area::habitat(placed_habitats_[i],habitat_models_.mesh(placed_habitats_[i]),i,corridor_network_.connections());pads.insert(pads.end(),a.begin(),a.end());}
    for(std::size_t i=0;i<placed_resources_.size();++i){auto a=gate_area::storage(placed_resources_[i],resource_models_.mesh(placed_resources_[i]),i,corridor_network_.connections());pads.insert(pads.end(),a.begin(),a.end());}
    for(const auto& pad:pads)for(std::size_t j=1;j<path.size();++j)if(pad.hits(path[j-1],path[j],margin))return false;
    return true;
}
bool GameApp::corridor_gate_layout_clear(const CorridorConnection& connection) const {
    if(!gate_route_clear(connection.path,geometry::kCorridorHalfWidth+0.2f))return false;
    for(const auto* e:{&connection.endpoint_a,&connection.endpoint_b}){
        if(e->kind!=CorridorEndpointKind::Habitat && e->kind!=CorridorEndpointKind::Resource)continue;
        const auto pad=gate_area::beside(e->door.wall_position,e->door.normal);const auto box=pad.bounds();
        const float half=playfield_.size()*0.5f;
        const float build_limit=half-geometry::kPlayfieldRobotPerimeterLane;
        for(float x:{-box.half_x,box.half_x})for(float z:{-box.half_z,box.half_z})if(std::abs(box.center.x+box.axis_x.x*x+box.axis_z.x*z)>build_limit || std::abs(box.center.y+box.axis_x.y*x+box.axis_z.y*z)>build_limit)return false;
        for(std::size_t i=0;i<placed_habitats_.size();++i){if(e->kind==CorridorEndpointKind::Habitat && e->index==i)continue;
            const auto& h=placed_habitats_[i];const auto& m=habitat_models_.mesh(h);if(world_math::overlaps(box,world_math::footprint(h.position,h.rotation_degrees,m.footprint_x,m.footprint_z),0.2f))return false;}
        for(std::size_t i=0;i<placed_resources_.size();++i){if(e->kind==CorridorEndpointKind::Resource && e->index==i)continue;
            const auto& r=placed_resources_[i];const auto& m=resource_models_.mesh(r);if(world_math::overlaps(box,world_math::footprint(r.position,r.rotation_degrees,m.footprint_x,m.footprint_z),0.2f))return false;}
        for(const auto& c:corridor_network_.connections())for(std::size_t j=1;j<c.path.size();++j)if(pad.hits(c.path[j-1],c.path[j],geometry::kCorridorHalfWidth+0.2f))return false;
        for(std::size_t j=1;j<connection.path.size();++j)if(pad.hits(connection.path[j-1],connection.path[j],geometry::kCorridorHalfWidth+0.2f))return false;
        for(const auto& u:utility_network_.connections())for(std::size_t j=1;j<u.path.size();++j)if(pad.hits(u.path[j-1],u.path[j],geometry::kSupplyTrayWidth*0.5f+0.2f))return false;
        for(const auto& w:walkway_network_.connections())for(std::size_t j=1;j<w.path.size();++j)if(pad.hits(w.path[j-1],w.path[j],geometry::kWalkwayHalfWidth+0.2f))return false;
    }
    return true;
}
bool GameApp::gate_layout_clear(const PlacedHabitat* habitat,const PlacedResource* resource) const {
    const auto& corridors=corridor_network_.connections();std::vector<gate_area::Area> proposed;
    world_math::OrientedFootprint body;
    if(habitat){const auto& m=habitat_models_.mesh(*habitat);body=world_math::footprint(habitat->position,habitat->rotation_degrees,m.footprint_x,m.footprint_z);proposed=gate_area::habitat(*habitat,m,kNoRobotIndex,{});}
    else if(resource){const auto& m=resource_models_.mesh(*resource);body=world_math::footprint(resource->position,resource->rotation_degrees,m.footprint_x,m.footprint_z);proposed=gate_area::storage(*resource,m,kNoRobotIndex,{});}
    else return false;
    std::vector<gate_area::Area> existing;
    for(std::size_t i=0;i<placed_habitats_.size();++i){const auto& h=placed_habitats_[i];const auto& m=habitat_models_.mesh(h);
        auto areas=gate_area::habitat(h,m,i,corridors);existing.insert(existing.end(),areas.begin(),areas.end());
        for(const auto& a:proposed)if(world_math::overlaps(a.bounds(),world_math::footprint(h.position,h.rotation_degrees,m.footprint_x,m.footprint_z),0.2f))return false;
    }
    for(std::size_t i=0;i<placed_resources_.size();++i){const auto& r=placed_resources_[i];const auto& m=resource_models_.mesh(r);
        auto areas=gate_area::storage(r,m,i,corridors);existing.insert(existing.end(),areas.begin(),areas.end());
        for(const auto& a:proposed)if(world_math::overlaps(a.bounds(),world_math::footprint(r.position,r.rotation_degrees,m.footprint_x,m.footprint_z),0.2f))return false;
    }
    for(const auto& a:existing){if(world_math::overlaps(body,a.bounds(),0.2f))return false;for(const auto& b:proposed)if(world_math::overlaps(a.bounds(),b.bounds(),0.2f))return false;}
    const float half=playfield_.size()*0.5f;
    const float build_limit=half-geometry::kPlayfieldRobotPerimeterLane;
    for(const auto& a:proposed){
        const auto box=a.bounds();for(float x:{-box.half_x,box.half_x})for(float z:{-box.half_z,box.half_z})if(std::abs(box.center.x+box.axis_x.x*x+box.axis_z.x*z)>build_limit || std::abs(box.center.y+box.axis_x.y*x+box.axis_z.y*z)>build_limit)return false;
        for(const auto& c:corridors)for(std::size_t j=1;j<c.path.size();++j)if(a.hits(c.path[j-1],c.path[j],geometry::kCorridorHalfWidth+0.2f))return false;
        for(const auto& u:utility_network_.connections())for(std::size_t j=1;j<u.path.size();++j)if(a.hits(u.path[j-1],u.path[j],geometry::kSupplyTrayWidth*0.5f+0.2f))return false;
        for(const auto& w:walkway_network_.connections())for(std::size_t j=1;j<w.path.size();++j)if(a.hits(w.path[j-1],w.path[j],geometry::kWalkwayHalfWidth+0.2f))return false;
        for(const auto& pile:ground_resource_piles_)if(a.hits(pile.position,pile.position,pile.radius+0.2f))return false;
    }
    return true;
}
void GameApp::render_gate_areas(eng::Renderer& r) const {
    // Clearances stay authoritative, but unused door candidates are not world
    // placements. Show only real cargo pads for the selected dome/gate.
    if(selection_kind_!=SelectionKind::Resource || selected_resource_<0 || (std::size_t)selected_resource_>=placed_resources_.size())return;
    const auto& selected=placed_resources_[(std::size_t)selected_resource_];
    const auto index=selected.kind==ResourceKind::StorageGate?selected.storage_parent_index:(std::size_t)selected_resource_;
    if(index>=placed_resources_.size())return;
    const auto& owner=placed_resources_[index];if(owner.kind!=ResourceKind::StorageDome)return;
    for(int f:storage_loading::faces(owner))if(f>=0 && (selected.kind!=ResourceKind::StorageGate || selected.storage_loading_face==f))
        gate_area::draw(r,{storage_loading::service(owner,f),storage_loading::normal(owner,f),1.0f,0.75f});
}
}
