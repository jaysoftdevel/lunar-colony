#include "game/world/corridor_network.hpp"
#include "game/world/corridor/detail.hpp"
#include <limits>

namespace game {
using namespace eng;
using namespace corridor_detail;


namespace {
constexpr float CLOSE_END_CONNECTION_RADIUS=1.35f;
constexpr float CLOSE_END_SPLICE_MAX_GAP=2.70f;

struct OpenEndMatch {
    bool found{false};
    Vec3 position{};
    Vec3 outward{1,0,0};
};

OpenEndMatch close_open_end(const CorridorConnection& host,Vec3 point){
    if(host.path.size()<2) return {};
    OpenEndMatch best{};
    float best_d=std::numeric_limits<float>::max();
    if(host.endpoint_a.kind==CorridorEndpointKind::OpenDoor && !host.endpoint_a.suppress_open_door){
        const float d=distance_xz(point,host.path.front());
        if(d<=CLOSE_END_CONNECTION_RADIUS && d<best_d){
            best_d=d; best.found=true; best.position=host.path.front();
            best.outward=flat_normal(host.path.front()-host.path[1]);
        }
    }
    if(host.endpoint_b.kind==CorridorEndpointKind::OpenDoor && !host.endpoint_b.suppress_open_door){
        const float d=distance_xz(point,host.path.back());
        if(d<=CLOSE_END_CONNECTION_RADIUS && d<best_d){
            best_d=d; best.found=true; best.position=host.path.back();
            best.outward=flat_normal(host.path.back()-host.path[host.path.size()-2]);
        }
    }
    return best;
}

bool consume_close_open_end(CorridorConnection& host,Vec3 point){
    if(host.path.size()<2) return false;
    bool changed=false;
    if(host.endpoint_a.kind==CorridorEndpointKind::OpenDoor &&
       distance_xz(point,host.path.front())<=CLOSE_END_CONNECTION_RADIUS){
        host.endpoint_a.suppress_open_door=true;
        changed=true;
    }
    if(host.endpoint_b.kind==CorridorEndpointKind::OpenDoor &&
       distance_xz(point,host.path.back())<=CLOSE_END_CONNECTION_RADIUS){
        host.endpoint_b.suppress_open_door=true;
        changed=true;
    }
    return changed;
}

void insert_shared_path_vertex(CorridorConnection& host,Vec3 point){
    if(host.path.size()<2) return;
    consume_close_open_end(host,point);
    for(const Vec3& p:host.path) if(distance_xz(p,point)<0.015f && std::abs(p.y-point.y)<0.05f) return;
    float best=std::numeric_limits<float>::max();
    std::size_t best_i=0;
    Vec3 snapped=point;
    for(std::size_t i=0;i+1<host.path.size();++i){
        const Vec3 q=closest_point_segment_xz(point,host.path[i],host.path[i+1]);
        const float d=distance_xz(q,point);
        if(d<best){ best=d; best_i=i; snapped=q; }
    }
    host.path.insert(host.path.begin()+(std::ptrdiff_t)best_i+1,snapped);
}



}

std::optional<CorridorEndpoint> CorridorNetwork::preview_anchor(
    const CorridorBuildAnchor& anchor,Vec3 other,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library) const {
    if(anchor.kind==CorridorEndpointKind::Corridor && anchor.index>=connections_.size()) return std::nullopt;
    if(anchor.kind==CorridorEndpointKind::Habitat && anchor.index>=habitats.size()) return std::nullopt;
    return resolve_anchor(anchor,other,habitats,library,connections_);
}

CorridorMesh CorridorNetwork::preview_endpoint_mesh(const CorridorEndpoint& endpoint) const {
    CorridorMesh mesh;
    append_docking_collar(mesh,endpoint);
    return mesh;
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
    // Materialize the exact branch point into each host path. This gives
    // corridor-to-corridor connections a shared centerline vertex instead of
    // two nearly coincident segments that can leave a crack or overlap.
    for(const CorridorEndpoint* ep:{&c->endpoint_a,&c->endpoint_b}){
        if(ep->kind!=CorridorEndpointKind::Corridor || ep->index>=connections_.size()) continue;
        insert_shared_path_vertex(connections_[ep->index],
                                  ep->surface_matched_junction?ep->host_position:ep->position);
        connections_[ep->index].mesh=build_mesh(connections_[ep->index].endpoint_a,connections_[ep->index].endpoint_b,
                                                connections_[ep->index].path,connections_);
    }
    connections_.push_back(std::move(*c));
    optimize_shared_entrance_groups(connections_,habitats,library);
    return true;
}

std::optional<CorridorConnection> CorridorNetwork::preview_corridor_link(
    std::size_t corridor_a,std::size_t corridor_b,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library) const {
    if(corridor_a>=connections_.size() || corridor_b>=connections_.size() || corridor_a==corridor_b) return std::nullopt;
    const auto pair=closest_pair(connections_[corridor_a].path,connections_[corridor_b].path);
    // Close corridor ends are valid direct-link candidates. Older builds
    // rejected distances below roughly one corridor radius, leaving the two
    // sealed end doors facing each other. Let the normal link path bridge that
    // small gap; insert_shared_path_vertex() consumes any nearby open end door
    // on both host corridors so the pressure shells connect directly.
    if(distance_xz(pair.first,pair.second)<0.08f) return std::nullopt;
    // If both selected points are close to still-open corridor ends, splice the
    // pressure tubes end-to-end.  Use the real host end positions and terminal
    // tangents so the connector's first/last cross-sections are identical to
    // the host cross-sections.  No entrance block or junction fitting is added.
    const OpenEndMatch end_a=close_open_end(connections_[corridor_a],pair.first);
    const OpenEndMatch end_b=close_open_end(connections_[corridor_b],pair.second);
    if(end_a.found && end_b.found && distance_xz(end_a.position,end_b.position)<=CLOSE_END_SPLICE_MAX_GAP){
        auto path=seamless_end_splice_route(end_a.position,end_a.outward,end_b.position,end_b.outward);
        if(path.size()<2) return std::nullopt;

        CorridorConnection out;
        out.kind=CorridorConnectionKind::NetworkLink;
        out.endpoint_a.kind=CorridorEndpointKind::Corridor;
        out.endpoint_a.index=corridor_a;
        out.endpoint_a.position=end_a.position;
        out.endpoint_a.tangent=end_a.outward;
        out.endpoint_a.host_tangent=end_a.outward;
        out.endpoint_a.seamless_splice=true;
        out.endpoint_b.kind=CorridorEndpointKind::Corridor;
        out.endpoint_b.index=corridor_b;
        out.endpoint_b.position=end_b.position;
        out.endpoint_b.tangent=end_b.outward*-1.0f;
        out.endpoint_b.host_tangent=end_b.outward*-1.0f;
        out.endpoint_b.seamless_splice=true;
        out.path=std::move(path);
        out.mesh=build_mesh(out.endpoint_a,out.endpoint_b,out.path,connections_);
        return out;
    }

    CorridorBuildAnchor a{CorridorEndpointKind::Corridor,corridor_a,pair.first};
    CorridorBuildAnchor b{CorridorEndpointKind::Corridor,corridor_b,pair.second};
    return make_generic(a,b,CorridorConnectionKind::NetworkLink,habitats,library,connections_);
}

bool CorridorNetwork::connect_corridors(std::size_t corridor_a,std::size_t corridor_b,
                                        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library){
    auto c=preview_corridor_link(corridor_a,corridor_b,habitats,library);
    if(!c) return false;
    for(const CorridorEndpoint* ep:{&c->endpoint_a,&c->endpoint_b}){
        if(ep->kind!=CorridorEndpointKind::Corridor || ep->index>=connections_.size()) continue;
        insert_shared_path_vertex(connections_[ep->index],
                                  ep->surface_matched_junction?ep->host_position:ep->position);
        connections_[ep->index].mesh=build_mesh(connections_[ep->index].endpoint_a,connections_[ep->index].endpoint_b,
                                                connections_[ep->index].path,connections_);
    }
    connections_.push_back(std::move(*c));
    return true;
}

void CorridorNetwork::update_construction(float dt){
    for(auto& connection:connections_) connection.construction.update(dt);
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

bool CorridorNetwork::habitat_entrance_connected(std::size_t habitat_index,std::size_t entrance_index) const {
    for(const auto& c:connections_){
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(!endpoint_uses_habitat(*ep,habitat_index)) continue;
            if(!ep->door.generated && ep->door.entrance_index==(int)entrance_index) return true;
        }
    }
    return false;
}

float CorridorNetwork::connection_length(std::size_t corridor_index) const {
    if(corridor_index>=connections_.size()) return 0.0f;
    return path_length(connections_[corridor_index].path);
}



void CorridorNetwork::remove_corridor(std::size_t corridor_index){
    if(corridor_index>=connections_.size()) return;

    std::vector<bool> removed(connections_.size(),false);
    removed[corridor_index]=true;

    // Network links can depend on another corridor by index. Remove those
    // dependants recursively so no dangling junction endpoint survives.
    bool changed=true;
    while(changed){
        changed=false;
        for(std::size_t i=0;i<connections_.size();++i){
            if(removed[i]) continue;
            for(const CorridorEndpoint* ep:{&connections_[i].endpoint_a,&connections_[i].endpoint_b}){
                if(ep->kind==CorridorEndpointKind::Corridor && ep->index<removed.size() && removed[ep->index]){
                    removed[i]=true;
                    changed=true;
                    break;
                }
            }
        }
    }

    const std::size_t invalid=std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> remap(connections_.size(),invalid);
    std::vector<CorridorConnection> kept;
    kept.reserve(connections_.size());
    for(std::size_t i=0;i<connections_.size();++i){
        if(removed[i]) continue;
        remap[i]=kept.size();
        kept.push_back(std::move(connections_[i]));
    }
    for(auto& c:kept){
        for(CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(ep->kind==CorridorEndpointKind::Corridor && ep->index<remap.size()) ep->index=remap[ep->index];
        }
    }
    connections_=std::move(kept);
    for(auto& c:connections_) c.mesh=build_mesh(c.endpoint_a,c.endpoint_b,c.path,connections_);
}

void CorridorNetwork::remove_habitat(std::size_t habitat_index){
    for(auto& connection:connections_){
        for(CorridorEndpoint* endpoint:{&connection.endpoint_a,&connection.endpoint_b}){
            if(endpoint->kind!=CorridorEndpointKind::Habitat) continue;
            if(endpoint->index==habitat_index){
                // Preserve the corridor in place. Once its habitat disappears,
                // the former docking end becomes a sealed free corridor end at
                // the already-built terminal ring rather than deleting the run.
                endpoint->kind=CorridorEndpointKind::OpenDoor;
                endpoint->index=0;
                endpoint->door=CorridorDoor{};
                endpoint->suppress_open_door=false;
                endpoint->seamless_splice=false;
                endpoint->surface_matched_junction=false;
                endpoint->host_tangent={0,0,0};
                endpoint->host_position={};
            }else if(endpoint->index>habitat_index){
                --endpoint->index;
            }
        }
    }
    for(auto& connection:connections_)
        connection.mesh=build_mesh(connection.endpoint_a,connection.endpoint_b,connection.path,connections_);
}

} // namespace game
