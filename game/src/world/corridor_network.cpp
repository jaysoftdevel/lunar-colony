#include "game/world/corridor_network.hpp"
#include "game/systems/construction_costs.hpp"
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


std::size_t network_junction_count(const CorridorConnection& c){
    std::size_t count=0;
    if(c.endpoint_a.kind==CorridorEndpointKind::Corridor) ++count;
    if(c.endpoint_b.kind==CorridorEndpointKind::Corridor) ++count;
    return count;
}



struct DisjointSets {
    std::vector<std::size_t> parent;
    explicit DisjointSets(std::size_t n):parent(n){ for(std::size_t i=0;i<n;++i) parent[i]=i; }
    std::size_t find(std::size_t x){ while(parent[x]!=x){ parent[x]=parent[parent[x]]; x=parent[x]; } return x; }
    void unite(std::size_t a,std::size_t b){ a=find(a); b=find(b); if(a!=b) parent[b]=a; }
};

bool endpoint_matches_anchor(const CorridorEndpoint& ep,const CorridorBuildAnchor& anchor){
    if(anchor.kind==CorridorEndpointKind::Habitat)
        return ep.kind==CorridorEndpointKind::Habitat && ep.index==anchor.index;
    if(anchor.kind==CorridorEndpointKind::Resource)
        return ep.kind==CorridorEndpointKind::Resource && ep.index==anchor.index;
    return false;
}

bool endpoint_same_owner(const CorridorEndpoint& a,const CorridorEndpoint& b){
    if(a.kind!=b.kind) return false;
    if(a.kind!=CorridorEndpointKind::Habitat && a.kind!=CorridorEndpointKind::Resource) return false;
    return a.index==b.index;
}

bool paths_have_merged_crossing(const CorridorConnection& a,const CorridorConnection& b){
    if(a.path.size()<2 || b.path.size()<2) return false;
    float aminx=a.path.front().x,amaxx=aminx,aminz=a.path.front().z,amaxz=aminz;
    float bminx=b.path.front().x,bmaxx=bminx,bminz=b.path.front().z,bmaxz=bminz;
    for(const Vec3 p:a.path){ aminx=std::min(aminx,p.x); amaxx=std::max(amaxx,p.x); aminz=std::min(aminz,p.z); amaxz=std::max(amaxz,p.z); }
    for(const Vec3 p:b.path){ bminx=std::min(bminx,p.x); bmaxx=std::max(bmaxx,p.x); bminz=std::min(bminz,p.z); bmaxz=std::max(bmaxz,p.z); }
    if(amaxx<bminx || bmaxx<aminx || amaxz<bminz || bmaxz<aminz) return false;
    for(std::size_t i=0;i+1<a.path.size();++i) for(std::size_t j=0;j+1<b.path.size();++j){
        Vec3 hit{};
        if(mergeable_corridor_crossing(a.path[i],a.path[i+1],b.path[j],b.path[j+1],hit)) return true;
    }
    return false;
}

bool corridor_anchors_already_connected(const CorridorBuildAnchor& a,const CorridorBuildAnchor& b,
                                        const std::vector<CorridorConnection>& connections){
    if(a.kind==b.kind && a.index==b.index &&
       (a.kind==CorridorEndpointKind::Habitat || a.kind==CorridorEndpointKind::Resource))
        return true;
    if(connections.empty()) return false;

    DisjointSets sets(connections.size());
    for(std::size_t i=0;i<connections.size();++i){
        for(const CorridorEndpoint* ep:{&connections[i].endpoint_a,&connections[i].endpoint_b})
            if(ep->kind==CorridorEndpointKind::Corridor && ep->index<connections.size()) sets.unite(i,ep->index);
    }
    // Connections that enter the same structure are part of one traversable
    // network through that structure interior. Transversal corridor crossings
    // are also real merged junctions and therefore share a component.
    for(std::size_t i=0;i<connections.size();++i) for(std::size_t j=i+1;j<connections.size();++j){
        const bool same_owner=endpoint_same_owner(connections[i].endpoint_a,connections[j].endpoint_a) ||
                              endpoint_same_owner(connections[i].endpoint_a,connections[j].endpoint_b) ||
                              endpoint_same_owner(connections[i].endpoint_b,connections[j].endpoint_a) ||
                              endpoint_same_owner(connections[i].endpoint_b,connections[j].endpoint_b);
        if(same_owner || paths_have_merged_crossing(connections[i],connections[j])) sets.unite(i,j);
    }

    std::vector<std::size_t> comps_a,comps_b;
    auto append_unique=[&](std::vector<std::size_t>& dst,std::size_t c){ c=sets.find(c); if(std::find(dst.begin(),dst.end(),c)==dst.end()) dst.push_back(c); };
    if(a.kind==CorridorEndpointKind::Corridor && a.index<connections.size()) append_unique(comps_a,a.index);
    if(b.kind==CorridorEndpointKind::Corridor && b.index<connections.size()) append_unique(comps_b,b.index);
    for(std::size_t i=0;i<connections.size();++i){
        if(endpoint_matches_anchor(connections[i].endpoint_a,a) || endpoint_matches_anchor(connections[i].endpoint_b,a)) append_unique(comps_a,i);
        if(endpoint_matches_anchor(connections[i].endpoint_a,b) || endpoint_matches_anchor(connections[i].endpoint_b,b)) append_unique(comps_b,i);
    }
    for(std::size_t ca:comps_a) for(std::size_t cb:comps_b) if(ca==cb) return true;
    return false;
}

std::vector<std::size_t> corridor_component_indices(const CorridorBuildAnchor& anchor,
                                                     const std::vector<CorridorConnection>& connections){
    std::vector<std::size_t> result;
    if(connections.empty()) return result;

    DisjointSets sets(connections.size());
    for(std::size_t i=0;i<connections.size();++i){
        for(const CorridorEndpoint* ep:{&connections[i].endpoint_a,&connections[i].endpoint_b})
            if(ep->kind==CorridorEndpointKind::Corridor && ep->index<connections.size()) sets.unite(i,ep->index);
    }
    for(std::size_t i=0;i<connections.size();++i) for(std::size_t j=i+1;j<connections.size();++j){
        const bool same_owner=endpoint_same_owner(connections[i].endpoint_a,connections[j].endpoint_a) ||
                              endpoint_same_owner(connections[i].endpoint_a,connections[j].endpoint_b) ||
                              endpoint_same_owner(connections[i].endpoint_b,connections[j].endpoint_a) ||
                              endpoint_same_owner(connections[i].endpoint_b,connections[j].endpoint_b);
        if(same_owner || paths_have_merged_crossing(connections[i],connections[j])) sets.unite(i,j);
    }

    std::vector<std::size_t> roots;
    if(anchor.kind==CorridorEndpointKind::Corridor && anchor.index<connections.size())
        roots.push_back(sets.find(anchor.index));
    for(std::size_t i=0;i<connections.size();++i){
        if(endpoint_matches_anchor(connections[i].endpoint_a,anchor) || endpoint_matches_anchor(connections[i].endpoint_b,anchor)){
            const auto root=sets.find(i);
            if(std::find(roots.begin(),roots.end(),root)==roots.end()) roots.push_back(root);
        }
    }
    if(roots.empty()) return result;
    for(std::size_t i=0;i<connections.size();++i){
        const auto root=sets.find(i);
        if(std::find(roots.begin(),roots.end(),root)!=roots.end()) result.push_back(i);
    }
    return result;
}

Vec3 nearest_point_on_path(const CorridorConnection& c,Vec3 p){
    if(c.path.empty()) return p;
    Vec3 best=c.path.front();
    float best_d=distance_xz(best,p);
    for(std::size_t i=0;i+1<c.path.size();++i){
        const Vec3 q=closest_point_segment_xz(p,c.path[i],c.path[i+1]);
        const float d=distance_xz(q,p);
        if(d<best_d){ best_d=d; best=q; }
    }
    return best;
}

std::vector<Vec3> sampled_path_points(const CorridorConnection& c,Vec3 toward){
    std::vector<Vec3> out;
    if(c.path.empty()) return out;
    const Vec3 nearest=nearest_point_on_path(c,toward);
    out.push_back(nearest);
    const std::size_t target_samples=9;
    const std::size_t stride=std::max<std::size_t>(1,(c.path.size()+target_samples-1)/target_samples);
    for(std::size_t i=0;i<c.path.size();i+=stride){
        const Vec3 p=c.path[i];
        bool duplicate=false;
        for(const Vec3 q:out) if(distance_xz(p,q)<0.10f){ duplicate=true; break; }
        if(!duplicate) out.push_back(p);
    }
    if(distance_xz(out.back(),c.path.back())>=0.10f) out.push_back(c.path.back());
    std::stable_sort(out.begin(),out.end(),[&](Vec3 a,Vec3 b){ return distance_xz(a,toward)<distance_xz(b,toward); });
    return out;
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

std::optional<CorridorEndpoint> CorridorNetwork::preview_anchor(
    const CorridorBuildAnchor& anchor,Vec3 other,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
    const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library) const {
    if(anchor.kind==CorridorEndpointKind::Corridor && anchor.index>=connections_.size()) return std::nullopt;
    if(anchor.kind==CorridorEndpointKind::Habitat && anchor.index>=habitats.size()) return std::nullopt;
    if(anchor.kind==CorridorEndpointKind::Resource && (anchor.index>=resources.size() || resources[anchor.index].kind!=ResourceKind::StorageDome)) return std::nullopt;
    return resolve_anchor(anchor,other,habitats,library,resources,resource_library,connections_);
}

CorridorMesh CorridorNetwork::preview_endpoint_mesh(const CorridorEndpoint& endpoint) const {
    CorridorMesh mesh;
    append_docking_collar(mesh,endpoint);
    return mesh;
}

std::optional<CorridorConnection> CorridorNetwork::preview_built_corridor(
    const CorridorBuildAnchor& a,const CorridorBuildAnchor& b,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
    const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library) const {
    if(a.kind==CorridorEndpointKind::Corridor && a.index>=connections_.size()) return std::nullopt;
    if(b.kind==CorridorEndpointKind::Corridor && b.index>=connections_.size()) return std::nullopt;
    if(a.kind==CorridorEndpointKind::Habitat && a.index>=habitats.size()) return std::nullopt;
    if(b.kind==CorridorEndpointKind::Habitat && b.index>=habitats.size()) return std::nullopt;
    if(a.kind==CorridorEndpointKind::Resource && (a.index>=resources.size() || resources[a.index].kind!=ResourceKind::StorageDome)) return std::nullopt;
    if(b.kind==CorridorEndpointKind::Resource && (b.index>=resources.size() || resources[b.index].kind!=ResourceKind::StorageDome)) return std::nullopt;
    // Two anchors on the same host corridor are valid as long as they are
    // genuinely different points.  This keeps every centerline position usable
    // as a source or destination and lets the router create a non-overlapping
    // bypass/loop between two explicit junction nodes when requested.
    if(a.kind==b.kind && a.index==b.index && a.kind!=CorridorEndpointKind::OpenDoor){
        if(a.kind!=CorridorEndpointKind::Corridor) return std::nullopt;
        const Vec3 pa=nearest_point_on_corridor(a.index,a.position);
        const Vec3 pb=nearest_point_on_corridor(b.index,b.position);
        if(distance_xz(pa,pb)<0.75f) return std::nullopt;
    }
    return make_generic(a,b,CorridorConnectionKind::BuiltSection,habitats,library,resources,resource_library,connections_,playfield_half_extent_);
}

bool CorridorNetwork::build_corridor(const CorridorBuildAnchor& a,const CorridorBuildAnchor& b,
                                     const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                                     const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library){
    // Do not add a second physical pressure tube when both requested anchors are
    // already connected through the merged corridor graph. A dense colony can
    // therefore accept repeated/all-to-all connection requests without growing
    // parallel overlapping tubes; the existing shared network is the route.
    if(corridor_anchors_already_connected(a,b,connections_)) return true;

    const auto commit=[&](CorridorConnection c){
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(ep->kind!=CorridorEndpointKind::Corridor || ep->index>=connections_.size()) continue;
            insert_shared_path_vertex(connections_[ep->index],
                                      ep->surface_matched_junction?ep->host_position:ep->position);
        }
        c.resource_plan=construction_costs::corridor(c.path,network_junction_count(c));
        connections_.push_back(std::move(c));
        optimize_shared_entrance_groups(connections_,habitats,library);
        rebuild_all_meshes(connections_);
        return true;
    };

    if(auto direct=preview_built_corridor(a,b,habitats,library,resources,resource_library))
        return commit(std::move(*direct));

    // Dense layouts often already contain most of the route we need. If a
    // direct structure-to-structure tube cannot be placed, connect the two
    // existing network components at their nearest valid centerline points
    // instead. If only one side is already networked, attach the unconnected
    // structure to that component. This reuses trunks, prevents parallel
    // duplicate corridors, and turns an encounter with an existing corridor
    // into an intentional merged junction instead of a hard routing failure.
    const auto comp_a=corridor_component_indices(a,connections_);
    const auto comp_b=corridor_component_indices(b,connections_);

    struct Candidate { float distance; CorridorBuildAnchor left; CorridorBuildAnchor right; };
    std::vector<Candidate> candidates;
    const auto anchor_pos=[&](const CorridorBuildAnchor& anchor)->Vec3{
        if(anchor.kind==CorridorEndpointKind::Habitat && anchor.index<habitats.size()) return habitats[anchor.index].position;
        if(anchor.kind==CorridorEndpointKind::Resource && anchor.index<resources.size()) return resources[anchor.index].position;
        if(anchor.kind==CorridorEndpointKind::Corridor && anchor.index<connections_.size()) return nearest_point_on_path(connections_[anchor.index],anchor.position);
        return anchor.position;
    };

    // A Storage Dome has four physical service doors.  If the user-selected
    // face is boxed in, dense-layout fallback may use another real door rather
    // than declaring the entire structure unreachable.  The original anchor is
    // always tried first; extra variants are only considered after the direct
    // requested route failed.
    const auto structure_anchor_variants=[&](const CorridorBuildAnchor& src){
        std::vector<CorridorBuildAnchor> out{src};
        if(src.kind!=CorridorEndpointKind::Resource || src.index>=resources.size() ||
           resources[src.index].kind!=ResourceKind::StorageDome) return out;
        const auto& placed=resources[src.index];
        const auto& mesh=resource_library.mesh(placed);
        const float r=std::min(mesh.footprint_x,mesh.footprint_z)*0.40f;
        const Vec3 local_normals[4]={{0,0,-1},{1,0,0},{0,0,1},{-1,0,0}};
        for(Vec3 ln:local_normals){
            CorridorBuildAnchor v=src;
            v.position=placed.position+flat_normal(rotate_y(ln,placed.rotation_degrees))*r;
            bool duplicate=false;
            for(const auto& e:out) if(distance_xz(e.position,v.position)<0.10f){ duplicate=true; break; }
            if(!duplicate) out.push_back(v);
        }
        return out;
    };

    if(comp_a.empty() && !comp_b.empty()){
        for(const auto& av:structure_anchor_variants(a))
            for(std::size_t ci:comp_b) for(const Vec3 q:sampled_path_points(connections_[ci],anchor_pos(av)))
                candidates.push_back({distance_xz(anchor_pos(av),q),av,{CorridorEndpointKind::Corridor,ci,q}});
    }else if(!comp_a.empty() && comp_b.empty()){
        for(const auto& bv:structure_anchor_variants(b))
            for(std::size_t ci:comp_a) for(const Vec3 q:sampled_path_points(connections_[ci],anchor_pos(bv)))
                candidates.push_back({distance_xz(anchor_pos(bv),q),bv,{CorridorEndpointKind::Corridor,ci,q}});
    }else if(!comp_a.empty() && !comp_b.empty()){
        // Prefer attaching either requested structure/entrance directly to the
        // opposite component. Search along the existing trunk rather than only
        // its nearest point: the nearest point may be boxed in by a neighboring
        // structure while a clean merged junction exists a few metres farther.
        for(const auto& av:structure_anchor_variants(a))
            for(std::size_t cb:comp_b) for(const Vec3 q:sampled_path_points(connections_[cb],anchor_pos(av)))
                candidates.push_back({distance_xz(anchor_pos(av),q),av,{CorridorEndpointKind::Corridor,cb,q}});
        for(const auto& bv:structure_anchor_variants(b))
            for(std::size_t ca:comp_a) for(const Vec3 q:sampled_path_points(connections_[ca],anchor_pos(bv)))
                candidates.push_back({distance_xz(anchor_pos(bv),q),bv,{CorridorEndpointKind::Corridor,ca,q}});
        // If neither selected entrance can reach the opposite component, join
        // the two components directly at their nearest pair of centerline points.
        for(std::size_t ca:comp_a) for(std::size_t cb:comp_b){
            if(ca==cb) continue;
            const auto pair=closest_pair(connections_[ca].path,connections_[cb].path);
            candidates.push_back({distance_xz(pair.first,pair.second),
                                  {CorridorEndpointKind::Corridor,ca,pair.first},
                                  {CorridorEndpointKind::Corridor,cb,pair.second}});
        }
    }

    std::stable_sort(candidates.begin(),candidates.end(),[](const Candidate& x,const Candidate& y){ return x.distance<y.distance; });
    // Reserve one representative target for every distinct existing corridor
    // before spending the direct-route budget.  The nearest trunk can be
    // completely boxed in while a slightly farther trunk is easy to reach; if
    // waypoint fallback were built only from the first failed direct attempts,
    // dense layouts could starve all of those farther trunks from consideration.
    constexpr std::size_t MAX_WAYPOINT_TARGETS=12;
    constexpr std::size_t MAX_PER_HOST=2;
    struct HostVariantCandidate { std::size_t host; Vec3 other_position; Candidate candidate; };
    std::vector<HostVariantCandidate> host_variants;
    std::vector<std::size_t> host_order;
    for(const auto& candidate:candidates){
        const CorridorBuildAnchor* host=nullptr;
        const CorridorBuildAnchor* other=nullptr;
        if(candidate.left.kind==CorridorEndpointKind::Corridor){ host=&candidate.left; other=&candidate.right; }
        else if(candidate.right.kind==CorridorEndpointKind::Corridor){ host=&candidate.right; other=&candidate.left; }
        if(!host || !other) continue;
        if(std::find(host_order.begin(),host_order.end(),host->index)==host_order.end()) host_order.push_back(host->index);
        bool duplicate=false;
        for(const auto& hv:host_variants){
            if(hv.host==host->index && distance_xz(hv.other_position,other->position)<0.75f){ duplicate=true; break; }
        }
        // candidates are sorted by route distance, so the first occurrence of
        // each host/structure-door pair is its nearest centerline target.
        if(!duplicate) host_variants.push_back({host->index,other->position,candidate});
    }
    std::vector<Candidate> waypoint_candidates;
    // Round-robin by host and then by distinct source/door variant. This gives
    // every trunk one chance before a second Storage Dome service face is tried.
    for(std::size_t round=0;round<MAX_PER_HOST && waypoint_candidates.size()<MAX_WAYPOINT_TARGETS;++round){
        for(std::size_t host_index:host_order){
            std::size_t seen=0;
            for(const auto& hv:host_variants){
                if(hv.host!=host_index) continue;
                if(seen++!=round) continue;
                waypoint_candidates.push_back(hv.candidate);
                break;
            }
            if(waypoint_candidates.size()>=MAX_WAYPOINT_TARGETS) break;
        }
    }
    if(waypoint_candidates.empty())
        for(std::size_t i=0;i<std::min<std::size_t>(candidates.size(),MAX_WAYPOINT_TARGETS);++i)
            waypoint_candidates.push_back(candidates[i]);

    // Keep direct attempts intentionally small.  This is the hot preview path:
    // a handful of shortest candidates is enough before switching to the
    // host-diverse one-bend search below.
    constexpr std::size_t MAX_COMPONENT_CANDIDATES=10;
    std::size_t tried=0;
    for(const auto& candidate:candidates){
        if(tried++>=MAX_COMPONENT_CANDIDATES) break;
        auto routed=preview_built_corridor(candidate.left,candidate.right,habitats,library,resources,resource_library);
        if(routed) return commit(std::move(*routed));
    }

    // One bend is enough for the remaining dense-layout cases: leave the
    // source into open space, then approach one candidate on each distinct host
    // corridor from a clean angle.  The bounded host count/shapes keep failed
    // previews responsive even in a fully connected base.
    const float half=playfield_half_extent_;

    for(const auto& candidate:waypoint_candidates){
        const Vec3 lp=candidate.left.kind==CorridorEndpointKind::Corridor
            ? nearest_point_on_path(connections_[candidate.left.index],candidate.left.position) : candidate.left.position;
        const Vec3 rp=candidate.right.kind==CorridorEndpointKind::Corridor
            ? nearest_point_on_path(connections_[candidate.right.index],candidate.right.position) : candidate.right.position;
        Vec3 forward=flat_normal(rp-lp);
        if(length(forward)<0.001f) forward={1,0,0};
        const Vec3 side=perp_xz(forward);
        struct WaypointShape { float progress; float offset; };
        const WaypointShape shapes[]={
            {0.335f,9.0f}
        };
        for(const auto shape:shapes){
            const Vec3 base=lp+(rp-lp)*shape.progress;
            for(float sign:{-1.0f,1.0f}){
                Vec3 waypoint=base+side*(shape.offset*sign);
                waypoint.x=std::clamp(waypoint.x,-half+1.0f,half-1.0f);
                waypoint.z=std::clamp(waypoint.z,-half+1.0f,half-1.0f);
                auto routed=make_generic_via_waypoint(candidate.left,candidate.right,waypoint,
                                                       CorridorConnectionKind::BuiltSection,
                                                       habitats,library,resources,resource_library,
                                                       connections_,playfield_half_extent_);
                if(routed) return commit(std::move(*routed));
            }
        }
    }
    return false;
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
    static const std::vector<PlacedResource> no_resources;
    static const ResourceModelLibrary resource_library;
    return make_generic(a,b,CorridorConnectionKind::NetworkLink,habitats,library,no_resources,resource_library,connections_,playfield_half_extent_);
}

bool CorridorNetwork::connect_corridors(std::size_t corridor_a,std::size_t corridor_b,
                                        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library){
    auto c=preview_corridor_link(corridor_a,corridor_b,habitats,library);
    if(!c) return false;
    for(const CorridorEndpoint* ep:{&c->endpoint_a,&c->endpoint_b}){
        if(ep->kind!=CorridorEndpointKind::Corridor || ep->index>=connections_.size()) continue;
        insert_shared_path_vertex(connections_[ep->index],
                                  ep->surface_matched_junction?ep->host_position:ep->position);
    }
    c->resource_plan=construction_costs::corridor(c->path,network_junction_count(*c));
    connections_.push_back(std::move(*c));
    rebuild_all_meshes(connections_);
    return true;
}

void CorridorNetwork::update_construction(float dt,ResourceManagement& resources,bool allow_negative){
    for(auto& connection:connections_){
        connection.construction.update_gated(dt,[&](std::uint8_t next_step){
            return resources.consume(connection.resource_plan.next_step_cost((std::uint8_t)(next_step-1)),allow_negative);
        });
    }
}

Vec3 CorridorNetwork::nearest_point_on_corridor(std::size_t corridor_index,Vec3 p) const {
    if(corridor_index>=connections_.size()) return p;
    return nearest_path_point(connections_[corridor_index].path,p);
}

CorridorBuildAnchor CorridorNetwork::corridor_anchor(std::size_t corridor_index,Vec3 near_point) const {
    CorridorBuildAnchor anchor{CorridorEndpointKind::Corridor,corridor_index,nearest_point_on_corridor(corridor_index,near_point)};
    anchor.cursor_position=near_point;
    anchor.cursor_position.y=0.0f;
    anchor.has_cursor_position=true;
    return anchor;
}

std::size_t CorridorNetwork::habitat_connection_count(std::size_t habitat_index) const {
    std::size_t count=0;
    for(const auto& c:connections_){
        if(endpoint_uses_habitat(c.endpoint_a,habitat_index) || endpoint_uses_habitat(c.endpoint_b,habitat_index)) ++count;
    }
    return count;
}

std::size_t CorridorNetwork::resource_connection_count(std::size_t resource_index) const {
    std::size_t count=0;
    for(const auto& c:connections_)
        if(endpoint_uses_resource(c.endpoint_a,resource_index) || endpoint_uses_resource(c.endpoint_b,resource_index)) ++count;
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
    rebuild_all_meshes(connections_);
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
    rebuild_all_meshes(connections_);
}

void CorridorNetwork::remove_resource(std::size_t resource_index){
    for(auto& connection:connections_){
        for(CorridorEndpoint* endpoint:{&connection.endpoint_a,&connection.endpoint_b}){
            if(endpoint->kind!=CorridorEndpointKind::Resource) continue;
            if(endpoint->index==resource_index){
                endpoint->kind=CorridorEndpointKind::OpenDoor;
                endpoint->index=0;
                endpoint->door=CorridorDoor{};
                endpoint->suppress_open_door=false;
            }else if(endpoint->index>resource_index){
                --endpoint->index;
            }
        }
    }
    rebuild_all_meshes(connections_);
}

} // namespace game
