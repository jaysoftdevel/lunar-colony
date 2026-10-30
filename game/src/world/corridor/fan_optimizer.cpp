#include "game/world/corridor/detail.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace game::corridor_detail {
namespace {
constexpr float FAN_ZONE=48.0f;
constexpr float FIRST_BRANCH_SLOT=2.15f;
constexpr float SAME_SIDE_SPACING=2.65f;
constexpr float TRUNK_AFTER_LAST_BRANCH=1.55f;
constexpr float ROUTE_CLEARANCE_BETWEEN_CORRIDORS=2.18f;

struct GroupMember {
    std::size_t connection{};
    bool root{false};
    bool attach_endpoint_a{false};
    CorridorEndpoint far{};
    float lateral{};
    float forward{};
    float angle{};
    int side{};
    int side_rank{};
};

float distance_along_from_end(const CorridorConnection& host,bool from_a,Vec3 point){
    if(host.path.size()<2) return std::numeric_limits<float>::max();
    float accumulated=0.0f;
    float best_spatial=std::numeric_limits<float>::max();
    float best_along=std::numeric_limits<float>::max();
    if(from_a){
        for(std::size_t i=0;i+1<host.path.size();++i){
            const Vec3 q=closest_point_segment_xz(point,host.path[i],host.path[i+1]);
            const float spatial=distance_xz(q,point);
            const float along=accumulated+distance_xz(host.path[i],q);
            if(spatial<best_spatial){ best_spatial=spatial; best_along=along; }
            accumulated+=distance_xz(host.path[i],host.path[i+1]);
        }
    }else{
        for(std::size_t i=host.path.size()-1;i>0;--i){
            const Vec3 q=closest_point_segment_xz(point,host.path[i],host.path[i-1]);
            const float spatial=distance_xz(q,point);
            const float along=accumulated+distance_xz(host.path[i],q);
            if(spatial<best_spatial){ best_spatial=spatial; best_along=along; }
            accumulated+=distance_xz(host.path[i],host.path[i-1]);
        }
    }
    return best_along;
}

Vec3 point_along_from_end(const CorridorConnection& host,bool from_a,float wanted){
    if(host.path.empty()) return {};
    float remaining=std::max(0.0f,wanted);
    if(from_a){
        for(std::size_t i=0;i+1<host.path.size();++i){
            const float len=distance_xz(host.path[i],host.path[i+1]);
            if(len>=remaining && len>0.0001f) return host.path[i]+(host.path[i+1]-host.path[i])*(remaining/len);
            remaining-=len;
        }
        return host.path.back();
    }
    for(std::size_t i=host.path.size()-1;i>0;--i){
        const float len=distance_xz(host.path[i],host.path[i-1]);
        if(len>=remaining && len>0.0001f) return host.path[i]+(host.path[i-1]-host.path[i])*(remaining/len);
        remaining-=len;
    }
    return host.path.front();
}

CorridorEndpoint make_surface_branch(std::size_t root_index,const CorridorConnection& root,Vec3 host_point,Vec3 other){
    CorridorEndpoint out;
    out.kind=CorridorEndpointKind::Corridor;
    out.index=root_index;
    out.host_position=nearest_path_point(root.path,host_point);
    out.host_tangent=nearest_path_tangent(root.path,out.host_position);
    Vec3 side=perp_xz(out.host_tangent);
    if(length(side)<0.001f) side={0,0,1};
    if(dot(flatten(other-out.host_position),side)<0.0f) side=side*-1.0f;
    out.tangent=flat_normal(side);
    const float center_gap=distance_xz(other,out.host_position);
    const float free_gap=std::max(0.0f,center_gap-CORRIDOR_HALF_WIDTH);
    const float surface_adapter_run=std::clamp(free_gap*0.45f,0.08f,0.42f);
    out.position=out.host_position+out.tangent*(CORRIDOR_HALF_WIDTH+surface_adapter_run);
    out.position.y=out.host_position.y;
    out.surface_matched_junction=true;
    return out;
}

CorridorEndpoint& root_far(CorridorConnection& c,bool root_from_a){
    return root_from_a?c.endpoint_b:c.endpoint_a;
}
CorridorEndpoint& branch_far(CorridorConnection& c,bool attach_a){
    return attach_a?c.endpoint_b:c.endpoint_a;
}
CorridorEndpoint& branch_attach(CorridorConnection& c,bool attach_a){
    return attach_a?c.endpoint_a:c.endpoint_b;
}

float segment_distance_approx(Vec3 a0,Vec3 a1,Vec3 b0,Vec3 b1){
    float best=std::numeric_limits<float>::max();
    for(int i=0;i<=5;++i){
        const float t=(float)i/5.0f;
        const Vec3 p=a0+(a1-a0)*t;
        best=std::min(best,point_segment_distance_xz(p,b0,b1));
    }
    for(int i=0;i<=5;++i){
        const float t=(float)i/5.0f;
        const Vec3 p=b0+(b1-b0)*t;
        best=std::min(best,point_segment_distance_xz(p,a0,a1));
    }
    return best;
}

bool paths_conflict(const std::vector<Vec3>& a,const std::vector<Vec3>& b,Vec3 allowed_junction){
    if(a.size()<2 || b.size()<2) return false;
    std::vector<Vec3> crossings;
    for(std::size_t i=0;i+1<a.size();++i) for(std::size_t j=0;j+1<b.size();++j){
        Vec3 hit{};
        if(mergeable_corridor_crossing(a[i],a[i+1],b[j],b[j+1],hit)){
            bool duplicate=false;
            for(Vec3 p:crossings) if(distance_xz(p,hit)<0.25f){ duplicate=true; break; }
            if(!duplicate) crossings.push_back(hit);
        }
    }
    constexpr float AUTO_ZONE=CORRIDOR_HALF_WIDTH*1.75f;
    for(std::size_t i=0;i+1<a.size();++i){
        for(std::size_t j=0;j+1<b.size();++j){
            const Vec3 amid=(a[i]+a[i+1])*0.5f;
            const Vec3 bmid=(b[j]+b[j+1])*0.5f;
            constexpr float EXPLICIT_JUNCTION_ZONE=CORRIDOR_HALF_WIDTH*2.0f+0.32f;
            if(distance_xz(amid,allowed_junction)<EXPLICIT_JUNCTION_ZONE &&
               distance_xz(bmid,allowed_junction)<EXPLICIT_JUNCTION_ZONE) continue;
            if(segment_distance_approx(a[i],a[i+1],b[j],b[j+1])>=ROUTE_CLEARANCE_BETWEEN_CORRIDORS) continue;
            bool merged=false;
            for(Vec3 hit:crossings){
                if((point_segment_distance_xz(hit,a[i],a[i+1])<0.05f || distance_xz(amid,hit)<AUTO_ZONE) &&
                   (point_segment_distance_xz(hit,b[j],b[j+1])<0.05f || distance_xz(bmid,hit)<AUTO_ZONE)){
                    merged=true; break;
                }
            }
            if(!merged) return true;
        }
    }
    return false;
}

std::optional<Vec3> shared_junction(const std::vector<CorridorConnection>& connections,std::size_t a,std::size_t b){
    if(a>=connections.size() || b>=connections.size()) return std::nullopt;
    for(const CorridorEndpoint* ep:{&connections[a].endpoint_a,&connections[a].endpoint_b}){
        if(ep->kind==CorridorEndpointKind::Corridor && ep->index==b)
            return ep->surface_matched_junction?ep->host_position:ep->position;
    }
    for(const CorridorEndpoint* ep:{&connections[b].endpoint_a,&connections[b].endpoint_b}){
        if(ep->kind==CorridorEndpointKind::Corridor && ep->index==a)
            return ep->surface_matched_junction?ep->host_position:ep->position;
    }
    return std::nullopt;
}

bool network_has_conflicts(const std::vector<CorridorConnection>& connections){
    constexpr Vec3 FAR_AWAY{1000000.0f,0.0f,1000000.0f};
    for(std::size_t a=0;a<connections.size();++a){
        if(connections[a].path.size()<2) continue;
        if(corridor_path_self_overlaps(connections[a].path)) return true;
        for(std::size_t b=a+1;b<connections.size();++b){
            if(connections[b].path.size()<2) continue;
            const auto junction=shared_junction(connections,a,b);
            if(paths_conflict(connections[a].path,connections[b].path,junction.value_or(FAR_AWAY))) return true;
        }
    }
    return false;
}

std::vector<Vec3> root_route_with_trunk(const CorridorEndpoint& entrance,const CorridorEndpoint& destination,
                                        float trunk_length,const std::vector<PlacedHabitat>& habitats,
                                        const HabitatModelLibrary& library){
    const Vec3 dir=flat_normal(entrance.tangent);
    const float terminal=0.16f;
    for(int attempt=0;attempt<5;++attempt){
        const float run=std::max(1.35f,trunk_length-attempt*0.65f);
        CorridorEndpoint trunk=entrance;
        trunk.position=entrance.position+dir*run;
        trunk.position.y=entrance.position.y;
        trunk.tangent=dir;
        auto tail=smooth_route(trunk,destination,habitats,library);
        if(tail.size()<2) continue;
        std::vector<Vec3> out;
        out.reserve(tail.size()+3);
        out.push_back(entrance.position);
        out.push_back(entrance.position+dir*terminal);
        if(run>terminal+0.08f) out.push_back(trunk.position);
        for(std::size_t i=1;i<tail.size();++i) out.push_back(tail[i]);
        return out;
    }
    return smooth_route(entrance,destination,habitats,library);
}

int side_for(float lateral,int left_count,int right_count){
    if(lateral>0.18f) return 1;
    if(lateral<-0.18f) return -1;
    return left_count<=right_count?1:-1;
}

void optimize_group(std::vector<CorridorConnection>& connections,std::size_t root_index,bool root_from_a,
                    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library){
    if(root_index>=connections.size()) return;
    CorridorConnection& root=connections[root_index];
    if(root.path.size()<2) return;
    CorridorEndpoint& entrance=root_from_a?root.endpoint_a:root.endpoint_b;
    if(entrance.kind!=CorridorEndpointKind::Habitat && entrance.kind!=CorridorEndpointKind::Resource) return;

    const Vec3 entrance_pos=entrance.position;
    Vec3 entrance_dir=flat_normal(entrance.tangent);
    if(length(entrance_dir)<0.001f){
        entrance_dir=root_from_a?flat_normal(root.path[1]-root.path.front())
                                :flat_normal(root.path[root.path.size()-2]-root.path.back());
    }
    const Vec3 lateral_axis=perp_xz(entrance_dir);

    std::vector<GroupMember> members;
    GroupMember rm;
    rm.connection=root_index;
    rm.root=true;
    rm.far=root_far(root,root_from_a);
    members.push_back(rm);

    for(std::size_t ci=0;ci<connections.size();++ci){
        if(ci==root_index || connections[ci].kind!=CorridorConnectionKind::BuiltSection) continue;
        for(int es=0;es<2;++es){
            const CorridorEndpoint& ep=es==0?connections[ci].endpoint_a:connections[ci].endpoint_b;
            if(ep.kind!=CorridorEndpointKind::Corridor || ep.index!=root_index || !ep.surface_matched_junction) continue;
            const float current_along=distance_along_from_end(root,root_from_a,ep.host_position);
            const float opposite_along=distance_along_from_end(root,!root_from_a,ep.host_position);
            if(current_along>FAN_ZONE) continue;
            // A corridor with structures at both ends used to optimize the same
            // branch twice. The second pass could flip the branch to the opposite
            // side and create a tight U-turn/self-overlap. A branch belongs to
            // exactly one entrance fan: whichever root end is physically closer.
            // Resolve exact midpoint ties deterministically to endpoint A.
            constexpr float OWNER_TIE_EPS=0.12f;
            if(current_along>opposite_along+OWNER_TIE_EPS) continue;
            if(std::abs(current_along-opposite_along)<=OWNER_TIE_EPS && !root_from_a) continue;
            GroupMember m;
            m.connection=ci;
            m.attach_endpoint_a=es==0;
            m.far=es==0?connections[ci].endpoint_b:connections[ci].endpoint_a;
            members.push_back(m);
        }
    }
    if(members.size()<=1) return;

    // Rebalancing is allowed to move several already-built legs. Keep a full
    // topology snapshot so a candidate fan can be rolled back atomically if
    // the new shared layout would collide with any corridor outside the fan.
    const auto topology_snapshot=connections;

    // Optimize the whole destination set instead of preserving click order.
    // The destination most aligned with the door normal becomes the trunk
    // continuation; the others fan off left/right in angular order.
    for(auto& m:members){
        const Vec3 d=flatten(m.far.position-entrance_pos);
        const Vec3 n=length(d)>0.001f?normalize(d):entrance_dir;
        m.forward=dot(n,entrance_dir);
        m.lateral=dot(n,lateral_axis);
        m.angle=std::atan2(m.lateral,m.forward);
    }
    auto best_it=std::max_element(members.begin(),members.end(),[](const GroupMember& a,const GroupMember& b){
        if(std::abs(a.forward-b.forward)>0.001f) return a.forward<b.forward;
        return std::abs(a.angle)>std::abs(b.angle);
    });
    const CorridorEndpoint best_far=best_it->far;
    root_far(root,root_from_a)=best_far;

    std::vector<GroupMember*> branch_members;
    for(auto& m:members){
        if(m.root) continue;
        branch_members.push_back(&m);
    }
    // Reassign all non-trunk destinations to the existing branch connection
    // slots. The branch that happened to be built first/last does not own a
    // particular destination; topology is optimized independently of click
    // order. The root connection itself always keeps the habitat endpoint.
    std::vector<CorridorEndpoint> remaining;
    for(const auto& m:members) if(&m!=&*best_it) remaining.push_back(m.far);
    std::sort(remaining.begin(),remaining.end(),[&](const CorridorEndpoint& a,const CorridorEndpoint& b){
        const Vec3 da=flat_normal(a.position-entrance_pos),db=flat_normal(b.position-entrance_pos);
        return dot(da,lateral_axis)<dot(db,lateral_axis);
    });
    std::sort(branch_members.begin(),branch_members.end(),[](const GroupMember* a,const GroupMember* b){
        return a->connection<b->connection;
    });
    for(std::size_t i=0;i<branch_members.size() && i<remaining.size();++i){
        branch_members[i]->far=remaining[i];
        branch_far(connections[branch_members[i]->connection],branch_members[i]->attach_endpoint_a)=remaining[i];
        const Vec3 n=flat_normal(remaining[i].position-entrance_pos);
        branch_members[i]->forward=dot(n,entrance_dir);
        branch_members[i]->lateral=dot(n,lateral_axis);
        branch_members[i]->angle=std::atan2(branch_members[i]->lateral,branch_members[i]->forward);
    }

    int left_count=0,right_count=0;
    for(auto* m:branch_members){
        m->side=side_for(m->lateral,left_count,right_count);
        if(m->side>0) ++left_count; else ++right_count;
    }
    for(int side:{-1,1}){
        std::vector<GroupMember*> same;
        for(auto* m:branch_members) if(m->side==side) same.push_back(m);
        // More lateral destinations leave the common trunk first. This keeps
        // same-side routes nested rather than crossing each other.
        std::sort(same.begin(),same.end(),[](const GroupMember* a,const GroupMember* b){
            return std::abs(a->angle)>std::abs(b->angle);
        });
        for(std::size_t i=0;i<same.size();++i) same[i]->side_rank=(int)i;
    }

    float max_slot=0.0f;
    for(const auto* m:branch_members)
        max_slot=std::max(max_slot,FIRST_BRANCH_SLOT+(float)m->side_rank*SAME_SIDE_SPACING);
    const float trunk_length=std::max(2.8f,max_slot+TRUNK_AFTER_LAST_BRANCH);

    auto root_path=root_route_with_trunk(entrance,root_far(root,root_from_a),trunk_length,habitats,library);
    if(root_path.size()<2) return;
    if(!root_from_a) std::reverse(root_path.begin(),root_path.end());
    root.path=std::move(root_path);

    // Rebuild every branch from the newly optimized common trunk. Branches on
    // opposite sides may share a longitudinal slot; same-side branches are
    // spaced by more than one corridor diameter so their shells do not overlap.
    std::vector<std::vector<Vec3>> accepted;
    accepted.push_back(root.path);
    for(auto* m:branch_members){
        CorridorConnection& c=connections[m->connection];
        CorridorEndpoint& far=branch_far(c,m->attach_endpoint_a);
        const float slot=FIRST_BRANCH_SLOT+(float)m->side_rank*SAME_SIDE_SPACING;
        const Vec3 host_point=point_along_from_end(root,root_from_a,slot);
        CorridorEndpoint attach=make_surface_branch(root_index,root,host_point,far.position);
        // make_surface_branch() already chooses the local host side that faces
        // the destination. Do not overwrite that choice with the entrance-wide
        // left/right rank: when optimizing from endpoint B, the root path's
        // stored tangent orientation is reversed relative to that entrance and
        // the old forced sign sent branches to the wrong side. Only resolve the
        // genuinely ambiguous centered case with the fan's balanced side rank.
        Vec3 local_side=perp_xz(attach.host_tangent);
        const float local_lateral=dot(flatten(far.position-attach.host_position),local_side);
        if(std::abs(local_lateral)<0.18f){
            Vec3 desired=lateral_axis*(float)m->side;
            if(dot(local_side,desired)<0.0f) local_side=local_side*-1.0f;
            attach.tangent=flat_normal(local_side);
        }
        const float center_gap=distance_xz(far.position,attach.host_position);
        const float free_gap=std::max(0.0f,center_gap-CORRIDOR_HALF_WIDTH);
        const float adapter_run=std::clamp(free_gap*0.45f,0.08f,0.42f);
        attach.position=attach.host_position+attach.tangent*(CORRIDOR_HALF_WIDTH+adapter_run);
        branch_attach(c,m->attach_endpoint_a)=attach;

        auto path=smooth_route(c.endpoint_a,c.endpoint_b,habitats,library);
        if(path.size()<2) continue;
        // If the generic router bends back through another branch, make the
        // first departure longer and retry. This preserves the exact host
        // junction while separating the branch centerlines before they curve.
        bool conflict=false;
        for(const auto& other:accepted){
            if(paths_conflict(path,other,attach.host_position)){ conflict=true; break; }
        }
        if(conflict && path.size()>=3){
            const Vec3 start=attach.position;
            const Vec3 lead=start+attach.tangent*(2.1f+0.75f*(float)m->side_rank);
            CorridorEndpoint synthetic=attach;
            synthetic.position=lead;
            auto tail=smooth_route(synthetic,far,habitats,library);
            if(tail.size()>=2){
                std::vector<Vec3> retry{start,start+attach.tangent*0.16f,lead};
                retry.insert(retry.end(),tail.begin()+1,tail.end());
                bool retry_conflict=false;
                for(const auto& other:accepted){
                    if(paths_conflict(retry,other,attach.host_position)){ retry_conflict=true; break; }
                }
                if(!retry_conflict) path=std::move(retry);
            }
        }
        c.path=std::move(path);
        accepted.push_back(c.path);
    }

    // Rebuild the pressure geometry for every member against the same final
    // topology. The junction fitting references the exact optimized host point;
    // no click-order branch vertex is retained.
    root.mesh=build_mesh(root.endpoint_a,root.endpoint_b,root.path,connections);
    for(auto* m:branch_members){
        CorridorConnection& c=connections[m->connection];
        c.mesh=build_mesh(c.endpoint_a,c.endpoint_b,c.path,connections);
    }

    // Never trade an old valid topology for a prettier but intersecting fan.
    // Explicit parent/branch junctions are exempted by network_has_conflicts();
    // every other pair must retain full pressure-shell clearance.
    if(network_has_conflicts(connections)){
        connections=topology_snapshot;
        return;
    }
}
} // namespace

void optimize_shared_entrance_groups(std::vector<CorridorConnection>& connections,
                                     const std::vector<PlacedHabitat>& habitats,
                                     const HabitatModelLibrary& library){
    // Scan all habitat-owned roots. A root is the only corridor that directly
    // owns the pressure entrance; all additional destinations attach to its
    // common trunk and are therefore optimized as one topology group.
    for(std::size_t i=0;i<connections.size();++i){
        if(connections[i].path.size()<2) continue;
        if(connections[i].endpoint_a.kind==CorridorEndpointKind::Habitat ||
           connections[i].endpoint_a.kind==CorridorEndpointKind::Resource)
            optimize_group(connections,i,true,habitats,library);
        if(i<connections.size() && (connections[i].endpoint_b.kind==CorridorEndpointKind::Habitat ||
                                    connections[i].endpoint_b.kind==CorridorEndpointKind::Resource))
            optimize_group(connections,i,false,habitats,library);
    }
}

} // namespace game::corridor_detail
