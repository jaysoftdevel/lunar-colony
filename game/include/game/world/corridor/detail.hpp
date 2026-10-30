#pragma once
#include "game/world/corridor_network.hpp"
#include "game/world/geometry_standards.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace game::corridor_detail {
using namespace eng;

constexpr float PI=3.14159265358979323846f;
constexpr float CORRIDOR_HALF_WIDTH=geometry::kCorridorHalfWidth;
constexpr float CORRIDOR_FLOOR_Y=geometry::kCorridorFloorY;
constexpr float CORRIDOR_SIDE_Y=geometry::kCorridorWallY;
constexpr float CORRIDOR_ROOF_RADIUS=geometry::kCorridorRoofRadius;
constexpr float WALKWAY_Y=geometry::kCorridorDeckY;
constexpr float WALKWAY_HALF_WIDTH=geometry::kCorridorDeckHalfWidth;
constexpr float DOCK_RUN=geometry::kCorridorDockRun;
constexpr float GENERATED_ENTRANCE_DEPTH=geometry::kGeneratedEntranceDepth;
constexpr float GENERATED_ENTRANCE_HALF_WIDTH=geometry::kGeneratedEntranceHalfWidth;
constexpr float GENERATED_ENTRANCE_HEIGHT=geometry::kGeneratedEntranceHeight;
constexpr float GENERATED_HULL_OVERLAP=0.34f;
constexpr float DOCK_SLEEVE_HALF_WIDTH=geometry::kDockSleeveHalfWidth;
constexpr float DOCK_SLEEVE_HEIGHT=geometry::kDockSleeveHeight;
constexpr float DOCK_DOOR_OVERLAP=0.28f;
constexpr float DOCK_CORRIDOR_OVERLAP=0.42f;
constexpr float ENTRANCE_MIN_SPACING=geometry::kEntranceMinSpacing;
constexpr float ROUTE_CLEARANCE=geometry::kRouteClearance;
constexpr float DIRECT_CORRIDOR_MAX_RASTERS=8.0f;
constexpr float DIRECT_CORRIDOR_MIN_BASE_Y=0.0f;
constexpr std::size_t NONE_INDEX=std::numeric_limits<std::size_t>::max();

inline Vec3 rotate_y(Vec3 v,float degrees){
    const float a=radians(degrees);
    const float c=std::cos(a),s=std::sin(a);
    return {v.x*c+v.z*s,v.y,-v.x*s+v.z*c};
}
inline Vec3 flatten(Vec3 v){ v.y=0.0f; return v; }
inline Vec3 flat_normal(Vec3 v){ return normalize(flatten(v)); }
inline Vec3 perp_xz(Vec3 v){ v=flat_normal(v); return {-v.z,0.0f,v.x}; }
inline float distance_xz(Vec3 a,Vec3 b){
    const float dx=a.x-b.x,dz=a.z-b.z;
    return std::sqrt(dx*dx+dz*dz);
}
inline float point_segment_distance_xz(Vec3 p,Vec3 a,Vec3 b){
    const float dx=b.x-a.x,dz=b.z-a.z;
    const float den=dx*dx+dz*dz;
    if(den<0.0001f) return distance_xz(p,a);
    const float t=std::clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/den,0.0f,1.0f);
    return distance_xz(p,{a.x+dx*t,0,a.z+dz*t});
}
inline Vec3 closest_point_segment_xz(Vec3 p,Vec3 a,Vec3 b){
    const float dx=b.x-a.x,dz=b.z-a.z;
    const float den=dx*dx+dz*dz;
    if(den<0.0001f) return {a.x,0,a.z};
    const float t=std::clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/den,0.0f,1.0f);
    return {a.x+dx*t,a.y+(b.y-a.y)*t,a.z+dz*t};
}
inline bool segment_intersection_xz(Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3& out,float endpoint_margin=0.02f){
    const float rx=b.x-a.x,rz=b.z-a.z;
    const float sx=d.x-c.x,sz=d.z-c.z;
    const float den=rx*sz-rz*sx;
    if(std::abs(den)<1.0e-5f) return false;
    const float qx=c.x-a.x,qz=c.z-a.z;
    const float t=(qx*sz-qz*sx)/den;
    const float u=(qx*rz-qz*rx)/den;
    if(t<=endpoint_margin || t>=1.0f-endpoint_margin || u<=endpoint_margin || u>=1.0f-endpoint_margin) return false;
    out={a.x+rx*t,a.y+(b.y-a.y)*t,a.z+rz*t};
    return true;
}
inline bool mergeable_corridor_crossing(Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3& out){
    Vec3 ab=flat_normal(b-a),cd=flat_normal(d-c);
    if(length(ab)<0.001f || length(cd)<0.001f) return false;
    // Shallow intersections create long overlapping pressure shells. Only a
    // genuine branch/crossing angle is automatically merged into a junction.
    if(std::abs(dot(ab,cd))>0.82f) return false;
    return segment_intersection_xz(a,b,c,d,out,0.025f);
}
inline bool endpoint_uses_habitat(const CorridorEndpoint& e,std::size_t index){
    return e.kind==CorridorEndpointKind::Habitat && e.index==index;
}
inline bool endpoint_uses_resource(const CorridorEndpoint& e,std::size_t index){
    return e.kind==CorridorEndpointKind::Resource && e.index==index;
}


void tri(std::vector<Vec3>& dst,Vec3 a,Vec3 b,Vec3 c);
void edge(std::vector<Vec3>& dst,Vec3 a,Vec3 b);
std::vector<Vec2> corridor_cross_section(float scale=1.0f);
Vec3 ring_vertex(Vec3 center,Vec3 side,const Vec2& q);
void append_swept_shell(CorridorMesh& mesh,const std::vector<Vec3>& path);
void append_walkway(CorridorMesh& mesh,const std::vector<Vec3>& path);
void append_ring(CorridorMesh& mesh,Vec3 center,Vec3 tangent,float scale=1.055f);
void append_ribs(CorridorMesh& mesh,const std::vector<Vec3>& path);
void append_pressure_services(CorridorMesh& mesh,const std::vector<Vec3>& path);
void append_deck_panels(CorridorMesh& mesh,const std::vector<Vec3>& path);
void append_docking_collar(CorridorMesh& mesh,const CorridorEndpoint& endpoint);

std::optional<CorridorEndpoint> resolve_anchor(const CorridorBuildAnchor& anchor,Vec3 other,
                                               const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                                               const std::vector<CorridorConnection>& existing);
std::optional<CorridorEndpoint> resolve_anchor(const CorridorBuildAnchor& anchor,Vec3 other,
                                               const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                                               const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library,
                                               const std::vector<CorridorConnection>& existing);
std::vector<Vec3> smooth_route(const CorridorEndpoint& a,const CorridorEndpoint& b,
                               const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library);
std::vector<Vec3> seamless_end_splice_route(Vec3 a,Vec3 outward_a,Vec3 b,Vec3 outward_b);
CorridorMesh build_mesh(const CorridorEndpoint& a,const CorridorEndpoint& b,const std::vector<Vec3>& path,
                        const std::vector<CorridorConnection>& existing,std::size_t self_index=NONE_INDEX);
CorridorMesh build_endpoint_fitting(const CorridorEndpoint& endpoint);
bool corridor_path_self_overlaps(const std::vector<Vec3>& path);
void rebuild_all_meshes(std::vector<CorridorConnection>& connections);
std::optional<CorridorConnection> make_generic(const CorridorBuildAnchor& a_anchor,const CorridorBuildAnchor& b_anchor,
                                               CorridorConnectionKind kind,
                                               const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                                               const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library,
                                               const std::vector<CorridorConnection>& existing,float playfield_half_extent);
std::optional<CorridorConnection> make_generic_via_waypoint(const CorridorBuildAnchor& a_anchor,const CorridorBuildAnchor& b_anchor,
                                                            eng::Vec3 waypoint,CorridorConnectionKind kind,
                                                            const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                                                            const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library,
                                                            const std::vector<CorridorConnection>& existing,float playfield_half_extent);
Vec3 nearest_path_point(const std::vector<Vec3>& path,Vec3 p);
Vec3 nearest_path_tangent(const std::vector<Vec3>& path,Vec3 p);
std::pair<Vec3,Vec3> closest_pair(const std::vector<Vec3>& a,const std::vector<Vec3>& b);
float path_length(const std::vector<Vec3>& path);
void optimize_shared_entrance_groups(std::vector<CorridorConnection>& connections,
                                     const std::vector<PlacedHabitat>& habitats,
                                     const HabitatModelLibrary& library);

} // namespace game::corridor_detail
