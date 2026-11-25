#pragma once
#include "game/world/habitat_mesh.hpp"
#include "game/world/geometry_standards.hpp"
#include "game/world/resource_mesh.hpp"
#include <limits>
namespace game::interior_layout {
using runtime::Vec3;
inline std::array<std::vector<Vec3>*,8> layers(HabitatMesh& m){return {&m.interior,&m.details,&m.service_metal,&m.fabric,&m.displays,&m.lamps,&m.markings,&m.edges};}
inline std::array<const std::vector<Vec3>*,8> layers(const HabitatMesh& m){return {&m.interior,&m.details,&m.service_metal,&m.fabric,&m.displays,&m.lamps,&m.markings,&m.edges};}
class StationScope {
    HabitatMesh& mesh_;InteriorStation station_;
public:
    explicit StationScope(HabitatMesh& mesh,StationKind kind=StationKind::Equipment,std::size_t slot=0):mesh_(mesh){
        station_.kind=kind;station_.slot=slot;station_.screen_first=mesh.screens.size();
        const auto list=layers(mesh);for(std::size_t i=0;i<list.size();++i)station_.first[i]=list[i]->size();
    }
    ~StationScope(){
        const auto list=layers(mesh_);bool any=false;
        station_.low={1e9f,1e9f,1e9f};station_.high={-1e9f,-1e9f,-1e9f};
        for(std::size_t i=0;i<list.size();++i){station_.count[i]=list[i]->size()-station_.first[i];
            if(i==5||i==7)continue; // Ceiling lamps and guide lines do not define a work station's footprint.
            for(std::size_t j=station_.first[i];j<list[i]->size();++j){const auto p=(*list[i])[j];any=true;
                station_.low={std::min(station_.low.x,p.x),std::min(station_.low.y,p.y),std::min(station_.low.z,p.z)};
                station_.high={std::max(station_.high.x,p.x),std::max(station_.high.y,p.y),std::max(station_.high.z,p.z)};
            }
        }
        station_.screen_count=mesh_.screens.size()-station_.screen_first;
        if(any)mesh_.stations.push_back(station_);
    }
    StationScope(const StationScope&)=delete;
};
class ResourceStationScope {
    ResourceMesh& mesh_;InteriorStation station_;
public:
    explicit ResourceStationScope(ResourceMesh& mesh):mesh_(mesh){station_.first[1]=mesh.details.size();station_.first[7]=mesh.edges.size();}
    ~ResourceStationScope(){
        station_.count[1]=mesh_.details.size()-station_.first[1];station_.count[7]=mesh_.edges.size()-station_.first[7];
        station_.low={1e9f,1e9f,1e9f};station_.high={-1e9f,-1e9f,-1e9f};
        for(std::size_t j=station_.first[1];j<mesh_.details.size();++j){const auto p=mesh_.details[j];
            station_.low={std::min(station_.low.x,p.x),std::min(station_.low.y,p.y),std::min(station_.low.z,p.z)};
            station_.high={std::max(station_.high.x,p.x),std::max(station_.high.y,p.y),std::max(station_.high.z,p.z)};
        }
        if(station_.count[1])mesh_.stations.push_back(station_);
    }
};

struct Reservation {
    Vec3 wall{},normal{0,0,1};
    bool hits(Vec3 low,Vec3 high) const {
        if(high.y<wall.y-.05f || low.y>wall.y+geometry::kDoorClearHeight+.25f)return false;
        const Vec3 side{-normal.z,0,normal.x},center=wall-normal*1.10f;
        const Vec3 c=(low+high)*.5f,half=(high-low)*.5f,delta=c-center;
        const float reach_side=std::abs(side.x)*half.x+std::abs(side.z)*half.z;
        const float reach_depth=std::abs(normal.x)*half.x+std::abs(normal.z)*half.z;
        return std::abs(runtime::dot(delta,side))<=geometry::kDoorClearWidth*.5f+.20f+reach_side &&
            std::abs(runtime::dot(delta,normal))<=1.55f+reach_depth;
    }
};
inline bool can_place(const std::vector<Reservation>& reserved,Vec3 low,Vec3 high){
    for(const auto& area:reserved)if(area.hits(low,high))return false;
    return true;
}
struct Cache {const HabitatMesh* source{};std::vector<Reservation> reserved;HabitatMesh mesh;bool remove_machines{};};
inline const HabitatMesh& resolve(Cache& cache,const HabitatMesh& source,std::vector<Reservation> reserved,std::size_t grow_bays,bool remove_machines=false){
    if(reserved.empty()&&!remove_machines)return source;
    bool same=cache.source==&source&&cache.reserved.size()==reserved.size()&&cache.remove_machines==remove_machines;
    if(same)for(std::size_t i=0;i<reserved.size();++i)if(runtime::length(reserved[i].wall-cache.reserved[i].wall)>1e-5f||runtime::length(reserved[i].normal-cache.reserved[i].normal)>1e-5f){same=false;break;}
    if(same)return cache.mesh;
    cache.source=&source;cache.reserved=std::move(reserved);cache.remove_machines=remove_machines;cache.mesh=source;auto& out=cache.mesh;out.blocked_grow_bays=source.blocked_grow_bays;
    for(auto& station:out.stations){station.active=can_place(cache.reserved,station.low,station.high)&&!(remove_machines&&(station.kind==StationKind::Equipment||station.kind==StationKind::Grow));
        if(!station.active&&station.kind==StationKind::Grow&&grow_bays)out.blocked_grow_bays|=std::uint16_t(1u<<(station.slot%grow_bays));
    }
    for(auto& station:out.stations){
        if(station.kind==StationKind::Grow&&grow_bays&&(out.blocked_grow_bays&(1u<<(station.slot%grow_bays))))station.active=false;
        if(!station.active&&station.kind==StationKind::Bed&&station.slot<out.crew_bunks.size())out.crew_bunks[station.slot].available=false;
    }
    const auto src=layers(source);const auto dst=layers(out);
    for(std::size_t layer=0;layer<src.size();++layer){
        std::vector<bool> removed(src[layer]->size());
        for(const auto& station:out.stations)if(!station.active){
            const auto end=std::min(removed.size(),station.first[layer]+station.count[layer]);
            for(std::size_t i=station.first[layer];i<end;++i)removed[i]=true;
        }
        dst[layer]->clear();for(std::size_t i=0;i<removed.size();++i)if(!removed[i])dst[layer]->push_back((*src[layer])[i]);
    }
    out.screens.clear();for(std::size_t i=0;i<source.screens.size();++i){
        bool removed=false;for(const auto& station:out.stations)if(!station.active&&i>=station.screen_first&&i<station.screen_first+station.screen_count)removed=true;
        if(!removed)out.screens.push_back(source.screens[i]);
    }
    return out;
}
struct ResourceCache {const ResourceMesh* source{};std::vector<Reservation> reserved;ResourceMesh mesh;};
inline const ResourceMesh& resolve(ResourceCache& cache,const ResourceMesh& source,std::vector<Reservation> reserved){
    if(reserved.empty())return source;
    bool same=cache.source==&source&&cache.reserved.size()==reserved.size();
    if(same)for(std::size_t i=0;i<reserved.size();++i)if(runtime::length(reserved[i].wall-cache.reserved[i].wall)>1e-5f||runtime::length(reserved[i].normal-cache.reserved[i].normal)>1e-5f){same=false;break;}
    if(same)return cache.mesh;
    cache.source=&source;cache.reserved=std::move(reserved);cache.mesh=source;
    for(auto& station:cache.mesh.stations)station.active=can_place(cache.reserved,station.low,station.high);
    for(auto layer:{1,7}){
        const auto& src=layer==1?source.details:source.edges;auto& dst=layer==1?cache.mesh.details:cache.mesh.edges;
        std::vector<bool> removed(src.size());
        for(const auto& station:cache.mesh.stations)if(!station.active)for(std::size_t i=station.first[layer];i<std::min(removed.size(),station.first[layer]+station.count[layer]);++i)removed[i]=true;
        dst.clear();for(std::size_t i=0;i<removed.size();++i)if(!removed[i])dst.push_back(src[i]);
    }
    return cache.mesh;
}

}
