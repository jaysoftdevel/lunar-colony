#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/version.hpp"
#include "game/world/geometry_standards.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <system_error>

namespace game {
namespace {
namespace fs=std::filesystem;
constexpr std::array<char,8> kSaveMagic{{'G','A','M','E','S','A','V','1'}};
constexpr std::uint32_t kSaveSchema=13;
constexpr std::uint32_t kMinSaveSchema=1;
constexpr std::uint64_t kMaxVectorItems=2'000'000;
constexpr std::uint64_t kMaxObjects=100'000;

struct Writer {
    std::ofstream out;
    explicit Writer(const fs::path& path):out(path,std::ios::binary|std::ios::trunc){}
    template<class T> void pod(const T& value){ out.write(reinterpret_cast<const char*>(&value),sizeof(T)); }
    void boolean(bool value){ const std::uint8_t v=value?1:0; pod(v); }
    void string(const std::string& value){
        const std::uint32_t n=(std::uint32_t)std::min<std::size_t>(value.size(),4096);
        pod(n); if(n) out.write(value.data(),n);
    }
    bool ok() const { return out.good(); }
};

struct Reader {
    std::ifstream in;
    bool valid{true};
    // Schema 11 adds Aluminum as the eighth persisted material. Older saves
    // have seven ResourceBundle fields; keeping the count on the reader lets
    // every nested bundle/plan/robot loader remain backward-compatible.
    std::size_t material_resource_count{7};
    explicit Reader(const fs::path& path):in(path,std::ios::binary){ valid=in.good(); }
    template<class T> T pod(){ T value{}; if(!valid || !in.read(reinterpret_cast<char*>(&value),sizeof(T))) valid=false; return value; }
    bool boolean(){ return pod<std::uint8_t>()!=0; }
    std::string string(std::uint32_t max_len=4096){
        const auto n=pod<std::uint32_t>();
        if(!valid || n>max_len){ valid=false; return {}; }
        std::string value(n,'\0');
        if(n && !in.read(value.data(),n)) valid=false;
        return value;
    }
    bool ok() const { return valid && in.good(); }
};

void write_vec3(Writer& w,const eng::Vec3& v){ w.pod(v.x); w.pod(v.y); w.pod(v.z); }
eng::Vec3 read_vec3(Reader& r){ return {r.pod<float>(),r.pod<float>(),r.pod<float>()}; }

void write_vec3s(Writer& w,const std::vector<eng::Vec3>& values){
    const std::uint64_t n=values.size(); w.pod(n); for(const auto& v:values) write_vec3(w,v);
}
std::vector<eng::Vec3> read_vec3s(Reader& r){
    const auto n=r.pod<std::uint64_t>();
    if(!r.ok() || n>kMaxVectorItems){ r.valid=false; return {}; }
    std::vector<eng::Vec3> values; values.reserve((std::size_t)n);
    for(std::uint64_t i=0;i<n && r.valid;++i) values.push_back(read_vec3(r));
    return values;
}

void write_bundle(Writer& w,const ResourceBundle& b){
    for(std::size_t i=0;i<kMaterialResourceCount;++i) w.pod(b.amount((MaterialResource)i));
}
ResourceBundle read_bundle(Reader& r){
    ResourceBundle b{};
    const std::size_t count=std::min(r.material_resource_count,kMaterialResourceCount);
    for(std::size_t i=0;i<count;++i) b.amount((MaterialResource)i)=r.pod<std::int64_t>();
    return b;
}

void add_bundle(ResourceBundle& dst,const ResourceBundle& src){
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto material=(MaterialResource)i;
        dst.amount(material)+=src.amount(material);
    }
}

void write_plan(Writer& w,const ConstructionResourcePlan& p){
    w.pod(p.total_steps);
    for(const auto& step:p.steps) write_bundle(w,step);
}
ConstructionResourcePlan read_plan(Reader& r){
    ConstructionResourcePlan p{};
    p.total_steps=r.pod<std::uint8_t>();
    if(p.total_steps==0 || p.total_steps>kMaxConstructionResourceSteps) r.valid=false;
    for(auto& step:p.steps) step=read_bundle(r);
    return p;
}

ResourceBundle plan_range(const ConstructionResourcePlan& plan,std::uint8_t begin,std::uint8_t end){
    ResourceBundle out{};
    const std::size_t first=std::min<std::size_t>(begin,plan.steps.size());
    const std::size_t last=std::min<std::size_t>({(std::size_t)end,(std::size_t)plan.total_steps,plan.steps.size()});
    for(std::size_t si=first;si<last;++si)
        for(std::size_t ri=0;ri<kMaterialResourceCount;++ri){
            const auto material=(MaterialResource)ri;
            out.amount(material)+=plan.steps[si].amount(material);
        }
    return out;
}

void distribute_plan_range(ConstructionResourcePlan& plan,const ResourceBundle& amount,
                           std::uint8_t begin,std::uint8_t count){
    if(count==0) return;
    for(std::size_t ri=0;ri<kMaterialResourceCount;++ri){
        const auto material=(MaterialResource)ri;
        const std::int64_t total=std::max<std::int64_t>(0,amount.amount(material));
        const std::int64_t base=total/(std::int64_t)count;
        std::int64_t remainder=total%(std::int64_t)count;
        for(std::uint8_t offset=0;offset<count;++offset){
            const std::size_t index=(std::size_t)begin+(std::size_t)offset;
            if(index>=plan.steps.size()) break;
            plan.steps[index].amount(material)=base+(remainder>0?1:0);
            if(remainder>0) --remainder;
        }
    }
}

std::uint8_t remapped_completed_steps(std::uint8_t old_step,std::uint8_t old_total,std::uint8_t new_total){
    if(old_total==0) return 0;
    if(old_step>=old_total) return new_total;
    if(old_step==0) return 0;
    const float scaled=(float)old_step*(float)new_total/(float)old_total;
    const int rounded=(int)std::lround(scaled);
    return (std::uint8_t)std::clamp(rounded,1,std::max(1,(int)new_total-1));
}

void compact_construction_plan(ConstructionResourcePlan& plan,std::uint8_t old_step,
                               std::uint8_t new_step,std::uint8_t new_total){
    const std::uint8_t old_total=std::max<std::uint8_t>(1,plan.total_steps);
    const ResourceBundle consumed=plan_range(plan,0,std::min(old_step,old_total));
    const ResourceBundle remaining=plan_range(plan,std::min(old_step,old_total),old_total);
    ConstructionResourcePlan compact{};
    compact.total_steps=std::max<std::uint8_t>(1,new_total);
    distribute_plan_range(compact,consumed,0,new_step);
    distribute_plan_range(compact,remaining,new_step,(std::uint8_t)(compact.total_steps-new_step));
    plan=compact;
}

void compact_construction(ConstructionState& state,ConstructionResourcePlan& plan,std::uint8_t new_total){
    const std::uint8_t old_total=std::max<std::uint8_t>(1,plan.total_steps);
    if(old_total==new_total){ state.step=std::min(state.step,new_total); return; }
    const std::uint8_t old_step=std::min(state.step,old_total);
    const std::uint8_t new_step=remapped_completed_steps(old_step,old_total,new_total);
    compact_construction_plan(plan,old_step,new_step,new_total);
    state.step=new_step;
    state.step_elapsed=state.step>=new_total?0.0f:std::clamp(state.step_elapsed,0.0f,kConstructionStepSeconds*0.999f);
}

void compact_construction(StagedConstructionState& state,ConstructionResourcePlan& plan,std::uint8_t new_total){
    const std::uint8_t old_total=std::max<std::uint8_t>(1,state.total_steps);
    if(old_total==new_total && plan.total_steps==new_total){ state.step=std::min(state.step,new_total); return; }
    const std::uint8_t old_step=std::min(state.step,old_total);
    const std::uint8_t new_step=remapped_completed_steps(old_step,old_total,new_total);
    // Plan total normally mirrors state.total_steps. Use its own old boundary
    // when it does not, but preserve the same completed-stage proportion.
    compact_construction_plan(plan,std::min<std::uint8_t>(old_step,plan.total_steps),new_step,new_total);
    state.total_steps=new_total;
    state.step=new_step;
    state.step_elapsed=state.step>=new_total?0.0f:std::clamp(state.step_elapsed,0.0f,kConstructionStepSeconds*0.999f);
}

void write_construction(Writer& w,const ConstructionState& c){ w.pod(c.step); w.pod(c.step_elapsed); }
ConstructionState read_construction(Reader& r){
    ConstructionState c{}; c.step=r.pod<std::uint8_t>(); c.step_elapsed=r.pod<float>();
    // Older 0.40.x saves used ten structure stages. Accept that persisted
    // range here and compact it to the current stage count after the complete
    // object (including its resource plan) has been read.
    if(c.step>kMaxConstructionResourceSteps) r.valid=false;
    return c;
}
void write_staged(Writer& w,const StagedConstructionState& c){ w.pod(c.step); w.pod(c.total_steps); w.pod(c.step_elapsed); }
StagedConstructionState read_staged(Reader& r){
    const auto step=r.pod<std::uint8_t>();
    const auto total=r.pod<std::uint8_t>();
    const auto elapsed=r.pod<float>();
    if(total==0 || total>kMaxConstructionResourceSteps || step>total) r.valid=false;
    StagedConstructionState c(total?total:1); c.step=step; c.step_elapsed=elapsed; return c;
}

void write_door(Writer& w,const CorridorDoor& d){
    write_vec3(w,d.position); write_vec3(w,d.wall_position); write_vec3(w,d.normal);
    w.boolean(d.generated); w.pod((std::int32_t)d.entrance_index); write_vec3s(w,d.wall_profile);
}
CorridorDoor read_door(Reader& r){
    CorridorDoor d{}; d.position=read_vec3(r); d.wall_position=read_vec3(r); d.normal=read_vec3(r);
    d.generated=r.boolean(); d.entrance_index=(int)r.pod<std::int32_t>(); d.wall_profile=read_vec3s(r); return d;
}

void write_corridor_endpoint(Writer& w,const CorridorEndpoint& e){
    w.pod((std::uint8_t)e.kind); w.pod((std::uint64_t)e.index); write_vec3(w,e.position); write_vec3(w,e.tangent);
    write_vec3(w,e.host_tangent); write_vec3(w,e.host_position); write_door(w,e.door);
    w.boolean(e.suppress_open_door); w.boolean(e.seamless_splice); w.boolean(e.surface_matched_junction);
}
CorridorEndpoint read_corridor_endpoint(Reader& r){
    CorridorEndpoint e{}; e.kind=(CorridorEndpointKind)r.pod<std::uint8_t>(); e.index=(std::size_t)r.pod<std::uint64_t>();
    e.position=read_vec3(r); e.tangent=read_vec3(r); e.host_tangent=read_vec3(r); e.host_position=read_vec3(r); e.door=read_door(r);
    e.suppress_open_door=r.boolean(); e.seamless_splice=r.boolean(); e.surface_matched_junction=r.boolean(); return e;
}
void write_corridor_mesh(Writer& w,const CorridorMesh& m){ write_vec3s(w,m.shell); write_vec3s(w,m.details); write_vec3s(w,m.edges); }
CorridorMesh read_corridor_mesh(Reader& r){ CorridorMesh m{}; m.shell=read_vec3s(r); m.details=read_vec3s(r); m.edges=read_vec3s(r); return m; }
void write_corridor(Writer& w,const CorridorConnection& c){
    w.pod((std::uint8_t)c.kind); write_corridor_endpoint(w,c.endpoint_a); write_corridor_endpoint(w,c.endpoint_b); write_vec3s(w,c.path);
    write_corridor_mesh(w,c.mesh); write_staged(w,c.construction); write_plan(w,c.resource_plan); write_bundle(w,c.site_buffer);
    w.pod(c.placement_sequence);
}
CorridorConnection read_corridor(Reader& r,bool with_site_buffer,bool with_placement_sequence=false){
    CorridorConnection c{}; c.kind=(CorridorConnectionKind)r.pod<std::uint8_t>(); c.endpoint_a=read_corridor_endpoint(r); c.endpoint_b=read_corridor_endpoint(r);
    c.path=read_vec3s(r); c.mesh=read_corridor_mesh(r); c.construction=read_staged(r); c.resource_plan=read_plan(r);
    if(with_site_buffer) c.site_buffer=read_bundle(r);
    if(with_placement_sequence) c.placement_sequence=r.pod<std::uint64_t>();
    return c;
}

void write_utility_endpoint(Writer& w,const UtilityEndpoint& e){ w.pod((std::uint8_t)e.kind); w.pod((std::uint64_t)e.index); write_vec3(w,e.position); write_vec3(w,e.normal); }
UtilityEndpoint read_utility_endpoint(Reader& r){ UtilityEndpoint e{}; e.kind=(UtilityEndpointKind)r.pod<std::uint8_t>(); e.index=(std::size_t)r.pod<std::uint64_t>(); e.position=read_vec3(r); e.normal=read_vec3(r); return e; }
void write_utility_mesh(Writer& w,const UtilityMesh& m){ write_vec3s(w,m.shell); write_vec3s(w,m.details); write_vec3s(w,m.edges); }
UtilityMesh read_utility_mesh(Reader& r){ UtilityMesh m{}; m.shell=read_vec3s(r); m.details=read_vec3s(r); m.edges=read_vec3s(r); return m; }
void write_utility(Writer& w,const UtilityConnection& c){
    w.pod((std::uint8_t)c.kind); write_utility_endpoint(w,c.endpoint_a); write_utility_endpoint(w,c.endpoint_b); write_vec3s(w,c.path); write_vec3s(w,c.junctions);
    write_utility_mesh(w,c.mesh); write_staged(w,c.construction); write_plan(w,c.resource_plan); write_bundle(w,c.site_buffer);
    w.pod(c.placement_sequence);
}
UtilityConnection read_utility(Reader& r,bool with_site_buffer,bool with_placement_sequence=false){
    UtilityConnection c{}; c.kind=(UtilityKind)r.pod<std::uint8_t>(); c.endpoint_a=read_utility_endpoint(r); c.endpoint_b=read_utility_endpoint(r);
    c.path=read_vec3s(r); c.junctions=read_vec3s(r); c.mesh=read_utility_mesh(r); c.construction=read_staged(r); c.resource_plan=read_plan(r);
    if(with_site_buffer) c.site_buffer=read_bundle(r);
    if(with_placement_sequence) c.placement_sequence=r.pod<std::uint64_t>();
    return c;
}

void write_walkway_endpoint(Writer& w,const WalkwayEndpoint& e){
    w.pod((std::uint8_t)e.kind); w.pod((std::uint64_t)e.index); write_vec3(w,e.position); write_vec3(w,e.normal); write_door(w,e.door);
    w.boolean(e.landing_pad); w.boolean(e.generated_access);
}
WalkwayEndpoint read_walkway_endpoint(Reader& r){
    WalkwayEndpoint e{}; e.kind=(WalkwayEndpointKind)r.pod<std::uint8_t>(); e.index=(std::size_t)r.pod<std::uint64_t>(); e.position=read_vec3(r); e.normal=read_vec3(r);
    e.door=read_door(r); e.landing_pad=r.boolean(); e.generated_access=r.boolean(); return e;
}
void write_walkway_junction(Writer& w,const WalkwayJunction& j){ write_vec3(w,j.position); write_vec3s(w,j.arms); w.pod(j.radius); w.boolean(j.owns_patch); }
WalkwayJunction read_walkway_junction(Reader& r){ WalkwayJunction j{}; j.position=read_vec3(r); j.arms=read_vec3s(r); j.radius=r.pod<float>(); j.owns_patch=r.boolean(); return j; }
void write_walkway_crossing(Writer& w,const WalkwayCorridorCrossing& c){ w.pod((std::uint64_t)c.corridor_index); write_vec3(w,c.center); write_vec3(w,c.walkway_tangent); write_vec3(w,c.corridor_tangent); write_vec3(w,c.entrance_a); write_vec3(w,c.entrance_b); }
WalkwayCorridorCrossing read_walkway_crossing(Reader& r){ WalkwayCorridorCrossing c{}; c.corridor_index=(std::size_t)r.pod<std::uint64_t>(); c.center=read_vec3(r); c.walkway_tangent=read_vec3(r); c.corridor_tangent=read_vec3(r); c.entrance_a=read_vec3(r); c.entrance_b=read_vec3(r); return c; }
void write_walkway_bridge(Writer& w,const WalkwaySupplyBridge& b){ w.pod((std::uint64_t)b.utility_index); write_vec3(w,b.center); write_vec3(w,b.tangent); w.pod(b.path_distance); }
WalkwaySupplyBridge read_walkway_bridge(Reader& r){ WalkwaySupplyBridge b{}; b.utility_index=(std::size_t)r.pod<std::uint64_t>(); b.center=read_vec3(r); b.tangent=read_vec3(r); b.path_distance=r.pod<float>(); return b; }
void write_walkway_mesh(Writer& w,const WalkwayMesh& m){ write_vec3s(w,m.deck); write_vec3s(w,m.details); write_vec3s(w,m.edges); }
WalkwayMesh read_walkway_mesh(Reader& r){ WalkwayMesh m{}; m.deck=read_vec3s(r); m.details=read_vec3s(r); m.edges=read_vec3s(r); return m; }
void write_walkway(Writer& w,const WalkwayConnection& c){
    write_walkway_endpoint(w,c.endpoint_a); write_walkway_endpoint(w,c.endpoint_b); write_vec3s(w,c.path);
    w.pod((std::uint64_t)c.junctions.size()); for(const auto& j:c.junctions) write_walkway_junction(w,j);
    w.pod((std::uint64_t)c.corridor_crossings.size()); for(const auto& x:c.corridor_crossings) write_walkway_crossing(w,x);
    w.pod((std::uint64_t)c.supply_bridges.size()); for(const auto& b:c.supply_bridges) write_walkway_bridge(w,b);
    write_walkway_mesh(w,c.mesh); write_staged(w,c.construction); write_plan(w,c.resource_plan); write_bundle(w,c.site_buffer);
    w.boolean(c.carries_power); w.boolean(c.carries_water); w.boolean(c.pressurized);
    w.pod(c.placement_sequence);
}
WalkwayConnection read_walkway(Reader& r,bool with_site_buffer,bool with_placement_sequence=false){
    WalkwayConnection c{}; c.endpoint_a=read_walkway_endpoint(r); c.endpoint_b=read_walkway_endpoint(r); c.path=read_vec3s(r);
    auto n=r.pod<std::uint64_t>(); if(n>kMaxObjects) r.valid=false; c.junctions.reserve((std::size_t)std::min(n,kMaxObjects)); for(std::uint64_t i=0;i<n&&r.valid;++i)c.junctions.push_back(read_walkway_junction(r));
    n=r.pod<std::uint64_t>(); if(n>kMaxObjects) r.valid=false; c.corridor_crossings.reserve((std::size_t)std::min(n,kMaxObjects)); for(std::uint64_t i=0;i<n&&r.valid;++i)c.corridor_crossings.push_back(read_walkway_crossing(r));
    n=r.pod<std::uint64_t>(); if(n>kMaxObjects) r.valid=false; c.supply_bridges.reserve((std::size_t)std::min(n,kMaxObjects)); for(std::uint64_t i=0;i<n&&r.valid;++i)c.supply_bridges.push_back(read_walkway_bridge(r));
    c.mesh=read_walkway_mesh(r); c.construction=read_staged(r); c.resource_plan=read_plan(r);
    if(with_site_buffer) c.site_buffer=read_bundle(r);
    c.carries_power=r.boolean(); c.carries_water=r.boolean(); c.pressurized=r.boolean();
    if(with_placement_sequence) c.placement_sequence=r.pod<std::uint64_t>();
    return c;
}

std::string trimmed_name(std::string value){
    while(!value.empty() && std::isspace((unsigned char)value.front())) value.erase(value.begin());
    while(!value.empty() && std::isspace((unsigned char)value.back())) value.pop_back();
    return value;
}
std::string filename_for_name(const std::string& name){
    std::string stem; stem.reserve(name.size());
    for(unsigned char ch:name){
        if(std::isalnum(ch) || ch=='-' || ch=='_') stem.push_back((char)ch);
        else if(std::isspace(ch)) stem.push_back('_');
        else if(ch=='.') stem.push_back('-');
        else if(ch<128) stem.push_back('_');
    }
    while(stem.find("__")!=std::string::npos) stem.replace(stem.find("__"),2,"_");
    if(stem.empty()) stem="savegame";
    if(stem.size()>48) stem.resize(48);
    return stem+".gmsave";
}

bool read_metadata(const fs::path& path,std::string& name,std::string& version){
    Reader r(path);
    std::array<char,8> magic{};
    if(!r.in.read(magic.data(),magic.size()) || magic!=kSaveMagic) return false;
    const auto schema=r.pod<std::uint32_t>();
    if(schema<kMinSaveSchema || schema>kSaveSchema) return false;
    version=r.string(64); name=r.string(128);
    return r.valid && !name.empty();
}

template<class T,class ReadFn>
bool read_object_vector(Reader& r,std::vector<T>& out,ReadFn&& fn){
    const auto n=r.pod<std::uint64_t>();
    if(!r.ok() || n>kMaxObjects){ r.valid=false; return false; }
    out.clear(); out.reserve((std::size_t)n);
    for(std::uint64_t i=0;i<n&&r.valid;++i) out.push_back(fn(r));
    return r.valid;
}


void write_builder_robot(Writer& w,const BuilderRobotState& robot){
    w.pod((std::uint8_t)robot.role);
    write_vec3(w,robot.position); w.pod(robot.heading_degrees); w.pod((std::uint8_t)robot.mode);
    w.pod((std::uint8_t)robot.target.kind); w.pod((std::uint64_t)robot.target.index);
    write_bundle(w,robot.cargo); write_bundle(w,robot.site_buffer);
    w.pod(robot.work_elapsed); w.pod(robot.idle_elapsed); w.pod(robot.move_speed_mps); w.pod(robot.payload_capacity_kg);
    w.pod((std::uint64_t)robot.support_robot_index);
    w.boolean(robot.cargo_committed_to_build);
    write_bundle(w,robot.logistics_reservation);
    w.pod(robot.condition);
}

BuilderRobotState read_builder_robot_v5(Reader& r){
    BuilderRobotState robot{};
    robot.role=(RobotRole)r.pod<std::uint8_t>();
    robot.position=read_vec3(r); robot.heading_degrees=r.pod<float>(); robot.mode=(BuilderRobotMode)r.pod<std::uint8_t>();
    robot.target.kind=(ConstructionTargetKind)r.pod<std::uint8_t>(); robot.target.index=(std::size_t)r.pod<std::uint64_t>();
    robot.cargo=read_bundle(r); robot.site_buffer=read_bundle(r);
    robot.work_elapsed=r.pod<float>(); robot.idle_elapsed=r.pod<float>(); robot.move_speed_mps=r.pod<float>(); robot.payload_capacity_kg=r.pod<std::int64_t>();
    robot.support_robot_index=(std::size_t)r.pod<std::uint64_t>();
    robot.cargo_committed_to_build=r.boolean();
    robot.logistics_reservation=read_bundle(r);
    robot.condition=r.pod<float>();
    if((int)robot.role>1 || robot.payload_capacity_kg<=0 || robot.payload_capacity_kg>100000 || !std::isfinite(robot.condition)) r.valid=false;
    robot.condition=std::clamp(robot.condition,0.0f,1.0f);
    return robot;
}

BuilderRobotState read_builder_robot_v4(Reader& r){
    BuilderRobotState robot{};
    robot.role=(RobotRole)r.pod<std::uint8_t>();
    robot.position=read_vec3(r); robot.heading_degrees=r.pod<float>(); robot.mode=(BuilderRobotMode)r.pod<std::uint8_t>();
    robot.target.kind=(ConstructionTargetKind)r.pod<std::uint8_t>(); robot.target.index=(std::size_t)r.pod<std::uint64_t>();
    robot.cargo=read_bundle(r); robot.site_buffer=read_bundle(r);
    robot.work_elapsed=r.pod<float>(); robot.idle_elapsed=r.pod<float>(); robot.move_speed_mps=r.pod<float>(); robot.payload_capacity_kg=r.pod<std::int64_t>();
    robot.support_robot_index=(std::size_t)r.pod<std::uint64_t>();
    robot.cargo_committed_to_build=r.boolean();
    if((int)robot.role>1 || robot.payload_capacity_kg<=0 || robot.payload_capacity_kg>100000) r.valid=false;
    return robot;
}

BuilderRobotState read_builder_robot_legacy(Reader& r){
    BuilderRobotState robot{};
    robot.role=RobotRole::Constructor;
    robot.position=read_vec3(r); robot.heading_degrees=r.pod<float>();
    const auto old_mode=r.pod<std::uint8_t>();
    switch(old_mode){
        case 0: robot.mode=BuilderRobotMode::Idle; break;
        case 1: robot.mode=BuilderRobotMode::Fetching; break;
        case 2: robot.mode=BuilderRobotMode::Delivering; break;
        case 3: robot.mode=BuilderRobotMode::Building; break;
        default: r.valid=false; break;
    }
    robot.target.kind=(ConstructionTargetKind)r.pod<std::uint8_t>(); robot.target.index=(std::size_t)r.pod<std::uint64_t>();
    robot.cargo=read_bundle(r); robot.site_buffer=read_bundle(r);
    robot.work_elapsed=r.pod<float>(); robot.idle_elapsed=r.pod<float>(); robot.move_speed_mps=r.pod<float>(); robot.payload_capacity_kg=r.pod<std::int64_t>();
    robot.cargo_committed_to_build=!robot.cargo.empty();
    if(robot.payload_capacity_kg<=0 || robot.payload_capacity_kg>100000) r.valid=false;
    return robot;
}

template<class T,class WriteFn>
void write_object_vector(Writer& w,const std::vector<T>& values,WriteFn&& fn){
    w.pod((std::uint64_t)values.size()); for(const auto& value:values) fn(w,value);
}

} // namespace

std::string GameApp::save_game_directory() const {
    // Keep savegames with the installed game instead of placing them in the
    // user's global data directory.  /proc/self/exe gives us a stable path
    // even when the game is launched from a different working directory.
    std::error_code ec;
    const fs::path executable=fs::read_symlink("/proc/self/exe",ec);
    if(!ec && !executable.empty())
        return (executable.parent_path()/"savegames").string();
    return (fs::current_path()/"savegames").string();
}

void GameApp::refresh_save_games(){
    save_games_.clear();
    std::error_code ec;
    const fs::path dir(save_game_directory());
    fs::create_directories(dir,ec);
    if(ec) return;
    for(const auto& entry:fs::directory_iterator(dir,ec)){
        if(ec) break;
        if(!entry.is_regular_file() || entry.path().extension()!=".gmsave") continue;
        std::string name,version;
        if(!read_metadata(entry.path(),name,version)) continue;
        save_games_.push_back({name,entry.path().string(),version});
    }
    std::sort(save_games_.begin(),save_games_.end(),[](const SaveGameEntry& a,const SaveGameEntry& b){ return a.name<b.name; });
    load_save_scroll_=std::clamp(load_save_scroll_,0,std::max(0,(int)save_games_.size()-1));
}

bool GameApp::save_game_to_name(const std::string& raw_name,std::string& error){
    const std::string name=trimmed_name(raw_name);
    if(name.empty()){ error="Enter a savegame name."; return false; }
    if(name.size()>64){ error="Savegame name is too long."; return false; }

    std::error_code ec;
    const fs::path dir(save_game_directory());
    fs::create_directories(dir,ec);
    if(ec){ error="Unable to create the savegame directory."; return false; }
    const fs::path final_path=dir/filename_for_name(name);
    const fs::path temp_path=final_path.string()+".tmp";
    Writer w(temp_path);
    if(!w.ok()){ error="Unable to open the savegame file for writing."; return false; }

    w.out.write(kSaveMagic.data(),kSaveMagic.size());
    w.pod(kSaveSchema);
    w.string(std::string(version::current));
    w.string(name);

    w.pod((std::int32_t)playfield_.size());
    const eng::DisplayMode display_mode=app_.current_display_mode();
    w.pod((std::int32_t)display_mode.width);
    w.pod((std::int32_t)display_mode.height);
    w.boolean(app_.fullscreen());
    w.pod((std::int32_t)ui_scale_index_);
    w.boolean(debug_frame_enabled_);
    w.pod((std::uint8_t)habitat_view_mode_);
    w.pod((std::uint8_t)build_catalog_.category());
    w.pod(placement_rotation_degrees_);
    w.pod((std::uint8_t)placement_habitat_size_);
    w.pod((std::uint8_t)placement_storage_size_);
    w.pod((std::uint8_t)placement_resource_size_);
    w.pod((std::uint8_t)game_speed_);

    const CameraState camera_state=camera_.state();
    w.pod(camera_state.distance); w.pod(camera_state.elevation_degrees); w.pod(camera_state.azimuth_degrees); write_vec3(w,camera_state.target);
    const ColonyStatusState colony=status_.state();
    w.pod((std::int32_t)colony.day); w.pod(colony.hour); w.pod((std::int32_t)colony.power_current); w.pod((std::int32_t)colony.power_capacity);
    w.pod((std::int32_t)colony.water_current); w.pod((std::int32_t)colony.water_capacity); w.pod((std::int32_t)colony.crew_current); w.pod((std::int32_t)colony.crew_capacity);
    write_bundle(w,resources_.amounts());

    write_object_vector(w,placed_habitats_,[](Writer& out,const PlacedHabitat& h){
        out.pod((std::uint8_t)h.kind); write_vec3(out,h.position); out.pod(h.rotation_degrees); out.pod((std::uint8_t)h.size);
        write_construction(out,h.construction); write_plan(out,h.resource_plan); write_bundle(out,h.site_buffer);
        out.pod(h.instance_serial);
        out.pod(h.placement_sequence);
    });
    write_object_vector(w,placed_resources_,[](Writer& out,const PlacedResource& item){
        out.pod((std::uint8_t)item.kind); write_vec3(out,item.position); out.pod(item.rotation_degrees); write_construction(out,item.construction); write_plan(out,item.resource_plan); write_bundle(out,item.site_buffer);
        out.pod((std::uint8_t)item.storage_size);
        out.pod((std::uint8_t)item.size);
        out.pod((std::uint64_t)item.stored_boxes.size());
        for(const auto& box:item.stored_boxes){ out.pod((std::uint8_t)box.resource); out.pod(box.amount); out.pod(box.condition); }
        out.pod(item.instance_serial);
        out.pod(item.placement_sequence);
    });
    write_object_vector(w,ground_resource_piles_,[](Writer& out,const GroundResourcePile& pile){
        out.pod((std::uint8_t)pile.resource); out.pod(pile.amount); write_vec3(out,pile.position); out.pod(pile.radius); out.pod(pile.condition);
    });

    write_object_vector(w,corridor_network_.connections(),write_corridor);
    write_object_vector(w,utility_network_.connections(),write_utility);
    write_object_vector(w,walkway_network_.connections(),write_walkway);

    // Save schema 13: persistent global blueprint placement order for FIFO carrier logistics.
    // Save schema 12: v0.40.40 eight-pallet starter depots are migrated to hull-safe boundary clearance.
    // Save schema 11: Aluminum is persisted as the eighth resource in bundles and production state.
    // Save schema 10: persistent type/size-aware serial identities for habitat and resource structures.
    // Save schema 9: supply-channel geometry is restored to a flat, visible, rover-crossable ground profile.
    // Save schema 8: supply-channel geometry was buried below the rover plane.
    // Save schema 7: adds persistent SMALL/STANDARD sizing for non-storage resource structures.
    // Schema 6: blueprint-owned delivered-material buffers plus the mixed fleet.
    // Schema 5 already persisted exact per-robot construction logistics
    // reservations and persistent mechanical condition. This keeps carrier
    // demand accounting synchronized across save/load boundaries.
    const std::uint8_t robot_count=(std::uint8_t)std::clamp<std::size_t>(builder_robots_.size(),1,12);
    w.pod(robot_count);
    for(std::size_t i=0;i<robot_count;++i) write_builder_robot(w,builder_robots_[i]);
    write_vec3(w,starter_stash_center_);
    for(double value:production_fraction_) w.pod(value);
    w.pod(life_support_fraction_seconds_);

    w.out.flush();
    if(!w.ok()){ w.out.close(); fs::remove(temp_path,ec); error="Writing the savegame failed."; return false; }
    w.out.close();
    fs::remove(final_path,ec); ec.clear();
    fs::rename(temp_path,final_path,ec);
    if(ec){ fs::remove(temp_path,ec); error="Unable to finalize the savegame file."; return false; }
    refresh_save_games();
    error.clear();
    return true;
}


std::string GameApp::debug_blueprint_template_path() const {
    std::error_code ec;
    const fs::path executable=fs::read_symlink("/proc/self/exe",ec);
    if(!ec && !executable.empty())
        return (executable.parent_path()/"debug"/"0-40-32-debug-start.gmsave").string();
    return (fs::current_path()/"debug"/"0-40-32-debug-start.gmsave").string();
}

bool GameApp::load_debug_blueprint_template_from_file(const std::string& path,std::string& error){
    Reader r{fs::path(path)};
    std::array<char,8> magic{};
    if(!r.in.read(magic.data(),magic.size()) || magic!=kSaveMagic){ error="Template file is missing or invalid."; return false; }
    const auto source_schema=r.pod<std::uint32_t>();
    if(source_schema<6 || source_schema>kSaveSchema){ error="Unsupported debug-template format."; return false; }
    r.material_resource_count=source_schema>=11?kMaterialResourceCount:7;
    const std::string source_version=r.string(64);
    const std::string template_name=r.string(128);
    (void)source_version;
    (void)template_name;
    if(!r.valid){ error="Debug-template header is damaged."; return false; }

    const int saved_playfield=(int)r.pod<std::int32_t>();
    (void)r.pod<std::int32_t>(); // display width
    (void)r.pod<std::int32_t>(); // display height
    (void)r.boolean();           // fullscreen
    (void)r.pod<std::int32_t>(); // UI scale
    (void)r.boolean();           // debug frame
    (void)r.pod<std::uint8_t>(); // habitat view
    (void)r.pod<std::uint8_t>(); // catalog category
    (void)r.pod<float>();        // placement rotation
    (void)r.pod<std::uint8_t>(); // habitat size
    (void)r.pod<std::uint8_t>(); // storage size
    if(source_schema>=7) (void)r.pod<std::uint8_t>(); // resource size
    if(source_schema>=4) (void)r.pod<std::uint8_t>(); // game speed

    // Camera, colony state and colony resource inventory are deliberately
    // consumed but not imported. The debug option is still a NEW GAME.
    for(int i=0;i<6;++i) (void)r.pod<float>();
    (void)r.pod<std::int32_t>();
    (void)r.pod<float>();
    for(int i=0;i<6;++i) (void)r.pod<std::int32_t>();
    (void)read_bundle(r);

    if(!r.valid || saved_playfield!=64 || playfield_.size()!=64){
        error="Template requires a 64 X 64 playfield.";
        return false;
    }

    std::vector<PlacedHabitat> habitats;
    if(!read_object_vector(r,habitats,[&](Reader& in){
        PlacedHabitat h{};
        h.kind=(HabitatKind)in.pod<std::uint8_t>();
        h.position=read_vec3(in);
        h.rotation_degrees=in.pod<float>();
        h.size=(HabitatSizeTier)in.pod<std::uint8_t>();
        h.construction=read_construction(in);
        h.resource_plan=read_plan(in);
        if(source_schema>=6) h.site_buffer=read_bundle(in);
        if(source_schema>=10) h.instance_serial=in.pod<std::uint32_t>();
        if(source_schema>=13) h.placement_sequence=in.pod<std::uint64_t>();
        return h;
    })){ error="Template habitat data is damaged."; return false; }

    std::vector<PlacedResource> structures;
    if(!read_object_vector(r,structures,[&](Reader& in){
        PlacedResource item{};
        item.kind=(ResourceKind)in.pod<std::uint8_t>();
        item.position=read_vec3(in);
        item.rotation_degrees=in.pod<float>();
        item.construction=read_construction(in);
        item.resource_plan=read_plan(in);
        if(source_schema>=6) item.site_buffer=read_bundle(in);
        item.storage_size=(StorageDomeSize)in.pod<std::uint8_t>();
        item.size=source_schema>=7?(ResourceSizeTier)in.pod<std::uint8_t>():ResourceSizeTier::Standard;
        const auto box_count=in.pod<std::uint64_t>();
        if(box_count>kMaxObjects) in.valid=false;
        item.stored_boxes.reserve((std::size_t)std::min(box_count,kMaxObjects));
        for(std::uint64_t i=0;i<box_count && in.valid;++i){
            StoredResourceBox box{};
            box.resource=(MaterialResource)in.pod<std::uint8_t>();
            box.amount=in.pod<std::int64_t>();
            box.condition=in.pod<float>();
            item.stored_boxes.push_back(box);
        }
        if(source_schema>=10) item.instance_serial=in.pod<std::uint32_t>();
        if(source_schema>=13) item.placement_sequence=in.pod<std::uint64_t>();
        sync_stored_resource_totals(item);
        return item;
    })){ error="Template structure data is damaged."; return false; }

    std::vector<GroundResourcePile> ignored_piles;
    if(!read_object_vector(r,ignored_piles,[&](Reader& in){
        GroundResourcePile pile{};
        pile.resource=(MaterialResource)in.pod<std::uint8_t>();
        pile.amount=in.pod<std::int64_t>();
        pile.position=read_vec3(in);
        pile.radius=in.pod<float>();
        pile.condition=in.pod<float>();
        return pile;
    })){ error="Template resource-pile data is damaged."; return false; }

    std::vector<CorridorConnection> corridors;
    if(!read_object_vector(r,corridors,[&](Reader& in){ return read_corridor(in,source_schema>=6,source_schema>=13); })){
        error="Template corridor data is damaged."; return false;
    }
    std::vector<UtilityConnection> utilities;
    if(!read_object_vector(r,utilities,[&](Reader& in){ return read_utility(in,source_schema>=6,source_schema>=13); })){
        error="Template utility data is damaged."; return false;
    }
    std::vector<WalkwayConnection> walkways;
    if(!read_object_vector(r,walkways,[&](Reader& in){ return read_walkway(in,source_schema>=6,source_schema>=13); })){
        error="Template walkway data is damaged."; return false;
    }
    if(!r.valid){ error="Debug-template construction data is damaged."; return false; }

    // Schema 13 is the first format with a cross-kind placement sequence.
    // Older templates cannot reconstruct their historical interleaving because
    // the legacy file grouped objects by kind, so assign one deterministic FIFO
    // sequence in that serialized order. New placements remain exact thereafter.
    if(source_schema<13){
        std::uint64_t seq=1;
        for(auto& h:habitats) h.placement_sequence=seq++;
        for(auto& item:structures) item.placement_sequence=seq++;
        for(auto& c:corridors) c.placement_sequence=seq++;
        for(auto& c:utilities) c.placement_sequence=seq++;
        for(auto& c:walkways) c.placement_sequence=seq++;
    }

    // Import only the construction layout. Every imported object starts as a
    // fresh blueprint regardless of the source save's build progress or
    // delivered materials. Robots, inventory, ground piles, camera and clock
    // remain those of the newly-created game.
    for(auto& h:habitats){
        h.construction.step=0;
        h.construction.step_elapsed=0.0f;
        h.site_buffer={};
    }
    for(auto& item:structures){
        item.construction.step=0;
        item.construction.step_elapsed=0.0f;
        item.site_buffer={};
        item.stored_boxes.clear();
        sync_stored_resource_totals(item);
    }
    for(auto& c:corridors){
        c.construction.step=0;
        c.construction.step_elapsed=0.0f;
        c.site_buffer={};
    }
    for(auto& c:utilities){
        c.construction.step=0;
        c.construction.step_elapsed=0.0f;
        c.site_buffer={};
    }
    for(auto& c:walkways){
        c.construction.step=0;
        c.construction.step_elapsed=0.0f;
        c.site_buffer={};
    }

    placed_habitats_=std::move(habitats);
    placed_resources_=std::move(structures);
    corridor_network_.restore_connections(std::move(corridors));
    corridor_network_.set_playfield_half_extent((float)playfield_.size()*0.5f);
    corridor_network_.repair_invalid_topology(placed_habitats_,habitat_models_);
    utility_network_.restore_connections(std::move(utilities));
    utility_network_.set_playfield_half_extent((float)playfield_.size()*0.5f);
    walkway_network_.restore_connections(std::move(walkways));

    next_blueprint_placement_sequence_=1;
    auto advance_sequence=[&](std::uint64_t sequence){
        next_blueprint_placement_sequence_=std::max(next_blueprint_placement_sequence_,sequence+1);
    };
    for(const auto& h:placed_habitats_) advance_sequence(h.placement_sequence);
    for(const auto& item:placed_resources_) advance_sequence(item.placement_sequence);
    for(const auto& c:corridor_network_.connections()) advance_sequence(c.placement_sequence);
    for(const auto& c:utility_network_.connections()) advance_sequence(c.placement_sequence);
    for(const auto& c:walkway_network_.connections()) advance_sequence(c.placement_sequence);

    error.clear();
    return true;
}

bool GameApp::load_game_from_file(const std::string& path,std::string& error){
    Reader r{fs::path(path)};
    std::array<char,8> magic{};
    if(!r.in.read(magic.data(),magic.size()) || magic!=kSaveMagic){ error="Invalid savegame file."; return false; }
    const auto source_schema=r.pod<std::uint32_t>();
    if(source_schema<kMinSaveSchema || source_schema>kSaveSchema){ error="Unsupported savegame format."; return false; }
    r.material_resource_count=source_schema>=11?kMaterialResourceCount:7;
    const std::string source_version=r.string(64);
    const std::string save_name=r.string(128);
    (void)source_version;
    if(!r.valid || save_name.empty()){ error="Savegame header is damaged."; return false; }

    const int saved_playfield=(int)r.pod<std::int32_t>();
    const int saved_display_width=(int)r.pod<std::int32_t>();
    const int saved_display_height=(int)r.pod<std::int32_t>();
    const bool saved_fullscreen=r.boolean();
    const int saved_ui_scale=(int)r.pod<std::int32_t>();
    const bool saved_debug=r.boolean();
    const auto saved_view=(HabitatViewMode)r.pod<std::uint8_t>();
    const auto saved_category=(BuildCategory)r.pod<std::uint8_t>();
    const float saved_rotation=r.pod<float>();
    const auto saved_habitat_size=(HabitatSizeTier)r.pod<std::uint8_t>();
    const auto saved_storage_size=(StorageDomeSize)r.pod<std::uint8_t>();
    const auto saved_resource_size=source_schema>=7?(ResourceSizeTier)r.pod<std::uint8_t>():ResourceSizeTier::Standard;
    const auto saved_game_speed=source_schema>=4?(GameSpeed)r.pod<std::uint8_t>():GameSpeed::Normal;
    if(saved_playfield<16 || saved_playfield>512){ error="Savegame playfield size is invalid."; return false; }

    CameraState camera_state{};
    camera_state.distance=r.pod<float>(); camera_state.elevation_degrees=r.pod<float>(); camera_state.azimuth_degrees=r.pod<float>(); camera_state.target=read_vec3(r);
    ColonyStatusState colony{};
    colony.day=(int)r.pod<std::int32_t>(); colony.hour=r.pod<float>(); colony.power_current=(int)r.pod<std::int32_t>(); colony.power_capacity=(int)r.pod<std::int32_t>();
    colony.water_current=(int)r.pod<std::int32_t>(); colony.water_capacity=(int)r.pod<std::int32_t>(); colony.crew_current=(int)r.pod<std::int32_t>(); colony.crew_capacity=(int)r.pod<std::int32_t>();
    const ResourceBundle resource_state=read_bundle(r);

    std::vector<PlacedHabitat> habitats;
    if(!read_object_vector(r,habitats,[&](Reader& in){
        PlacedHabitat h{}; h.kind=(HabitatKind)in.pod<std::uint8_t>(); h.position=read_vec3(in); h.rotation_degrees=in.pod<float>(); h.size=(HabitatSizeTier)in.pod<std::uint8_t>();
        h.construction=read_construction(in); h.resource_plan=read_plan(in); if(source_schema>=6) h.site_buffer=read_bundle(in);
        if(source_schema>=10) h.instance_serial=in.pod<std::uint32_t>();
        if(source_schema>=13) h.placement_sequence=in.pod<std::uint64_t>();
        return h;
    })){ error="Savegame habitat data is damaged."; return false; }

    std::vector<PlacedResource> structures;
    if(!read_object_vector(r,structures,[&](Reader& in){
        PlacedResource item{}; item.kind=(ResourceKind)in.pod<std::uint8_t>(); item.position=read_vec3(in); item.rotation_degrees=in.pod<float>(); item.construction=read_construction(in); item.resource_plan=read_plan(in); if(source_schema>=6) item.site_buffer=read_bundle(in);
        item.storage_size=(StorageDomeSize)in.pod<std::uint8_t>();
        item.size=source_schema>=7?(ResourceSizeTier)in.pod<std::uint8_t>():ResourceSizeTier::Standard;
        const auto n=in.pod<std::uint64_t>(); if(n>kMaxObjects) in.valid=false;
        item.stored_boxes.reserve((std::size_t)std::min(n,kMaxObjects));
        for(std::uint64_t i=0;i<n&&in.valid;++i){ StoredResourceBox box{}; box.resource=(MaterialResource)in.pod<std::uint8_t>(); box.amount=in.pod<std::int64_t>(); box.condition=in.pod<float>(); item.stored_boxes.push_back(box); }
        if(source_schema>=10) item.instance_serial=in.pod<std::uint32_t>();
        if(source_schema>=13) item.placement_sequence=in.pod<std::uint64_t>();
        sync_stored_resource_totals(item); return item;
    })){ error="Savegame structure data is damaged."; return false; }

    // Older saves had no stable structure identity. Assign deterministic serials
    // in saved placement order. Also repair a zero/duplicate serial defensively
    // so every loaded structure name remains unique inside its type/size group.
    for(std::size_t i=0;i<habitats.size();++i){
        auto& h=habitats[i];
        bool duplicate=h.instance_serial==0;
        std::uint32_t next=1;
        for(std::size_t j=0;j<i;++j){
            const auto& previous=habitats[j];
            if(previous.kind!=h.kind || previous.size!=h.size) continue;
            next=std::max(next,previous.instance_serial+1);
            if(previous.instance_serial==h.instance_serial) duplicate=true;
        }
        if(duplicate) h.instance_serial=next;
    }
    for(std::size_t i=0;i<structures.size();++i){
        auto& item=structures[i];
        bool duplicate=item.instance_serial==0;
        std::uint32_t next=1;
        for(std::size_t j=0;j<i;++j){
            const auto& previous=structures[j];
            if(previous.kind!=item.kind) continue;
            const bool same_size=item.kind==ResourceKind::StorageDome?
                previous.storage_size==item.storage_size:previous.size==item.size;
            if(!same_size) continue;
            next=std::max(next,previous.instance_serial+1);
            if(previous.instance_serial==item.instance_serial) duplicate=true;
        }
        if(duplicate) item.instance_serial=next;
    }

    std::vector<GroundResourcePile> piles;
    if(!read_object_vector(r,piles,[](Reader& in){
        GroundResourcePile pile{}; pile.resource=(MaterialResource)in.pod<std::uint8_t>(); pile.amount=in.pod<std::int64_t>(); pile.position=read_vec3(in); pile.radius=in.pod<float>(); pile.condition=in.pod<float>(); return pile;
    })){ error="Savegame resource-box data is damaged."; return false; }

    std::vector<CorridorConnection> corridors;
    if(!read_object_vector(r,corridors,[&](Reader& in){ return read_corridor(in,source_schema>=6,source_schema>=13); })){ error="Savegame corridor data is damaged."; return false; }
    std::vector<UtilityConnection> utilities;
    if(!read_object_vector(r,utilities,[&](Reader& in){ return read_utility(in,source_schema>=6,source_schema>=13); })){ error="Savegame supply-channel data is damaged."; return false; }
    std::vector<WalkwayConnection> walkways;
    if(!read_object_vector(r,walkways,[&](Reader& in){ return read_walkway(in,source_schema>=6,source_schema>=13); })){ error="Savegame walkway data is damaged."; return false; }

    // Legacy saves have no cross-kind placement history. Their old on-disk
    // layout grouped blueprints by kind, so migrate them deterministically in
    // that order. Schema 13+ preserves the exact player placement sequence.
    if(source_schema<13){
        std::uint64_t seq=1;
        for(auto& h:habitats) h.placement_sequence=seq++;
        for(auto& item:structures) item.placement_sequence=seq++;
        for(auto& c:corridors) c.placement_sequence=seq++;
        for(auto& c:utilities) c.placement_sequence=seq++;
        for(auto& c:walkways) c.placement_sequence=seq++;
    }

    // v0.40.10 shortens the physical construction sequence. Compact old plans
    // while preserving the exact consumed-vs-remaining material totals at the
    // saved completion boundary, so an existing colony neither loses already
    // spent resources nor has to pay for them a second time.
    for(auto& h:habitats) compact_construction(h.construction,h.resource_plan,kConstructionSteps);
    for(auto& item:structures) compact_construction(item.construction,item.resource_plan,kConstructionSteps);
    for(auto& c:corridors) compact_construction(c.construction,c.resource_plan,kCorridorConstructionSteps);
    for(auto& c:utilities) compact_construction(c.construction,c.resource_plan,kSupplyConstructionSteps);
    for(auto& c:walkways) compact_construction(c.construction,c.resource_plan,kWalkwayConstructionSteps);

    // Schema <=7 used the original raised centerline (+0.30 m); schema 8 used
    // the temporarily buried centerline (-0.24 m). Schema 9 restores channels
    // to a very low but visible ground profile. Shift the complete saved
    // connection so old colonies immediately get the same crossable geometry.
    if(source_schema<9){
        constexpr float kLegacyRaisedSupplyChannelY=0.30f;
        constexpr float kLegacyBuriedSupplyChannelY=-0.24f;
        const float old_y=source_schema<8?kLegacyRaisedSupplyChannelY:kLegacyBuriedSupplyChannelY;
        const float dy=geometry::kSupplyChannelY-old_y;
        auto shift_y=[&](eng::Vec3& v){ v.y+=dy; };
        for(auto& c:utilities){
            shift_y(c.endpoint_a.position);
            shift_y(c.endpoint_b.position);
            for(auto& v:c.path) shift_y(v);
            for(auto& v:c.junctions) shift_y(v);
            for(auto& v:c.mesh.shell) shift_y(v);
            for(auto& v:c.mesh.details) shift_y(v);
            for(auto& v:c.mesh.edges) shift_y(v);
        }
    }

    std::vector<BuilderRobotState> saved_robots;
    eng::Vec3 saved_stash{};
    std::array<double,kMaterialResourceCount> saved_production{};
    double saved_life_fraction=0.0;
    if(source_schema>=5){
        const auto robot_count=r.pod<std::uint8_t>();
        if(robot_count<1 || robot_count>12) r.valid=false;
        saved_robots.reserve(robot_count);
        for(std::uint8_t i=0;i<robot_count && r.valid;++i) saved_robots.push_back(read_builder_robot_v5(r));
        saved_stash=read_vec3(r);
        for(std::size_t i=0;i<r.material_resource_count && i<saved_production.size();++i) saved_production[i]=r.pod<double>();
        saved_life_fraction=r.pod<double>();
    }else if(source_schema>=4){
        const auto robot_count=r.pod<std::uint8_t>();
        if(robot_count<1 || robot_count>12) r.valid=false;
        saved_robots.reserve(robot_count);
        for(std::uint8_t i=0;i<robot_count && r.valid;++i) saved_robots.push_back(read_builder_robot_v4(r));
        saved_stash=read_vec3(r);
        for(std::size_t i=0;i<r.material_resource_count && i<saved_production.size();++i) saved_production[i]=r.pod<double>();
        saved_life_fraction=r.pod<double>();
    }else if(source_schema>=3){
        const auto robot_count=r.pod<std::uint8_t>();
        if(robot_count<1 || robot_count>6) r.valid=false;
        saved_robots.reserve(robot_count);
        for(std::uint8_t i=0;i<robot_count && r.valid;++i) saved_robots.push_back(read_builder_robot_legacy(r));
        saved_stash=read_vec3(r);
        for(std::size_t i=0;i<r.material_resource_count && i<saved_production.size();++i) saved_production[i]=r.pod<double>();
        saved_life_fraction=r.pod<double>();
    }else if(source_schema>=2){
        // v0.31.0 schema 2 stored exactly one BR-02 without a count field.
        saved_robots.push_back(read_builder_robot_legacy(r));
        saved_stash=read_vec3(r);
        for(std::size_t i=0;i<r.material_resource_count && i<saved_production.size();++i) saved_production[i]=r.pod<double>();
        saved_life_fraction=r.pod<double>();
    }
    if(!r.valid){ error="Savegame ended unexpectedly."; return false; }

    // v0.40.11 and older stored delivered construction material on the BR-02
    // coordinator. Move that authoritative buffer onto the blueprint itself so
    // carriers can continue supplying an unstaffed blueprint and constructors
    // can change targets without making already-delivered material disappear.
    if(source_schema<6){
        auto migrate_site_buffer=[&](const ConstructionTargetRef& target,const ResourceBundle& buffer){
            if(buffer.empty() || !target.valid()) return;
            switch(target.kind){
                case ConstructionTargetKind::Habitat:
                    if(target.index<habitats.size()) add_bundle(habitats[target.index].site_buffer,buffer);
                    break;
                case ConstructionTargetKind::Resource:
                    if(target.index<structures.size()) add_bundle(structures[target.index].site_buffer,buffer);
                    break;
                case ConstructionTargetKind::Corridor:
                    if(target.index<corridors.size()) add_bundle(corridors[target.index].site_buffer,buffer);
                    break;
                case ConstructionTargetKind::Utility:
                    if(target.index<utilities.size()) add_bundle(utilities[target.index].site_buffer,buffer);
                    break;
                case ConstructionTargetKind::Walkway:
                    if(target.index<walkways.size()) add_bundle(walkways[target.index].site_buffer,buffer);
                    break;
                case ConstructionTargetKind::None: break;
            }
        };
        for(auto& robot:saved_robots){
            if(robot.role==RobotRole::Constructor) migrate_site_buffer(robot.target,robot.site_buffer);
            robot.site_buffer={};
        }
    }else{
        for(auto& robot:saved_robots) robot.site_buffer={};
    }

    playfield_size_=saved_playfield;
    playfield_.rebuild(saved_playfield);
    camera_.restore(camera_state);
    status_.restore(colony);
    resources_.restore(resource_state);
    placed_habitats_=std::move(habitats);
    placed_resources_=std::move(structures);
    ground_resource_piles_=std::move(piles);
    corridor_network_.restore_connections(std::move(corridors));
    corridor_network_.set_playfield_half_extent((float)playfield_.size()*0.5f);
    corridor_network_.repair_invalid_topology(placed_habitats_,habitat_models_);
    utility_network_.restore_connections(std::move(utilities));
    utility_network_.set_playfield_half_extent((float)playfield_.size()*0.5f);
    walkway_network_.restore_connections(std::move(walkways));

    next_blueprint_placement_sequence_=1;
    auto advance_sequence=[&](std::uint64_t sequence){
        next_blueprint_placement_sequence_=std::max(next_blueprint_placement_sequence_,sequence+1);
    };
    for(const auto& h:placed_habitats_) advance_sequence(h.placement_sequence);
    for(const auto& item:placed_resources_) advance_sequence(item.placement_sequence);
    for(const auto& c:corridor_network_.connections()) advance_sequence(c.placement_sequence);
    for(const auto& c:utility_network_.connections()) advance_sequence(c.placement_sequence);
    for(const auto& c:walkway_network_.connections()) advance_sequence(c.placement_sequence);

    if(source_schema>=2){
        builder_robots_=std::move(saved_robots);
        if(builder_robots_.empty()) builder_robots_.push_back(BuilderRobotState{});
        starter_stash_center_=saved_stash;
        production_fraction_=saved_production;
        life_support_fraction_seconds_=saved_life_fraction;

        // Older fleets contained constructors only. Introduce one carrier when
        // loading them so the new logistics rules work without invalidating saves.
        int carriers=0; int constructors=0;
        for(const auto& robot:builder_robots_){
            if(robot.role==RobotRole::Carrier) ++carriers; else ++constructors;
        }
        starting_robot_count_=std::clamp(constructors,1,6);
        starting_carrier_count_=std::clamp(carriers,1,6);
        if(carriers==0){
            // Legacy constructor-only fleets receive one CR-01 so old saves
            // remain usable with the logistics system.
            BuilderRobotState carrier{}; carrier.role=RobotRole::Carrier;
            carrier.position={starter_stash_center_.x+6.35f,0.0f,starter_stash_center_.z};
            carrier.heading_degrees=90.0f; carrier.move_speed_mps=kCarrierRobotTransitSpeedMps; carrier.payload_capacity_kg=3600;
            builder_robots_.push_back(carrier);
            starting_carrier_count_=1;
        }
    }else{
        builder_robots_.assign(2,BuilderRobotState{});
        starting_robot_count_=1;
        starting_carrier_count_=1;
        const float half=(float)playfield_.size()*0.5f;
        starter_stash_center_={-half+5.0f,0.0f,-half+5.0f};
        builder_robots_[0].role=RobotRole::Constructor;
        builder_robots_[0].position={starter_stash_center_.x+3.15f,0.0f,starter_stash_center_.z};
        builder_robots_[0].heading_degrees=90.0f;
        builder_robots_[1].role=RobotRole::Carrier;
        builder_robots_[1].position={starter_stash_center_.x+6.35f,0.0f,starter_stash_center_.z};
        builder_robots_[1].heading_degrees=90.0f; builder_robots_[1].move_speed_mps=kCarrierRobotTransitSpeedMps; builder_robots_[1].payload_capacity_kg=3600;
        production_fraction_.fill(0.0);
        life_support_fraction_seconds_=0.0;
    }

    // Safety contracts/routes are transient. A loaded fleet is re-initialized
    // from stationary holds and must pass the shared non-overlap/static checks.
    robot_safety_time_seconds_=0.0f;
    for(auto& robot:builder_robots_) robot.safety={};

    // v0.40.30 used 2.20 m parking-bay spacing. That was physically clear
    // for the rendered rover boxes but leaves almost no clearance between the
    // conservative rotation-independent safety discs introduced by the new
    // coordinator. If the complete fleet is still sitting in that untouched
    // legacy apron, migrate the bays once on load to the wider layout. We do
    // not move a partially deployed fleet or arbitrary player-positioned rover.
    {
        const float half=(float)playfield_.size()*0.5f;
        auto legacy_spawn=[&](int ordinal){
            eng::Vec3 p{};
            if(half>=12.0f){
                const eng::Vec3 center=starter_stash_center_+eng::Vec3{4.60f,0.0f,11.60f};
                const int column=ordinal%3,row=ordinal/3;
                p={center.x+((float)column-1.0f)*2.20f,0.0f,center.z+((float)row-1.5f)*2.20f};
            }else{
                const eng::Vec3 center=starter_stash_center_+eng::Vec3{7.00f,0.0f,0.0f};
                const int column=ordinal%2,row=ordinal/2;
                p={center.x+((float)column-0.5f)*2.30f,0.0f,center.z+((float)row-2.5f)*2.05f};
            }
            p.x=std::clamp(p.x,-half+1.05f,half-1.05f);
            p.z=std::clamp(p.z,-half+1.05f,half-1.05f);
            return p;
        };
        auto safe_spawn=[&](int ordinal){
            eng::Vec3 p{};
            if(half>=12.0f){
                const eng::Vec3 center=starter_stash_center_+eng::Vec3{4.60f,0.0f,11.60f};
                const int column=ordinal%3,row=ordinal/3;
                p={center.x+((float)column-1.0f)*3.00f,0.0f,center.z+((float)row-1.5f)*3.00f};
            }else{
                const eng::Vec3 center=starter_stash_center_+eng::Vec3{7.00f,0.0f,0.0f};
                const int column=ordinal%2,row=ordinal/2;
                p={center.x+((float)column-0.5f)*2.40f,0.0f,center.z+((float)row-2.5f)*2.40f};
            }
            p.x=std::clamp(p.x,-half+1.05f,half-1.05f);
            p.z=std::clamp(p.z,-half+1.05f,half-1.05f);
            return p;
        };
        bool untouched=!builder_robots_.empty();
        for(std::size_t i=0;i<builder_robots_.size() && untouched;++i)
            { const auto legacy=legacy_spawn((int)i); const float dx=builder_robots_[i].position.x-legacy.x; const float dz=builder_robots_[i].position.z-legacy.z; untouched=std::sqrt(dx*dx+dz*dz)<0.12f; }
        if(untouched){
            for(std::size_t i=0;i<builder_robots_.size();++i) builder_robots_[i].position=safe_spawn((int)i);
        }
    }

    // Transit speed is a role characteristic rather than persistent tuning.
    // Normalize older saves to the current role travel-speed defaults.
    for(auto& robot:builder_robots_){
        robot.move_speed_mps=robot.role==RobotRole::Carrier?kCarrierRobotTransitSpeedMps:kBuilderRobotTransitSpeedMps;
        // Payload is a role characteristic just like transit speed. v0.40.40
        // doubles both rover payloads and normalizes existing savegames.
        robot.payload_capacity_kg=robot.role==RobotRole::Carrier?3600:2400;
        robot.condition=std::clamp(robot.condition,0.0f,1.0f);
        robot.visual_position=robot.position;
        robot.visual_heading_degrees=robot.heading_degrees;
        robot.visual_initialized=true;
        if(source_schema<5){
            robot.logistics_reservation={};
            // Existing in-transit cargo becomes its own exact reservation.
            // Empty FETCHING assignments are intentionally released and will
            // be recalculated by the new synchronized demand ledger.
            if(robot.cargo_committed_to_build && robot.target.valid() && !robot.cargo.empty())
                robot.logistics_reservation=robot.cargo;
            if(robot.condition<=0.0f) robot.condition=1.0f;
        }
    }
    // Schema 11 introduces Aluminum. Older colonies have no persisted Al
    // stock, so grant one recalculated starter aluminum reserve and materialize
    // it as a pallet near the starter depot. This keeps existing saves capable
    // of constructing the newly rebalanced lunar recipes without rewriting
    // their already-delivered/consumed legacy construction plans.
    if(source_schema<11){
        const std::int64_t bootstrap_aluminum=starter_resource_reserve().aluminum;
        if(bootstrap_aluminum>0){
            ResourceBundle add{};
            add.aluminum=bootstrap_aluminum;
            resources_.add(add);
            bool already_present=false;
            for(const auto& pile:ground_resource_piles_)
                if(pile.resource==MaterialResource::Aluminum && pile.amount>0){ already_present=true; break; }
            if(!already_present){
                GroundResourcePile pile{};
                pile.resource=MaterialResource::Aluminum;
                pile.amount=bootstrap_aluminum;
                pile.radius=0.36f;
                pile.condition=1.0f;
                const float half=(float)playfield_.size()*0.5f;
                const eng::Vec3 preferred{starter_stash_center_.x-2.15f,0.0f,starter_stash_center_.z+8.20f};
                pile.position={std::clamp(preferred.x,-half+0.50f,half-0.50f),0.0f,
                               std::clamp(preferred.z,-half+0.50f,half-0.50f)};
                // If the long row would hit the boundary or another pallet, use
                // a second depot column rather than overlapping the old seven.
                auto occupied=[&](eng::Vec3 p){
                    for(const auto& existing:ground_resource_piles_){
                        const float dx=existing.position.x-p.x,dz=existing.position.z-p.z;
                        if(dx*dx+dz*dz<1.10f*1.10f) return true;
                    }
                    return false;
                };
                if(occupied(pile.position)){
                    const eng::Vec3 alt{starter_stash_center_.x+0.15f,0.0f,starter_stash_center_.z};
                    pile.position={std::clamp(alt.x,-half+0.50f,half-0.50f),0.0f,
                                   std::clamp(alt.z,-half+0.50f,half-0.50f)};
                }
                ground_resource_piles_.push_back(pile);
            }
        }
    }

    // v0.40.40 expanded the starter depot from seven to eight pallets without
    // increasing the depot's south-boundary inset. On a 64x64 field the Iron
    // row therefore ended at z=-31.675, inside the rover centre clearance from
    // the map edge. CR-01 could reserve Iron but no full-hull route to its
    // pickup lane existed. Migrate only the exact schema-11 eight-pallet depot
    // signature one metre north; player-moved/produced piles remain untouched.
    if(source_schema==11){
        const float half=(float)playfield_.size()*0.5f;
        constexpr float old_inset_z=7.50f;
        constexpr float new_inset_z=8.50f;
        constexpr float pallet_spacing=2.05f;
        const float old_center_z=-half+old_inset_z;
        const float new_center_z=-half+new_inset_z;
        const float center_index=((float)kMaterialResourceCount-1.0f)*0.5f;
        std::size_t aligned=0;
        if(std::abs(starter_stash_center_.z-old_center_z)<0.35f){
            for(const auto& pile:ground_resource_piles_){
                const std::size_t mi=(std::size_t)pile.resource;
                if(mi>=kMaterialResourceCount) continue;
                const float expected_z=old_center_z+((float)mi-center_index)*pallet_spacing;
                if(std::abs(pile.position.x-(starter_stash_center_.x-2.15f))<0.35f &&
                   std::abs(pile.position.z-expected_z)<0.35f) ++aligned;
            }
        }
        if(aligned>=6){
            const float dz=new_center_z-starter_stash_center_.z;
            for(auto& pile:ground_resource_piles_){
                const std::size_t mi=(std::size_t)pile.resource;
                if(mi>=kMaterialResourceCount) continue;
                const float expected_z=old_center_z+((float)mi-center_index)*pallet_spacing;
                if(std::abs(pile.position.x-(starter_stash_center_.x-2.15f))<0.35f &&
                   std::abs(pile.position.z-expected_z)<0.35f)
                    pile.position.z+=dz;
            }
            starter_stash_center_.z=new_center_z;
            // Any persisted pickup station was computed against the old row.
            // Let active carriers choose a fresh east-side service bay/gate on
            // their next gameplay update instead of chasing the stale target.
            for(auto& robot:builder_robots_){
                if(robot.role!=RobotRole::Carrier) continue;
                robot.interaction_position_valid=false;
                robot.pickup_lane_claimed=false;
                robot.pickup_queue_age=0.0f;
                robot.safety.cached_route.clear();
                robot.safety.route_cursor=0;
                robot.safety.route_replan_required=true;
                robot.safety.route_failed=false;
                robot.safety.route_retry_elapsed=0.0f;
            }
        }
    }

    // v0.31.8/0.31.9 accidentally placed the starter pallet row at an
    // absolute world Z coordinate. Migrate only piles that still match that
    // legacy depot signature so continued games regain a stash aligned to the
    // saved starter_stash_center_ without disturbing produced/moved resources.
    constexpr float legacy_pallet_spacing=3.00f;
    constexpr float aligned_pallet_spacing=1.45f;
    const float aligned_center_index=((float)kMaterialResourceCount-1.0f)*0.5f;
    for(auto& pile:ground_resource_piles_){
        const std::size_t material_index=(std::size_t)pile.resource;
        if(material_index>=kMaterialResourceCount) continue;
        const float legacy_z=-30.0f+(float)material_index*legacy_pallet_spacing;
        if(std::abs(pile.position.x-(starter_stash_center_.x-2.15f))<0.35f &&
           std::abs(pile.position.z-legacy_z)<0.35f){
            pile.position.x=starter_stash_center_.x-2.15f;
            pile.position.z=starter_stash_center_.z+((float)material_index-aligned_center_index)*aligned_pallet_spacing;
        }
    }

    // v0.31.6 and older constructors did not use support_robot_index. Seed a
    // stable coordinator pointer for their existing target so a newly joining
    // helper cannot accidentally steal ownership of an in-progress site buffer.
    for(std::size_t i=0;i<builder_robots_.size();++i){
        auto& robot=builder_robots_[i];
        if(robot.role!=RobotRole::Constructor || !robot.target.valid()) continue;
        bool valid_owner=false;
        if(robot.support_robot_index<builder_robots_.size()){
            const auto& owner=builder_robots_[robot.support_robot_index];
            valid_owner=owner.role==RobotRole::Constructor && owner.target.valid() && owner.target.same_as(robot.target);
        }
        if(!valid_owner) robot.support_robot_index=i;
    }

    if(saved_display_width>0 && saved_display_height>0) app_.set_display_mode(saved_display_width,saved_display_height);
    app_.set_fullscreen(saved_fullscreen);
    display_modes_.clear();
    display_mode_index_=-1;
    ui_scale_index_=std::clamp(saved_ui_scale,0,layout::UI_SCALE_COUNT-1);
    ui_scale_=layout::UI_SCALES[ui_scale_index_];
    debug_frame_enabled_=saved_debug;
    habitat_view_mode_=saved_view;
    game_speed_=saved_game_speed;
    build_catalog_.set_category(saved_category);
    build_catalog_.clear_selection();
    placement_rotation_degrees_=saved_rotation;
    placement_habitat_size_=saved_habitat_size;
    placement_storage_size_=saved_storage_size;
    placement_resource_size_=saved_resource_size;

    first_person_mode_=FirstPersonMode::Off;
    app_.set_relative_mouse_mode(false);
    first_person_carried_resource_.reset();
    first_person_hover_ground_resource_=-1;
    first_person_hover_storage_resource_=-1;
    first_person_hover_storage_box_=-1;
    clear_world_selection();
    selection_marquee_active_=false;
    selection_marquee_dragged_=false;
    corridor_link_source_=-1;
    corridor_build_start_.reset(); utility_build_start_.reset(); walkway_build_start_.reset(); walkway_build_target_.reset();
    game_session_active_=true;
    save_game_name_=save_name;
    error.clear();
    return true;
}


} // namespace game
