#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <system_error>

namespace game {
namespace {
namespace fs=std::filesystem;
constexpr std::array<char,8> kSaveMagic{{'G','A','M','E','S','A','V','1'}};
constexpr std::uint32_t kSaveSchema=1;
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
    for(std::size_t i=0;i<kMaterialResourceCount;++i) b.amount((MaterialResource)i)=r.pod<std::int64_t>();
    return b;
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

void write_construction(Writer& w,const ConstructionState& c){ w.pod(c.step); w.pod(c.step_elapsed); }
ConstructionState read_construction(Reader& r){
    ConstructionState c{}; c.step=r.pod<std::uint8_t>(); c.step_elapsed=r.pod<float>();
    if(c.step>kConstructionSteps) r.valid=false;
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
    write_corridor_mesh(w,c.mesh); write_staged(w,c.construction); write_plan(w,c.resource_plan);
}
CorridorConnection read_corridor(Reader& r){
    CorridorConnection c{}; c.kind=(CorridorConnectionKind)r.pod<std::uint8_t>(); c.endpoint_a=read_corridor_endpoint(r); c.endpoint_b=read_corridor_endpoint(r);
    c.path=read_vec3s(r); c.mesh=read_corridor_mesh(r); c.construction=read_staged(r); c.resource_plan=read_plan(r); return c;
}

void write_utility_endpoint(Writer& w,const UtilityEndpoint& e){ w.pod((std::uint8_t)e.kind); w.pod((std::uint64_t)e.index); write_vec3(w,e.position); write_vec3(w,e.normal); }
UtilityEndpoint read_utility_endpoint(Reader& r){ UtilityEndpoint e{}; e.kind=(UtilityEndpointKind)r.pod<std::uint8_t>(); e.index=(std::size_t)r.pod<std::uint64_t>(); e.position=read_vec3(r); e.normal=read_vec3(r); return e; }
void write_utility_mesh(Writer& w,const UtilityMesh& m){ write_vec3s(w,m.shell); write_vec3s(w,m.details); write_vec3s(w,m.edges); }
UtilityMesh read_utility_mesh(Reader& r){ UtilityMesh m{}; m.shell=read_vec3s(r); m.details=read_vec3s(r); m.edges=read_vec3s(r); return m; }
void write_utility(Writer& w,const UtilityConnection& c){
    w.pod((std::uint8_t)c.kind); write_utility_endpoint(w,c.endpoint_a); write_utility_endpoint(w,c.endpoint_b); write_vec3s(w,c.path); write_vec3s(w,c.junctions);
    write_utility_mesh(w,c.mesh); write_staged(w,c.construction); write_plan(w,c.resource_plan);
}
UtilityConnection read_utility(Reader& r){
    UtilityConnection c{}; c.kind=(UtilityKind)r.pod<std::uint8_t>(); c.endpoint_a=read_utility_endpoint(r); c.endpoint_b=read_utility_endpoint(r);
    c.path=read_vec3s(r); c.junctions=read_vec3s(r); c.mesh=read_utility_mesh(r); c.construction=read_staged(r); c.resource_plan=read_plan(r); return c;
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
    write_walkway_mesh(w,c.mesh); write_staged(w,c.construction); write_plan(w,c.resource_plan);
    w.boolean(c.carries_power); w.boolean(c.carries_water); w.boolean(c.pressurized);
}
WalkwayConnection read_walkway(Reader& r){
    WalkwayConnection c{}; c.endpoint_a=read_walkway_endpoint(r); c.endpoint_b=read_walkway_endpoint(r); c.path=read_vec3s(r);
    auto n=r.pod<std::uint64_t>(); if(n>kMaxObjects) r.valid=false; c.junctions.reserve((std::size_t)std::min(n,kMaxObjects)); for(std::uint64_t i=0;i<n&&r.valid;++i)c.junctions.push_back(read_walkway_junction(r));
    n=r.pod<std::uint64_t>(); if(n>kMaxObjects) r.valid=false; c.corridor_crossings.reserve((std::size_t)std::min(n,kMaxObjects)); for(std::uint64_t i=0;i<n&&r.valid;++i)c.corridor_crossings.push_back(read_walkway_crossing(r));
    n=r.pod<std::uint64_t>(); if(n>kMaxObjects) r.valid=false; c.supply_bridges.reserve((std::size_t)std::min(n,kMaxObjects)); for(std::uint64_t i=0;i<n&&r.valid;++i)c.supply_bridges.push_back(read_walkway_bridge(r));
    c.mesh=read_walkway_mesh(r); c.construction=read_staged(r); c.resource_plan=read_plan(r); c.carries_power=r.boolean(); c.carries_water=r.boolean(); c.pressurized=r.boolean(); return c;
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
    if(r.pod<std::uint32_t>()!=kSaveSchema) return false;
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

    const CameraState camera_state=camera_.state();
    w.pod(camera_state.distance); w.pod(camera_state.elevation_degrees); w.pod(camera_state.azimuth_degrees); write_vec3(w,camera_state.target);
    const ColonyStatusState colony=status_.state();
    w.pod((std::int32_t)colony.day); w.pod(colony.hour); w.pod((std::int32_t)colony.power_current); w.pod((std::int32_t)colony.power_capacity);
    w.pod((std::int32_t)colony.water_current); w.pod((std::int32_t)colony.water_capacity); w.pod((std::int32_t)colony.crew_current); w.pod((std::int32_t)colony.crew_capacity);
    write_bundle(w,resources_.amounts());

    write_object_vector(w,placed_habitats_,[](Writer& out,const PlacedHabitat& h){
        out.pod((std::uint8_t)h.kind); write_vec3(out,h.position); out.pod(h.rotation_degrees); out.pod((std::uint8_t)h.size);
        write_construction(out,h.construction); write_plan(out,h.resource_plan);
    });
    write_object_vector(w,placed_resources_,[](Writer& out,const PlacedResource& item){
        out.pod((std::uint8_t)item.kind); write_vec3(out,item.position); out.pod(item.rotation_degrees); write_construction(out,item.construction); write_plan(out,item.resource_plan);
        out.pod((std::uint8_t)item.storage_size);
        out.pod((std::uint64_t)item.stored_boxes.size());
        for(const auto& box:item.stored_boxes){ out.pod((std::uint8_t)box.resource); out.pod(box.amount); out.pod(box.condition); }
    });
    write_object_vector(w,ground_resource_piles_,[](Writer& out,const GroundResourcePile& pile){
        out.pod((std::uint8_t)pile.resource); out.pod(pile.amount); write_vec3(out,pile.position); out.pod(pile.radius); out.pod(pile.condition);
    });

    write_object_vector(w,corridor_network_.connections(),write_corridor);
    write_object_vector(w,utility_network_.connections(),write_utility);
    write_object_vector(w,walkway_network_.connections(),write_walkway);

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

bool GameApp::load_game_from_file(const std::string& path,std::string& error){
    Reader r{fs::path(path)};
    std::array<char,8> magic{};
    if(!r.in.read(magic.data(),magic.size()) || magic!=kSaveMagic){ error="Invalid savegame file."; return false; }
    if(r.pod<std::uint32_t>()!=kSaveSchema){ error="Unsupported savegame format."; return false; }
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
    if(saved_playfield<16 || saved_playfield>512){ error="Savegame playfield size is invalid."; return false; }

    CameraState camera_state{};
    camera_state.distance=r.pod<float>(); camera_state.elevation_degrees=r.pod<float>(); camera_state.azimuth_degrees=r.pod<float>(); camera_state.target=read_vec3(r);
    ColonyStatusState colony{};
    colony.day=(int)r.pod<std::int32_t>(); colony.hour=r.pod<float>(); colony.power_current=(int)r.pod<std::int32_t>(); colony.power_capacity=(int)r.pod<std::int32_t>();
    colony.water_current=(int)r.pod<std::int32_t>(); colony.water_capacity=(int)r.pod<std::int32_t>(); colony.crew_current=(int)r.pod<std::int32_t>(); colony.crew_capacity=(int)r.pod<std::int32_t>();
    const ResourceBundle resource_state=read_bundle(r);

    std::vector<PlacedHabitat> habitats;
    if(!read_object_vector(r,habitats,[](Reader& in){
        PlacedHabitat h{}; h.kind=(HabitatKind)in.pod<std::uint8_t>(); h.position=read_vec3(in); h.rotation_degrees=in.pod<float>(); h.size=(HabitatSizeTier)in.pod<std::uint8_t>();
        h.construction=read_construction(in); h.resource_plan=read_plan(in); return h;
    })){ error="Savegame habitat data is damaged."; return false; }

    std::vector<PlacedResource> structures;
    if(!read_object_vector(r,structures,[](Reader& in){
        PlacedResource item{}; item.kind=(ResourceKind)in.pod<std::uint8_t>(); item.position=read_vec3(in); item.rotation_degrees=in.pod<float>(); item.construction=read_construction(in); item.resource_plan=read_plan(in);
        item.storage_size=(StorageDomeSize)in.pod<std::uint8_t>();
        const auto n=in.pod<std::uint64_t>(); if(n>kMaxObjects) in.valid=false;
        item.stored_boxes.reserve((std::size_t)std::min(n,kMaxObjects));
        for(std::uint64_t i=0;i<n&&in.valid;++i){ StoredResourceBox box{}; box.resource=(MaterialResource)in.pod<std::uint8_t>(); box.amount=in.pod<std::int64_t>(); box.condition=in.pod<float>(); item.stored_boxes.push_back(box); }
        sync_stored_resource_totals(item); return item;
    })){ error="Savegame structure data is damaged."; return false; }

    std::vector<GroundResourcePile> piles;
    if(!read_object_vector(r,piles,[](Reader& in){
        GroundResourcePile pile{}; pile.resource=(MaterialResource)in.pod<std::uint8_t>(); pile.amount=in.pod<std::int64_t>(); pile.position=read_vec3(in); pile.radius=in.pod<float>(); pile.condition=in.pod<float>(); return pile;
    })){ error="Savegame resource-box data is damaged."; return false; }

    std::vector<CorridorConnection> corridors;
    if(!read_object_vector(r,corridors,read_corridor)){ error="Savegame corridor data is damaged."; return false; }
    std::vector<UtilityConnection> utilities;
    if(!read_object_vector(r,utilities,read_utility)){ error="Savegame supply-channel data is damaged."; return false; }
    std::vector<WalkwayConnection> walkways;
    if(!read_object_vector(r,walkways,read_walkway)){ error="Savegame walkway data is damaged."; return false; }
    if(!r.valid){ error="Savegame ended unexpectedly."; return false; }

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
    utility_network_.restore_connections(std::move(utilities));
    utility_network_.set_playfield_half_extent((float)playfield_.size()*0.5f);
    walkway_network_.restore_connections(std::move(walkways));

    if(saved_display_width>0 && saved_display_height>0) app_.set_display_mode(saved_display_width,saved_display_height);
    app_.set_fullscreen(saved_fullscreen);
    display_modes_.clear();
    display_mode_index_=-1;
    ui_scale_index_=std::clamp(saved_ui_scale,0,layout::UI_SCALE_COUNT-1);
    ui_scale_=layout::UI_SCALES[ui_scale_index_];
    debug_frame_enabled_=saved_debug;
    habitat_view_mode_=saved_view;
    build_catalog_.set_category(saved_category);
    build_catalog_.clear_selection();
    placement_rotation_degrees_=saved_rotation;
    placement_habitat_size_=saved_habitat_size;
    placement_storage_size_=saved_storage_size;

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
