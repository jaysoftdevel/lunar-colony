#include "game/game_app.hpp"
#include "game/world/habitats/screen_mounts.hpp"
#include "game/world/gas_flow.hpp"
#include "game/world/world_labels.hpp"

namespace game::structure_display {
Content environment(const char* title,const SupplyState& supply){
    const auto partial_kpa=.30*gas::pressure(supply.oxygen_kg,supply.volume_m3)/1000.;
    const double net=supply.oxygen_net_kg_per_day;
    Content content{title,{"TEMP SET "+value(gas::temperature-273.15)+" C",
        "O2 PRESS "+value(partial_kpa,1)+" KPA",
        "NET O2 "+std::string(net>.049?"+":"")+value(std::abs(net)<.05?0.:net,1)+" KG/D"}};
    if(supply.warnings)content.accent={1,.68f,.23f,1};
    return content;
}
void draw_door_pair(runtime::SceneRenderer& renderer,Vec3 base,Vec3 normal,float half_width,float depth,
                    const Content& content,bool powered,Vec3 eye){
    using namespace runtime;
    std::vector<Vec3> metal,glass;
    for(float face:{-1.f,1.f}){
        const auto mount=jamb_mount(base,normal,half_width,depth,face);
        casing(metal,glass,mount);
        draw(renderer,mount,{},0,content,powered,eye);
    }
    renderer.draw_lunar_surface(metal,LunarMaterial::Alloy,{.26f,.31f,.34f,1});
    renderer.draw_lunar_surface(glass,powered?LunarMaterial::Display:LunarMaterial::Rubber,{.025f,.055f,.062f,1});
}
}
namespace game {
structure_display::Content GameApp::habitat_screen_content(std::size_t index) const {
    const auto& room=placed_habitats_[index];
    auto content=structure_display::environment(world_labels::habitat_name(room.kind),room.supply);
    if(room.kind==HabitatKind::AirLock&&index<airlock_cycles_.size())
        content.title="AIRLOCK / "+std::string(airlock::phase(airlock_cycles_[index]));
    return content;
}
}
