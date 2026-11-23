#include "game/game_app.hpp"
#include <bit>
#include "game/world/structure_display.hpp"
#include "game/util/world_math.hpp"
#include "game/world/world_labels.hpp"

namespace game {
structure_display::Content GameApp::habitat_screen_content(std::size_t index) const {
    using namespace game::runtime;using namespace structure_display;
    const auto& room=placed_habitats_[index];
    const auto& mesh=habitat_runtime_mesh(index);const auto& supply=room.supply;
    Content content;
    switch(room.kind){
        case HabitatKind::AirLock:
            content.title="AIRLOCK CONTROL";
            content.rows={"CABIN AIR "+value(supply.breathability())+"%","POWER "+value(supply.power_fraction*100)+"%","CYCLE IDLE"};
            if(index<airlock_cycles_.size()){
                content.rows[0]="CHAMBER "+value(airlock_cycles_[index].pressure*100)+"%";
                content.rows[2]=airlock::phase(airlock_cycles_[index]);
            }break;
        case HabitatKind::OxygenGeneratorDome:
            content.title="OXYGEN SYSTEM";content.rows={"O2 "+value(supply.oxygen_produced_kg_per_day,1)+" KG/D","WATER "+value(supply.water_processed_litres_per_day,1)+" L/D","CABIN AIR "+value(supply.breathability())+"%"};break;
        case HabitatKind::BioDome:{
            int planted=0;double growth=0;
            for(std::size_t i=0;i<hydroponics_bay_count(room.size);++i)if(!(mesh.blocked_grow_bays&(1u<<i))&&room.hydroponics[i].plant!=HydroponicPlant::Empty){++planted;growth+=room.hydroponics[i].growth_fraction;}
            content.title="HYDROPONICS";content.rows={"CROPS "+std::to_string(planted)+"/"+std::to_string(hydroponics_bay_count(room.size)-std::popcount(unsigned(mesh.blocked_grow_bays))),"GROWTH "+value(planted?growth*100/planted:0)+"%","CABIN AIR "+value(supply.breathability())+"%"};break;}
        case HabitatKind::CrewQuarters:
            content.title="CREW QUARTERS";content.rows={"BEDS "+std::to_string(std::count_if(mesh.crew_bunks.begin(),mesh.crew_bunks.end(),[](const auto& bed){return bed.available;})),"CABIN AIR "+value(supply.breathability())+"%","POWER "+value(supply.power_fraction*100)+"%"};break;
        case HabitatKind::CantineDome:
            content.title="CANTEEN STORES";content.rows={"COLONY FOOD "+value(resources_.amounts().food)+" KG","WATER "+value(supply.water_litres)+" L","CABIN AIR "+value(supply.breathability())+"%"};break;
        case HabitatKind::SickBay:
            content.title="MEDICAL STORES";content.rows={"MEDICAL "+value(resources_.amounts().medical)+" KG","CABIN AIR "+value(supply.breathability())+"%","POWER "+value(supply.power_fraction*100)+"%"};break;
        default:
            content.title=room.kind==HabitatKind::Laboratory?"LAB ENVIRONMENT":room.kind==HabitatKind::Factory?"FACTORY SERVICES":room.kind==HabitatKind::Manufactory?"MANUFACTORY":"PROCESSING SERVICES";
            content.rows={"POWER "+value(supply.power_fraction*100)+"%","CABIN AIR "+value(supply.breathability())+"%","WATER "+value(supply.water_litres)+" L"};break;
    }
    if(supply.warnings)content.accent={1,.68f,.23f,1};
    return content;
}
void GameApp::render_habitat_screens(game::runtime::SceneRenderer& renderer,std::size_t index) const {
    using namespace game::runtime;using namespace structure_display;
    const auto& room=placed_habitats_[index];if(!room.construction.complete()||mesh_layout_active())return;
    const auto& mesh=habitat_runtime_mesh(index);
    const bool powered=room.power_enabled&&room.supply.power_fraction>.01;
    const Vec3 eye=first_person_mode_==FirstPersonMode::Active?first_person_.eye_position():camera_.eye_position();
    const auto content=habitat_screen_content(index);
    const auto doors=habitat_doors(index);
    for(const auto& screen:mesh.screens){
        const auto center=world_math::rotate_y(screen.center,room.rotation_degrees)+room.position;
        bool door_overlap=false;
        for(const auto& door:doors){const auto offset=center-door.wall_position;const Vec3 side{-door.normal.z,0,door.normal.x};
            if(std::abs(dot(offset,door.normal))<1.f&&std::abs(dot(offset,side))<geometry::kDoorClearWidth*.5f+screen.width*.5f&&center.y>door.position.y&&center.y<door.position.y+geometry::kDoorClearHeight+.2f)door_overlap=true;
        }
        if(!door_overlap)draw(renderer,screen,room.position,room.rotation_degrees,content,powered,eye);
    }
}
}
