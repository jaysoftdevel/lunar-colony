#include "game/game_app.hpp"
#include "game/world/interior_unit_mesh.hpp"
#include "game/ui/layout.hpp"
#include "game/util/mesh_wireframe.hpp"
#include <sstream>
namespace game {
bool GameApp::interior_unit_pointer(InteriorUnit& unit) const {
    if(!interior_unit_tool_||interior_unit_host_>=placed_habitats_.size())return false;
    const auto& host=placed_habitats_[interior_unit_host_];const auto& model=habitat_models_.mesh(host);
    runtime::Vec3 ground;if(!pick_build_ground(ground))return false;
    const auto eye=first_person_mode_==FirstPersonMode::Active?first_person_.eye_position():camera_.eye_position();
    const auto ray=ground-eye;const float plane=host.position.y+model.walkable_floor_y;
    if(std::abs(ray.y)<1e-5f)return false;
    const float t=(plane-eye.y)/ray.y;
    if(t<=0)return false;
    auto local=world_math::rotate_y(eye+ray*t-host.position,-host.rotation_degrees);
    local.x=std::round(local.x*4.f)*.25f;local.z=std::round(local.z*4.f)*.25f;local.y=model.walkable_floor_y;
    unit={};unit.kind=*interior_unit_tool_;unit.position=local;unit.rotation_degrees=interior_unit_rotation_;return true;
}
bool GameApp::update_interior_unit_placement(const runtime::InputState& in){
    if(!interior_unit_tool_)return false;
    if(build_catalog_.selected()||storage_gate_placement_active_||interior_unit_host_>=placed_habitats_.size()||selection_kind_!=SelectionKind::Habitat||selected_habitat_!=int(interior_unit_host_)){
        interior_unit_tool_.reset();return true;
    }
    if(in.key_n_pressed)interior_unit_rotation_=std::fmod(interior_unit_rotation_+345.f,360.f);
    if(in.key_m_pressed)interior_unit_rotation_=std::fmod(interior_unit_rotation_+15.f,360.f);
    if(in.left_pressed&&!mouse_over_playfield_overlay(in)&&!mouse_over_selection_panel(in)){
        InteriorUnit candidate;
        if(interior_unit_pointer(candidate)&&place_interior_unit(interior_unit_host_,candidate,interior_unit_message_)){
            interior_unit_message_="UNIT INSTALLED";if(!in.key_shift_down)interior_unit_tool_.reset();
        }
    }
    return true;
}
void GameApp::render_interior_unit_preview(runtime::SceneRenderer& renderer){
    if(!interior_unit_tool_||mouse_over_selection_panel(input_)||mouse_over_playfield_overlay(input_))return;
    InteriorUnit candidate;if(!interior_unit_pointer(candidate))return;
    std::string reason;const bool valid=interior_unit_position_clear(interior_unit_host_,candidate,reason)&&resources_.can_consume(interior_unit_profile(candidate.kind).cost);
    const auto& host=placed_habitats_[interior_unit_host_];const auto& model=interior_unit_mesh(candidate.kind);
    std::vector<runtime::Vec3> triangles;
    for(const auto p:model.interior)triangles.push_back(host.position+world_math::rotate_y(candidate.position+world_math::rotate_y(p,candidate.rotation_degrees),host.rotation_degrees));
    mesh_wireframe::draw_layer(renderer,triangles,valid?runtime::Color{.20f,.95f,.65f,.85f}:runtime::Color{1,.20f,.15f,.85f},2.f);
    placement_tip_title_=interior_unit_profile(candidate.kind).name;
    placement_tip_detail_=valid?"LMB INSTALL / N-M ROTATE / SHIFT REPEAT / RMB CANCEL":reason.empty()?"NOT ENOUGH CONSTRUCTION MATERIAL":reason;
    placement_tip_cost_=interior_unit_profile(candidate.kind).cost;
}
float GameApp::interior_unit_panel_height(std::size_t index) const {
    if(index>=placed_habitats_.size()||!placed_habitats_[index].construction.complete())return 0;
    const auto& h=placed_habitats_[index];
    int count=0;for(int i=0;i<int(InteriorUnitKind::Count);++i)count+=interior_unit_allowed(h.kind,static_cast<InteriorUnitKind>(i));
    return 106.f+64.f*count+58.f*h.units.size();
}
void GameApp::render_interior_unit_panel(runtime::SceneRenderer& r,std::size_t index,float x,float y,float width){
    using namespace runtime;using namespace layout;auto& host=placed_habitats_[index];
    if(interior_unit_panel_height(index)==0)return;
    const auto panel=selection_panel_rect();
    const auto visible=[&](float row,float height){return row>=panel.y&&row+height<=panel.y+panel.h;};
    const auto label=[&](float row,const std::string& text,float scale=.40f){if(visible(row,15))game_label_fitted(r,{x,row,width,15},text,scale,theme_,true);};
    const auto button=[&](Rect box,const std::string& text,bool enabled=true){return visible(box.y,box.h)&&ui::button(r,ui_input_,box,text,theme_,enabled);};
    label(y,"INTERNAL UNITS / "+std::to_string(host.units.size())+" INSTALLED",.50f);y+=21;
    label(y,"SELECT UNIT, THEN CLICK ITS POSITION ON THE DECK");y+=20;
    for(int n=0;n<int(InteriorUnitKind::Count);++n){const auto kind=static_cast<InteriorUnitKind>(n);if(!interior_unit_allowed(host.kind,kind))continue;
        const auto& profile=interior_unit_profile(kind);
        if(button({x,y,width,26},std::string(interior_unit_tool_==kind?"PLACING / ":"PLACE / ")+profile.name)){
            build_catalog_.clear_selection();interior_unit_tool_=kind;interior_unit_host_=index;interior_unit_rotation_=0;interior_unit_message_.clear();
        }
        std::ostringstream cost;cost<<"FE "<<profile.cost.iron<<" / CU "<<profile.cost.copper<<" / SI "<<profile.cost.silicon<<" / P "<<profile.cost.plastic<<" / ";
        if(kind==InteriorUnitKind::HydroponicRack)cost<<"CROP LOAD";else cost<<profile.power_kw<<" KW";
        label(y+29,cost.str(),.34f);label(y+45,interior_unit_recipe(kind),.32f);y+=64;
    }
    std::ostringstream ore;ore<<"ORE INPUT FE/CU/SI/AL  ";for(double kg:host.unit_ore_kg)ore<<int(kg)<<" ";label(y,ore.str(),.34f);y+=20;
    label(y,interior_unit_message_.empty()?"CLEAR CENTER AISLE AND DOOR APPROACHES ARE RESERVED":interior_unit_message_,.34f);y+=21;
    for(std::size_t n=0;n<host.units.size();++n){auto& unit=host.units[n];const auto& profile=interior_unit_profile(unit.kind);
        label(y,std::to_string(n+1)+" / "+profile.name,.42f);y+=16;
        label(y,interior_unit_status(index,unit),.34f);y+=15;
        const bool passive=unit.kind==InteriorUnitKind::DiningTable||unit.kind==InteriorUnitKind::SeatingBench;
        if(!passive&&button({x,y,width*.48f,22},unit.enabled?"SWITCH OFF":"SWITCH ON")){unit.enabled=!unit.enabled;update_supply_simulation(0);}
        const std::string removal=unit.batch_active?"SCRAP / LOSE BATCH":unit.water_litres>0||unit.food_kg>0?"REMOVE / LOSE STOCK":"REMOVE UNIT";
        if(button({x+(passive?0:width*.52f),y,width*(passive?1.f:.48f),22},removal)){
            host.units.erase(host.units.begin()+static_cast<std::ptrdiff_t>(n));interior_unit_message_="UNIT REMOVED / MATERIALS SCRAPPED";return;
        }
        y+=27;
    }
}
}
