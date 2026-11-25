#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include <queue>
#include <unordered_set>
#include <limits>
namespace game {
using namespace runtime;
void GameApp::reset_first_person_crew(){
    astronaut_views_={};controlled_astronaut_=last_fpv_astronaut_=-1;
    test_astronauts_={};astronaut_cache_={};selected_astronaut_=-1;
}
Vec3 GameApp::crew_cache_position() const {
    return lander_.has_depot()?lunar_lander_salvage_station(lander_,0):Vec3{};
}
std::optional<Vec3> GameApp::nearest_first_person_spawn(Vec3 desired,bool exterior) const {
    if(!std::isfinite(desired.x)||!std::isfinite(desired.y)||!std::isfinite(desired.z))return {};
    const float half=playfield_.size()*.5f-first_person_.body_radius()-.02f;
    Vec3 origin{std::clamp(desired.x,-half,half),desired.y,std::clamp(desired.z,-half,half)};
    const auto clear=[&](Vec3& p){
        p.y=first_person_floor_height(p);
        if(exterior&&!first_person_spawn_valid(p))return false;
        if(!first_person_move_allowed(p,p)||first_person_ceiling_height(p)<p.y+first_person_.body_height()+.02f)return false;
        for(const auto& rover:builder_robots_)if(p.y<rover.position.y+1.4f&&p.y+first_person_.body_height()>rover.position.y&&world_math::distance_xz(p,rover.position)<1.65f)return false;
        for(std::size_t i=0;i<test_astronauts_.size();++i)if(int(i)!=controlled_astronaut_&&astronaut_present(i)){
            const auto q=test_astronauts_[i].position;
            if(std::abs(p.y-q.y)<first_person_.body_height()&&world_math::distance_xz(p,q)<kIndoorAstronautSeparation)return false;
        }
        return true;
    };
    Vec3 point=origin;if(clear(point))return point;
    struct Node {float distance;int x,z;};
    const auto farther=[](const Node& a,const Node& b){return a.distance>b.distance;};
    std::priority_queue<Node,std::vector<Node>,decltype(farther)> pending(farther);
    std::unordered_set<std::uint64_t> seen;
    constexpr float spacing=.25f;
    const auto add=[&](int x,int z){
        const Vec3 p{origin.x+x*spacing,origin.y,origin.z+z*spacing};
        if(p.x<-half||p.x>half||p.z<-half||p.z>half)return;
        const auto key=(std::uint64_t(std::uint32_t(x))<<32)|std::uint32_t(z);
        if(seen.insert(key).second)pending.push({(p.x-desired.x)*(p.x-desired.x)+(p.z-desired.z)*(p.z-desired.z),x,z});
    };
    add(0,0);
    while(!pending.empty()){
        const auto n=pending.top();pending.pop();
        point={origin.x+n.x*spacing,origin.y,origin.z+n.z*spacing};
        if(clear(point))return point;
        for(int x=-1;x<=1;++x)for(int z=-1;z<=1;++z)if(x||z)add(n.x+x,n.z+z);
    }
    return {};
}
void GameApp::enter_first_person_mode(){
    door_hold_={};
    const bool explicit_selection=selected_astronaut_>=0&&astronaut_present(std::size_t(selected_astronaut_));
    int index=explicit_selection?selected_astronaut_:last_fpv_astronaut_;
    if(index<0||!astronaut_present(std::size_t(index))||(!explicit_selection&&test_astronauts_[index].health<=0)){
        index=0;for(std::size_t i=0;i<test_astronauts_.size();++i)if(astronaut_present(i)&&test_astronauts_[i].health>0){index=int(i);break;}
    }
    auto& actor=test_astronauts_[index];auto& view=astronaut_views_[index];
    if(astronaut_present(index)&&actor.health<=0){
        actor.activity=AstronautActivity::Knockout;
        cancel_placement_tool();clear_world_selection();
        selected_astronaut_=index;
        activity_log_.record(ui_time_seconds_,LogEvent::Incident,"FPV / This astronaut is deceased. Select living crew or dispatch a new explorer at the lander.");
        return;
    }
    controlled_astronaut_=index;
    const bool first=!view.visited&&!explicit_selection;
    const auto desired=first?crew_cache_position():actor.position;
    const auto spawn=nearest_first_person_spawn(desired,first);
    if(!spawn){controlled_astronaut_=-1;activity_log_.record(ui_time_seconds_,LogEvent::Incident,"FPV / No safe spawn position is available");return;}
    if(!astronaut_present(index))actor={};
    actor.position=*spawn;actor.destination.reset();actor.route.clear();actor.route_step=0;actor.entry_step=2;actor.construction_job=actor.bed_owner=0;
    if(actor.activity!=AstronautActivity::Sleeping)actor.activity=AstronautActivity::Manual;
    float yaw=view.visited?view.yaw:std::fmod(540.f-actor.heading_degrees,360.f);
    if(first&&lander_.has_depot()){const auto d=lander_.site-*spawn;yaw=std::atan2(d.x,-d.z)*57.2957795f;}
    first_person_.spawn(*spawn,yaw);first_person_.restore_view(yaw,view.visited?view.pitch:0);
    view.present=view.visited=true;last_fpv_astronaut_=index;
    cancel_placement_tool();clear_world_selection();selected_astronaut_=index;
    first_person_hover_ground_resource_=first_person_hover_storage_resource_=first_person_hover_storage_box_=-1;
    first_person_cursor_=first_person_alt_cursor_=false;first_person_mode_=FirstPersonMode::Active;
    sync_controlled_astronaut(0);app_.set_relative_mouse_mode(true);app_.set_cursor_visible(false);
}
bool GameApp::explorer_slot_available() const {
    if(!lander_.has_depot()||astronaut_test_active())return false;
    for(std::size_t i=0;i<astronaut_views_.size();++i)if(!astronaut_present(i))return true;
    return false;
}
bool GameApp::dispatch_explorer(){
    if(!explorer_slot_available())return false;
    if(first_person_mode_==FirstPersonMode::Active)exit_first_person_mode();
    const auto old_last=last_fpv_astronaut_;const auto old_selected=selected_astronaut_;
    for(std::size_t i=0;i<astronaut_views_.size();++i)if(!astronaut_present(i)){
        const auto old_actor=test_astronauts_[i];const auto old_view=astronaut_views_[i];
        test_astronauts_[i]={};astronaut_views_[i]={};astronaut_views_[i].present=true;
        last_fpv_astronaut_=int(i);selected_astronaut_=-1;enter_first_person_mode();
        if(first_person_mode_==FirstPersonMode::Active){activity_log_.record(ui_time_seconds_,LogEvent::Note,"New explorer dispatched at the lander");return true;}
        test_astronauts_[i]=old_actor;astronaut_views_[i]=old_view;last_fpv_astronaut_=old_last;selected_astronaut_=old_selected;return false;
    }
    return false;
}
void GameApp::sync_controlled_astronaut(float dt){
    if(first_person_mode_!=FirstPersonMode::Active||controlled_astronaut_<0)return;
    auto& actor=test_astronauts_[controlled_astronaut_];auto& view=astronaut_views_[controlled_astronaut_];
    const auto p=first_person_.feet_position();const float distance=world_math::distance_xz(p,actor.position);
    actor.position=p;actor.heading_degrees=std::fmod(540.f-first_person_.yaw_degrees(),360.f);
    actor.speed=dt>0?distance/dt:0;actor.stride_phase=std::fmod(actor.stride_phase+distance/astronaut_stride(actor.suited),1.f);
    actor.destination.reset();actor.route.clear();actor.entry_step=2;actor.construction_job=actor.bed_owner=0;
    if(actor.activity!=AstronautActivity::Sleeping&&actor.health>0)actor.activity=AstronautActivity::Manual;
    view.present=view.visited=true;view.yaw=first_person_.yaw_degrees();view.pitch=first_person_.pitch_degrees();
}
void GameApp::exit_first_person_mode(){
    door_hold_={};
    sync_controlled_astronaut(0);
    if(first_person_carried_resource_)drop_first_person_carried_resource();
    if(controlled_astronaut_>=0)selected_astronaut_=last_fpv_astronaut_=controlled_astronaut_;
    controlled_astronaut_=-1;first_person_mode_=FirstPersonMode::Off;
    first_person_hover_ground_resource_=first_person_hover_storage_resource_=first_person_hover_storage_box_=-1;
    first_person_cursor_=first_person_alt_cursor_=false;cancel_placement_tool();
    app_.set_relative_mouse_mode(false);app_.set_cursor_visible(true);
}
Rect GameApp::astronaut_status_rect() const {
    const float w=viewport_width_/ui_scale_,h=viewport_height_/ui_scale_;
    const float width=std::min(390.f,w-24.f),height=std::min(252.f,h-layout::STATUS_BAR_H-layout::BUILD_BAR_H-24.f);
    return {w-width-12.f,h-layout::BUILD_BAR_H-height-12.f,width,height};
}
bool GameApp::astronaut_status_hover(const InputState& in) const {
    if(!astronauts_active()||(first_person_mode_!=FirstPersonMode::Active&&selected_astronaut_<0))return false;
    return astronaut_status_rect().contains(in.mouse_x/ui_scale_,in.mouse_y/ui_scale_);
}
void GameApp::cycle_astronaut_control(){
    const bool active=first_person_mode_==FirstPersonMode::Active;
    const int current=active?controlled_astronaut_:selected_astronaut_;
    for(std::size_t offset=1;offset<=test_astronauts_.size();++offset){
        const auto index=(std::size_t(std::max(current,0))+offset)%test_astronauts_.size();
        if(!astronaut_present(index)||(active&&test_astronauts_[index].health<=0))continue;
        if(active){exit_first_person_mode();selected_astronaut_=int(index);enter_first_person_mode();}
        else selected_astronaut_=int(index);
        return;
    }
}
}

