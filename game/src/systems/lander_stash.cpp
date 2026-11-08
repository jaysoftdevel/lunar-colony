#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace game {
namespace {
using eng::Vec3;
using world_math::distance_xz;
using world_math::footprint;
using world_math::overlaps;
constexpr float kPairPitch=3.70f;
constexpr float kHalfPair=0.30f;
constexpr float kStackPitch=0.64f;
constexpr float kStackSeparation=0.57f;
constexpr float kStashSearchRadius=36.0f;

ResourceBundle salvage_yield(){
    ResourceBundle bundle;
    bundle.iron=2400;
    bundle.aluminum=1800;
    bundle.copper=600;
    bundle.silicon=300;
    bundle.plastic=300;
    return bundle;
}

bool overlaps_deployment_yard(const LunarLanderState& lander,Vec3 point,float width){
    return lander.active()&&!lunar_lander_deployment_clear(lander,point,0,width,width,1);
}
}

bool GameApp::stash_ground_clear(Vec3 p,float width) const {
    const float half=playfield_.size()*.5f-1-width*.5f;
    if(!std::isfinite(p.x)||!std::isfinite(p.z)||std::abs(p.x)>half||std::abs(p.z)>half) return false;
    const auto area=footprint(p,0,width,width);
    for(const auto& h:placed_habitats_){
        const auto& mesh=habitat_models_.mesh(h);
        if(overlaps(area,footprint(h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z),1)) return false;
    }
    for(const auto& item:placed_resources_){
        const auto& mesh=resource_models_.mesh(item);
        if(overlaps(area,footprint(item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z),1)) return false;
    }
    const auto route_clear=[&](const auto& connections,float route_width){
        for(const auto& connection:connections) for(std::size_t i=1;i<connection.path.size();++i){
            const auto a=connection.path[i-1],b=connection.path[i];
            if(world_math::point_segment_distance_xz(p,a,b)<(width+route_width)*.5f+1) return false;
        }
        return true;
    };
    return route_clear(corridor_network_.connections(),4)
        && route_clear(utility_network_.connections(),2)
        && route_clear(walkway_network_.connections(),3)
        && trader_placement_clear(p,0,width,width);
}

bool GameApp::lander_site_valid(Vec3 p) const {
    const float half=playfield_.size()*.5f-1;
    if(lander_.legacy_parking){
        if(p.x-6<-half||p.x+14>half||p.z-7<-half||p.z+13>half) return false;
        return stash_ground_clear(p+Vec3{4,0,3},20);
    }
    if(p.x-8<-half||p.x+8>half||p.z-6<-half||p.z+14.9f>half) return false;
    return stash_ground_clear(p+Vec3{0,0,4.45f},20.9f);
}

bool GameApp::stash_site_valid(Vec3 p) const {
    if(distance_xz(p,lander_.site)>24||!stash_ground_clear(p,kStashMarkerRadius*2)
       ||overlaps_deployment_yard(lander_,p,kStashMarkerRadius*2)) return false;
    for(const auto& robot:builder_robots_)
        if(distance_xz(p,robot.position)<kStashMarkerRadius+1) return false;
    for(const auto& box:ground_resource_piles_)
        if(distance_xz(p,box.position)<kStashMarkerRadius+box.radius) return false;
    return true;
}

bool GameApp::pick_lander_placement_ground(Vec3& hit) const {
    const float y=static_cast<float>(input_.mouse_y)/ui_scale_;
    return y>=layout::STATUS_BAR_H && y<static_cast<float>(viewport_height_)/ui_scale_-layout::BUILD_BAR_H &&
           !mouse_over_playfield_overlay(input_) && !mouse_over_selection_panel(input_) &&
           camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit);
}

bool GameApp::confirm_lander_site(Vec3 p){
    if(!lander_.awaiting_site||!lander_site_valid(p)){
        landing_message_="LANDER, RAMP AND ROBOT APRON MUST FIT IN CLEAR GROUND";
        return false;
    }
    p.y=0;
    lander_.site=p;
    lander_.awaiting_site=false;
    landing_message_.clear();
    return true;
}

bool GameApp::confirm_stash_site(Vec3 p){
    if(!lander_.awaiting_stash || lander_.phase<LanderPhase::Landed) return false;
    if(!stash_site_valid(p)){
        landing_message_="PLACE THE STASH MARKER ON CLEAR GROUND NEAR THE LANDER";
        return false;
    }
    p.y=0;
    if(!plan_stash_boxes(lander_.remaining,p)){
        landing_message_="CHOOSE A MARKER WITH MORE CLEAR GROUND NEARBY";
        return false;
    }
    const auto old_center=lander_.stash_center;
    lander_.stash_center=p;
    if(!repack_lander_stash()){
        lander_.stash_center=old_center;
        return false;
    }
    starter_stash_center_=p;
    lander_.awaiting_stash=false;
    landing_message_.clear();
    return true;
}

bool GameApp::prepare_default_landing(){
    const float half=playfield_.size()*.5f-1;
    const auto nearest=[&](Vec3 origin,const auto& valid)->std::optional<Vec3>{
        std::optional<Vec3> chosen;
        float best=std::numeric_limits<float>::infinity();
        for(float x=-half;x<=half;x+=2) for(float z=-half;z<=half;z+=2){
            const Vec3 p{x,0,z};
            const float d=distance_xz(p,origin);
            if(d<best && valid(p)){best=d;chosen=p;}
        }
        return chosen;
    };
    if(lander_.awaiting_site){
        const auto site=nearest({},[&](Vec3 p){return lander_site_valid(p);});
        if(!site||!confirm_lander_site(*site)) return false;
    }
    if(!lander_.awaiting_stash) return true;
    const auto marker=nearest(lander_.site,[&](Vec3 p){return stash_site_valid(p);});
    return marker&&confirm_stash_site(*marker);
}

std::optional<std::vector<GroundResourcePile>> GameApp::plan_stash_boxes(const ResourceBundle& cargo,Vec3 marker) const {
    std::vector<GroundResourcePile> stacks;
    if(cargo.empty()) return stacks;
    struct Site {Vec3 position;float aisle_side;};
    std::vector<Site> sites;
    constexpr int pairs=(int)(kStashSearchRadius/kPairPitch);
    constexpr int rows=(int)(kStashSearchRadius/kStackPitch);
    for(int x=-pairs;x<=pairs;++x) for(int z=-rows;z<=rows;++z) for(float side:{-1.f,1.f}){
        const Vec3 offset{x*kPairPitch+side*kHalfPair,0,z*kStackPitch};
        const float distance=eng::length(offset);
        if(distance>=kStashMarkerRadius+.40f&&distance<=kStashSearchRadius)
            sites.push_back({marker+offset,side});
    }
    std::stable_sort(sites.begin(),sites.end(),[&](const Site& a,const Site& b){
        return distance_xz(a.position,marker)<distance_xz(b.position,marker);
    });
    const auto clear_site=[&](const Site& site){
        const Vec3 p=site.position;
        if(overlaps_deployment_yard(lander_,p,1.2f)||!stash_ground_clear(p,1.2f)) return false;
        // Paired stacks share their inside edge. The outside of each stack
        // must have room for a rover to approach and work.
        const Vec3 service=p+Vec3{site.aisle_side*1.45f,0,0};
        if(!stash_ground_clear(service,1.1f)||lunar_lander_segment_blocked(lander_,service,service,1.0f))
            return false;
        for(const auto& box:ground_resource_piles_){
            if(distance_xz(p,box.position)<std::max(kStackSeparation,box.radius+.33f)||
               distance_xz(service,box.position)<box.radius+1.1f) return false;
        }
        for(const auto& lot:trader_.lots)
            if(lot.remaining>0&&(distance_xz(p,lot.position)<1.1f||distance_xz(service,lot.position)<1.3f)) return false;
        for(const auto& rover:builder_robots_){
            if(distance_xz(p,rover.position)<1.5f) return false;
            if(robots::contract_intersects_region(rover.safety.active_contract,[&](Vec3 point,float radius){
                return distance_xz(point,p)<radius+.6f;
            })) return false;
        }
        return first_person_mode_!=FirstPersonMode::Active||distance_xz(p,first_person_.feet_position())>1.1f;
    };
    auto site=sites.begin();
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto material=(MaterialResource)i;
        auto remaining=cargo.amount(material);
        const auto stack_capacity=kGroundStackLayers*resource_package_capacity_kg(material);
        while(remaining>0){
            site=std::find_if(site,sites.end(),clear_site);
            if(site==sites.end()) return std::nullopt;
            const auto amount=std::min(remaining,stack_capacity);
            stacks.push_back({material,amount,site->position,resource_package_half_width(material,amount),1,GroundPileKind::Stash});
            ++site;
            remaining-=amount;
        }
    }
    return stacks;
}

bool GameApp::add_stash_boxes(const ResourceBundle& cargo,bool credit){
    auto stacks=plan_stash_boxes(cargo,lander_.stash_center);
    if(!stacks){
        landing_message_="CLEAR SPACE NEAR THE STASH MARKER";
        return false;
    }
    ground_resource_piles_.insert(ground_resource_piles_.end(),stacks->begin(),stacks->end());
    if(credit) resources_.add(cargo);
    return true;
}

bool GameApp::drop_lander_cargo(){
    if(!lander_.boxed || (!lander_.remaining.empty() && !add_stash_boxes(lander_.remaining))) return false;
    for(std::size_t i=0;i<colony_seeds_.size();++i)colony_seeds_[i]+=lander_.seeds_aboard[i];
    lander_.remaining={};
    lander_.seeds_aboard={};
    return true;
}

bool GameApp::repack_lander_stash(){
    if(lander_.boxed) return true;
    const auto original=ground_resource_piles_;
    ResourceBundle cargo;
    std::erase_if(ground_resource_piles_,[&](const auto& pile){
        if(distance_xz(pile.position,lunar_lander_stash_position(lander_,pile.resource))>.15f) return false;
        cargo.amount(pile.resource)+=pile.amount;
        return true;
    });
    if(!add_stash_boxes(cargo,false)){
        ground_resource_piles_=original;
        return false;
    }
    lander_.boxed=true;
    return true;
}

void GameApp::migrate_boxed_lander_stash(bool legacy_grid){
    // Detect the exact former crate grid. Produced and relocated boxes remain where they are.
    int slots=16;
    const auto recovered=salvage_yield();
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto material=(MaterialResource)i;
        const auto cap=resource_package_capacity_kg(material);
        slots+=(int)((lander_.manifest.amount(material)+cap-1)/cap+(recovered.amount(material)+cap-1)/cap);
    }
    const int columns=std::clamp((int)std::ceil(std::sqrt(slots/3.0f)),1,32);
    const float depth=((slots+columns-1)/columns)*1.2f+1;
    const Vec3 first=lander_.stash_center+Vec3{-columns*1.7f+1.7f,0,-depth*.5f+1};
    ResourceBundle cargo;
    auto original=ground_resource_piles_;
    for(auto& pile:original){
        if(legacy_grid){
            const auto offset=pile.position-first;
            const int x=(int)std::round(offset.x/3.4f),z=(int)std::round(offset.z/1.2f);
            if(x<0||x>=columns||z<0||z*columns+x>=slots||
               std::abs(offset.y)>.05f||std::abs(offset.x-x*3.4f)>.05f||std::abs(offset.z-z*1.2f)>.05f) continue;
            pile.kind=GroundPileKind::Stash;
        }
        if(pile.kind==GroundPileKind::Stash) cargo.amount(pile.resource)+=pile.amount;
    }
    if(cargo.empty()) return;
    ground_resource_piles_=original;
    std::erase_if(ground_resource_piles_,[](const auto& pile){return pile.kind==GroundPileKind::Stash;});
    if(!add_stash_boxes(cargo,false)) ground_resource_piles_=std::move(original);
}

bool GameApp::salvage_lander(){
    if(!lander_.active()||lander_.phase!=LanderPhase::Ready||!lander_.remaining.empty()) return false;
    if(lander_.salvage_seconds>=kLanderSalvageSeconds) return complete_lander_salvage();
    if(lander_.salvage_requested) return false;
    lander_.salvage_requested=true;
    landing_message_="SALVAGE QUEUED / WAITING FOR A BUILDER";
    return true;
}

bool GameApp::complete_lander_salvage(){
    // The hull stays authoritative until the recovered stock has a safe home.
    lander_.salvage_worker=-1;
    if(!add_stash_boxes(salvage_yield())) return false;
    lander_.salvaged=true;
    lander_.salvage_requested=false;
    if(selection_kind_==SelectionKind::Lander) selection_kind_=SelectionKind::Stash;
    nav_wall_cache_.reset();
    landing_message_="LANDER SALVAGED / RECOVERED 5400 KG";
    return true;
}

void GameApp::render_stash(eng::Renderer& r){
    if(!lander_.has_depot()) return;
    const auto rectangle=[&](Vec3 center,float x,float z,eng::Color color){
        constexpr float y=.045f;
        const Vec3 a=center+Vec3{-x,y,-z},b=center+Vec3{x,y,-z},c=center+Vec3{x,y,z},d=center+Vec3{-x,y,z};
        r.draw_lines_3d({a,b,b,c,c,d,d,a},color,1.5f);
    };
    Vec3 marker=lander_.stash_center;
    bool valid=true;
    if(lander_.awaiting_site||lander_.awaiting_stash){
        if(!pick_lander_placement_ground(marker)) return;
        if(lander_.awaiting_site){
            rectangle(marker+Vec3{0,0,2.6f},6.1f,8.7f,lander_site_valid(marker)?eng::Color{.2f,.95f,.65f,1}:eng::Color{1,.2f,.15f,1});
            return;
        }
        valid=stash_site_valid(marker);
    }else if(!lander_.boxed || lander_.phase<LanderPhase::Landed) return;
    const eng::Color color=!valid?eng::Color{1,.2f,.15f,1}:
        lander_.awaiting_stash||selection_kind_==SelectionKind::Stash?eng::Color{.2f,.95f,.65f,1}:eng::Color{.95f,.72f,.26f,1};
    const float h=kStashMarkerRadius;
    marker.y=.045f;
    const Vec3 a=marker+Vec3{-h,0,0},b=marker+Vec3{0,0,-h},c=marker+Vec3{h,0,0},d=marker+Vec3{0,0,h};
    const Vec3 tip=marker+Vec3{0,1.15f,0};
    r.draw_lines_3d({a,b,b,c,c,d,d,a,a,c,b,d,marker,tip,tip,tip+Vec3{.55f,-.3f,0},tip+Vec3{.55f,-.3f,0},tip+Vec3{0,-.60f,0}},color,2);
}

void GameApp::render_stash_panel(eng::Renderer& r){
    using namespace layout;
    sync_selection_panel_scroll();
    const auto panel=selection_panel_rect();
    layout::hud_panel(r,panel,theme_);
    const auto label=[&](float offset,const std::string& text,float scale){
        const float y=panel.y+offset-selection_panel_scroll_;
        if(y>=panel.y+4&&y+14<=panel.y+panel.h-4)
            game_label_fitted(r,{panel.x+14,y,panel.w-28,14},text,scale,theme_);
    };
    label(12,"RESOURCE STASH",.65f);
    ResourceBundle stock;
    std::int64_t boxes=0;
    int stacks=0;
    for(const auto& pile:ground_resource_piles_) if(pile.kind==GroundPileKind::Stash){
        stock.amount(pile.resource)+=pile.amount;
        const auto capacity=resource_package_capacity_kg(pile.resource);
        boxes+=(pile.amount+capacity-1)/capacity;
        ++stacks;
    }
    label(42,std::to_string(boxes)+" BOXES / "+std::to_string(stacks)+" STACKS / "+std::to_string(stock.total_amount())+" KG",.44f);
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto material=(MaterialResource)i;
        label(70+i*19,std::string(material_resource_name(material))+"  "+std::to_string(stock.amount(material))+" KG",.43f);
    }
    label(224,"SEED PACKETS IN COLONY STOCK  "+std::to_string(seed_packet_total(colony_seeds_)),.43f);
    if(!landing_message_.empty()) label(245,landing_message_,.38f);
    if(selection_panel_max_scroll()>0)
        r.draw_rect({panel.x+panel.w-5,panel.y+5+(panel.h-30)*selection_panel_scroll_/selection_panel_max_scroll(),3,20},theme_.accent);
}
}
