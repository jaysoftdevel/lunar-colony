#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/world/world_labels.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace game {
using namespace eng;
using namespace layout;
using namespace world_labels;

namespace {

void draw_resource_icon(Renderer& renderer,MaterialResource resource,Rect box,Color c){
    const eng::ui::IconCanvas r(renderer,box,{22,28});
    const float x=0,y=0,w=22,h=28;
    const float cx=x+w*0.5f,cy=y+h*0.5f;
    switch(resource){
        case MaterialResource::Iron:
            r.draw_rect({x+3,y+8,w-6,h-14},{c.r,c.g,c.b,0.15f});
            r.draw_line({x+3,y+8},{x+w-3,y+8},c,1.8f);
            r.draw_line({x+3,y+h-6},{x+w-3,y+h-6},c,1.8f);
            r.draw_line({x+3,y+8},{x+7,y+3},c,1.4f);
            r.draw_line({x+w-3,y+8},{x+w-7,y+3},c,1.4f);
            r.draw_line({x+7,y+3},{x+w-7,y+3},c,1.4f);
            break;
        case MaterialResource::Copper:
            for(int i=0;i<3;++i){
                const float yy=y+5.0f+i*6.0f;
                r.draw_line({x+4,yy},{x+w-4,yy+3.0f},c,1.7f);
                r.draw_line({x+w-4,yy+3.0f},{x+4,yy+6.0f},c,1.7f);
            }
            r.draw_line({x+4,y+4},{x+4,y+h-3},c,1.0f);
            r.draw_line({x+w-4,y+7},{x+w-4,y+h-3},c,1.0f);
            break;
        case MaterialResource::Silicon:
            r.draw_rect({cx-7,cy-7,14,14},{c.r,c.g,c.b,0.13f});
            r.draw_line({cx-7,cy-7},{cx+7,cy-7},c,1.6f);
            r.draw_line({cx+7,cy-7},{cx+7,cy+7},c,1.6f);
            r.draw_line({cx+7,cy+7},{cx-7,cy+7},c,1.6f);
            r.draw_line({cx-7,cy+7},{cx-7,cy-7},c,1.6f);
            for(int i=-1;i<=1;++i){
                const float o=(float)i*5.0f;
                r.draw_line({cx+o,cy-11},{cx+o,cy-7},c,1.2f);
                r.draw_line({cx+o,cy+7},{cx+o,cy+11},c,1.2f);
            }
            break;
        case MaterialResource::Plastic:
            r.draw_rect({x+3,y+5,7,7},{c.r,c.g,c.b,0.14f});
            r.draw_rect({x+12,y+10,8,8},{c.r,c.g,c.b,0.14f});
            r.draw_rect({x+5,y+18,9,7},{c.r,c.g,c.b,0.14f});
            r.draw_line({x+3,y+5},{x+10,y+5},c,1.2f);
            r.draw_line({x+12,y+10},{x+20,y+10},c,1.2f);
            r.draw_line({x+5,y+18},{x+14,y+18},c,1.2f);
            break;
        case MaterialResource::Aluminum:
            r.draw_rect({x+4,y+7,w-8,h-12},{c.r,c.g,c.b,0.12f});
            r.draw_line({x+4,y+7},{x+w-4,y+7},c,1.5f);
            r.draw_line({x+4,y+h-5},{x+w-4,y+h-5},c,1.5f);
            r.draw_line({x+4,y+7},{x+8,y+3},c,1.3f);
            r.draw_line({x+w-4,y+7},{x+w-8,y+3},c,1.3f);
            r.draw_line({x+8,y+3},{x+w-8,y+3},c,1.3f);
            r.draw_line({cx,y+7},{cx,y+h-5},c,1.0f);
            break;
        case MaterialResource::Water:
            // Droplet.
            r.draw_line({cx,cy-11},{cx-8,cy+2},c,1.7f);
            r.draw_line({cx-8,cy+2},{cx-5,cy+9},c,1.7f);
            r.draw_line({cx-5,cy+9},{cx+5,cy+9},c,1.7f);
            r.draw_line({cx+5,cy+9},{cx+8,cy+2},c,1.7f);
            r.draw_line({cx+8,cy+2},{cx,cy-11},c,1.7f);
            break;
        case MaterialResource::Food:
            // Compact ration crate / leaf mark.
            r.draw_rect({cx-9,cy-7,18,14},{c.r,c.g,c.b,0.12f});
            r.draw_line({cx-9,cy-7},{cx+9,cy-7},c,1.5f);
            r.draw_line({cx-9,cy+7},{cx+9,cy+7},c,1.5f);
            r.draw_line({cx,cy-4},{cx,cy+5},c,1.2f);
            r.draw_line({cx,cy-1},{cx+5,cy-5},c,1.2f);
            break;
        case MaterialResource::Medical:
            // Medical cross.
            r.draw_rect({cx-3,cy-10,6,20},{c.r,c.g,c.b,0.18f});
            r.draw_rect({cx-10,cy-3,20,6},{c.r,c.g,c.b,0.18f});
            r.draw_line({cx-3,cy-10},{cx+3,cy-10},c,1.5f);
            r.draw_line({cx-10,cy-3},{cx-10,cy+3},c,1.5f);
            r.draw_line({cx+10,cy-3},{cx+10,cy+3},c,1.5f);
            r.draw_line({cx-3,cy+10},{cx+3,cy+10},c,1.5f);
            break;
        case MaterialResource::Count:
            break;
    }
}

std::string amount_text(std::int64_t value){
    std::ostringstream out;
    const std::int64_t magnitude=value<0?-value:value;
    if(magnitude>=1000000){
        out<<std::fixed<<std::setprecision(magnitude>=10000000?0:1)<<(double)value/1000000.0<<"M";
    }else if(magnitude>=10000){
        out<<std::fixed<<std::setprecision(magnitude>=100000?0:1)<<(double)value/1000.0<<"K";
    }else{
        out<<value;
    }
    return out.str();
}

bool contains(Rect r,float x,float y){
    return x>=r.x && x<=r.x+r.w && y>=r.y && y<=r.y+r.h;
}

} // namespace

void GameApp::render_status_bar(Renderer& r){
    const float logical_w=ui_width(r);
    r.draw_rect({0,0,logical_w,STATUS_BAR_H},{0.025f,0.035f,0.045f,0.97f});
    r.draw_line({0,STATUS_BAR_H-1},{logical_w,STATUS_BAR_H-1},theme_.border,1.0f);

    const int total_minutes=(int)std::floor(status_.hour()*60.0f);
    const int hour=(total_minutes/60)%24;
    const int minute=total_minutes%60;
    double energy=0,energy_capacity=0,air_mass=0,air_capacity=0,water_stored=0,water_capacity=0;
    const auto collect_air=[&](const SupplyState& supply,double volume){
        const double capacity=volume*kOxygenKgPerM3;
        if(!std::isfinite(capacity) || capacity<=0)return;
        air_capacity+=capacity;
        if(std::isfinite(supply.oxygen_kg))air_mass+=std::clamp(supply.oxygen_kg,0.0,capacity);
    };
    for(const auto& h:placed_habitats_)if(h.construction.complete()){
        const auto& m=habitat_models_.mesh(h);
        collect_air(h.supply,h.supply.volume_m3>0?h.supply.volume_m3:m.footprint_x*m.footprint_z*std::max(0.0f,m.height-m.walkable_floor_y)*0.65);
    }
    for(const auto& item:placed_resources_)if(item.construction.complete()){
        if(item.kind==ResourceKind::PowerStorage){
            const double capacity=item.size==ResourceSizeTier::Small?30.0:60.0;energy_capacity+=capacity;
            if(std::isfinite(item.supply.energy_kwh))energy+=std::clamp(item.supply.energy_kwh,0.0,capacity);
        }else if(item.kind==ResourceKind::WaterStorage){
            const double capacity=item.size==ResourceSizeTier::Small?1000.0:2000.0;water_capacity+=capacity;
            if(std::isfinite(item.supply.water_litres))water_stored+=std::clamp(item.supply.water_litres,0.0,capacity);
        }else if(item.kind==ResourceKind::StorageDome){
            const auto& m=resource_models_.mesh(item);
            collect_air(item.supply,item.supply.volume_m3>0?item.supply.volume_m3:m.footprint_x*m.footprint_z*std::max(0.0f,m.height-m.walkable_floor_y)*0.65);
        }
    }
    const double charge=energy_capacity>0?energy/energy_capacity:0;
    const double air_fraction=air_capacity>0?air_mass/air_capacity:0;
    std::ostringstream day,power,air,water,crew;
    day<<"DAY "<<status_.day()<<"  "<<std::setfill('0')<<std::setw(2)<<hour<<":"<<std::setw(2)<<minute<<"  "<<(status_.is_daylight()?"LIGHT":"NIGHT");
    power<<"POWER "<<std::fixed<<std::setprecision(1)<<energy<<"/"<<energy_capacity<<" KWH";
    if(air_capacity>0)air<<"AIR "<<std::fixed<<std::setprecision(1)<<100*air_fraction<<"%";
    else air<<"AIR --";
    water<<"WATER "<<std::fixed<<std::setprecision(1)<<water_stored<<"/"<<water_capacity<<" L";
    crew<<"CREW "<<status_.crew_current()<<"/"<<status_.crew_capacity();

    // Keep enough room for all inventory resources while retaining colony status.
    const float day_w=logical_w*0.16f;
    const float power_w=logical_w*0.12f;
    const float air_w=logical_w*0.10f;
    const float water_w=logical_w*0.08f;
    const float crew_w=logical_w*0.07f;
    const float resource_start=day_w+power_w+air_w+water_w+crew_w;
    const float resource_w=(logical_w-resource_start)/(float)kMaterialResourceCount;

    const std::array<float,5> split={day_w,day_w+power_w,day_w+power_w+air_w,day_w+power_w+air_w+water_w,resource_start};
    for(float x:split) r.draw_line({x,8},{x,STATUS_BAR_H-8},theme_.border,1.0f);

    game_label_fitted(r,{9,9,day_w-16,18},day.str(),0.66f,theme_);
    game_label_fitted(r,{day_w+7,5,power_w-14,14},power.str(),0.58f,theme_);
    game_label_fitted(r,{day_w+power_w+7,9,air_w-14,18},air.str(),0.58f,theme_);
    game_label_fitted(r,{day_w+power_w+air_w+7,5,water_w-14,14},water.str(),0.58f,theme_);
    game_label_fitted(r,{day_w+power_w+air_w+water_w+7,9,crew_w-14,18},crew.str(),0.58f,theme_);
    const auto bar=[&](float x,float width,double fraction,Color color){
        r.draw_rect({x+7,33,width-14,6},{0.08f,0.13f,0.17f,1});
        r.draw_rect({x+7,33,(width-14)*(float)std::clamp(fraction,0.0,1.0),6},color);
    };
    bar(day_w,power_w,charge,charge<0.20?Color{1,0.38f,0.30f,1}:Color{1,0.78f,0.22f,1});
    bar(day_w+power_w,air_w,air_fraction,air_fraction<0.95?Color{1,0.38f,0.30f,1}:Color{0.25f,0.86f,0.98f,1});
    const double water_fraction=water_capacity>0?water_stored/water_capacity:0;
    bar(day_w+power_w+air_w,water_w,water_fraction,water_fraction<0.10?Color{1,0.38f,0.30f,1}:Color{0.25f,0.65f,1,1});
    const auto rate=[&](float x,float width,double net,double load,const char* unit,const char* load_label){
        std::ostringstream text;
        if(game_speed_==GameSpeed::Pause)text<<"RATE PAUSED";
        else text<<"RATE "<<std::showpos<<std::fixed<<std::setprecision(1)<<(std::abs(net)<0.05?0:net)<<" "<<unit
                 <<std::noshowpos<<" / "<<load_label<<" "<<std::setprecision(1)<<std::max(0.0,load);
        game_label_fitted(r,{x+7,21,width-14,10},text.str(),0.43f,theme_,true);
    };
    rate(day_w,power_w,battery_net_kw_,supply_power_load_kw_,"KW","LOAD");
    rate(day_w+power_w+air_w,water_w,water_net_litres_per_day_,supply_water_load_litres_per_day_,"L/D","USE");


    const float mouse_x=(float)ui_input_.mouse_x;
    const float mouse_y=(float)ui_input_.mouse_y;
    int hovered=-1;
    Rect hovered_rect{};
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto resource=static_cast<MaterialResource>(i);
        const float x=resource_start+(float)i*resource_w;
        const Rect cell{x,0,resource_w,STATUS_BAR_H};
        if(i>0) r.draw_line({x,8},{x,STATUS_BAR_H-8},theme_.border,1.0f);
        const Rect icon{x+5,8,22,28};
        const bool negative=resources_.amount(resource)<0;
        const Color icon_color=negative?Color{1.0f,0.38f,0.30f,1.0f}:theme_.accent;
        draw_resource_icon(r,resource,icon,icon_color);
        game_label_fitted(r,{x+29,13,resource_w-33,20},amount_text(resources_.amount(resource)),0.57f,theme_);
        if(contains(cell,mouse_x,mouse_y)){ hovered=(int)i; hovered_rect=cell; }
    }

    std::string status_tip;
    const Rect power_cell{day_w,0,power_w,STATUS_BAR_H},air_cell{day_w+power_w,0,air_w,STATUS_BAR_H},water_cell{day_w+power_w+air_w,0,water_w,STATUS_BAR_H};
    if(contains(power_cell,mouse_x,mouse_y)){
        std::ostringstream tip;tip<<"BATTERY CHARGE "<<std::fixed<<std::setprecision(1)<<charge*100<<"% / LOAD "<<(game_speed_==GameSpeed::Pause?0:supply_power_load_kw_)<<" KW / RATE IS NET STORED ENERGY";
        status_tip=energy_capacity>0?tip.str():"NO COMPLETED BATTERIES";hovered_rect=power_cell;
    }else if(contains(air_cell,mouse_x,mouse_y)){
        status_tip=air_capacity>0?"AVERAGE INDOOR AIR / WEIGHTED BY VOLUME":"NO COMPLETED INDOOR STRUCTURES";hovered_rect=air_cell;
    }
    if(contains(water_cell,mouse_x,mouse_y)){
        std::ostringstream tip;tip<<"TANK FILL "<<std::fixed<<std::setprecision(1)<<water_fraction*100<<"% / USE "<<(game_speed_==GameSpeed::Pause?0:supply_water_load_litres_per_day_)<<" L/D / RATE IS NET TANK FLOW";
        status_tip=water_capacity>0?tip.str():"NO COMPLETED WATER TANKS";hovered_rect=water_cell;
    }
    if(hovered>=0 || !status_tip.empty()){
        const std::string name=status_tip.empty()?material_resource_name(static_cast<MaterialResource>(hovered)):status_tip;
        const float tooltip_w=std::min(logical_w-8,std::max(72.0f,game_text_width(name,0.62f,theme_)+20.0f));
        const float tx=std::clamp(hovered_rect.x+(hovered_rect.w-tooltip_w)*0.5f,4.0f,logical_w-tooltip_w-4.0f);
        const Rect tooltip{tx,STATUS_BAR_H+4.0f,tooltip_w,28.0f};
        r.draw_rect(tooltip,{0.025f,0.035f,0.045f,0.96f});
        r.draw_line({tooltip.x,tooltip.y},{tooltip.x+tooltip.w,tooltip.y},theme_.accent,1.5f);
        game_label_fitted(r,{tooltip.x+10,tooltip.y+5,tooltip.w-20,18},name,0.62f,theme_);
    }
}

} // namespace game
