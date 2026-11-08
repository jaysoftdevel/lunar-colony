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
    const bool paused=game_speed_==GameSpeed::Pause;
    if(ui_time_seconds_>=status_rate_display_.next_sample || paused!=status_rate_display_.paused){
        status_rate_display_={ui_time_seconds_+kStatusRateRefreshSeconds,paused,
            battery_net_kw_,supply_power_load_kw_,supply_power_delivered_kw_,water_net_litres_per_day_,supply_water_load_litres_per_day_};
    }
    const auto& rates=status_rate_display_;
    r.draw_rect({0,0,logical_w,STATUS_BAR_H},kHudFill);
    r.draw_line({0,STATUS_BAR_H-1},{logical_w,STATUS_BAR_H-1},theme_.border,1.0f);

    const int total_minutes=(int)std::floor(status_.hour()*60.0f);
    const int hour=(total_minutes/60)%24;
    const int minute=total_minutes%60;
    double energy=0,energy_capacity=0,air_mass=0,air_capacity=0,water_stored=0,water_capacity=0,hydrogen=0,oxygen_stock=0;
    std::int64_t storage_used=0,storage_capacity=0;
    const auto collect_air=[&](const SupplyState& supply,double volume){
        const double capacity=volume*kOxygenKgPerM3;
        if(!std::isfinite(capacity) || capacity<=0)return;
        air_capacity+=capacity;
        if(std::isfinite(supply.oxygen_kg))air_mass+=std::clamp(supply.oxygen_kg,0.0,capacity);
    };
    for(const auto& h:placed_habitats_)if(h.construction.complete()){
        if(h.kind==HabitatKind::OxygenGeneratorDome){
            hydrogen+=std::max(0.0,std::isfinite(h.supply.hydrogen_kg)?h.supply.hydrogen_kg:0.0);
            oxygen_stock+=std::max(0.0,trader_oxygen_stock_amount(h.placement_sequence));
        }
        const auto& m=habitat_models_.mesh(h);
        collect_air(h.supply,h.supply.volume_m3>0?h.supply.volume_m3:m.footprint_x*m.footprint_z*std::max(0.0f,m.height-m.walkable_floor_y)*0.65);
    }
    for(const auto& item:placed_resources_)if(item.construction.complete()){
        if(item.kind==ResourceKind::PowerStorage){
            const double capacity=power_storage_capacity_kwh(item.size);energy_capacity+=capacity;
            if(std::isfinite(item.supply.energy_kwh))energy+=std::clamp(item.supply.energy_kwh,0.0,capacity);
        }else if(item.kind==ResourceKind::WaterStorage){
            const double capacity=item.size==ResourceSizeTier::Small?1000.0:2000.0;water_capacity+=capacity;
            if(std::isfinite(item.supply.water_litres))water_stored+=std::clamp(item.supply.water_litres,0.0,capacity);
        }else if(item.kind==ResourceKind::OxygenTank){
            oxygen_stock+=item.stored_oxygen_kg;
        }else if(item.kind==ResourceKind::HydrogenTank){
            if(std::isfinite(item.supply.hydrogen_kg))hydrogen+=std::max(0.0,item.supply.hydrogen_kg);
        }else if(item.kind==ResourceKind::StorageDome){
            storage_used+=storage_dome_used_kg(item);
            storage_capacity+=storage_dome_capacity_kg(item.storage_size);
            const auto& m=resource_models_.mesh(item);
            collect_air(item.supply,item.supply.volume_m3>0?item.supply.volume_m3:m.footprint_x*m.footprint_z*std::max(0.0f,m.height-m.walkable_floor_y)*0.65);
        }
    }
    const double charge=energy_capacity>0?energy/energy_capacity:0;
    const double air_fraction=air_capacity>0?air_mass/air_capacity:0;
    const double storage_fraction=storage_capacity>0?(double)storage_used/storage_capacity:0;
    std::ostringstream day,power,air,water,storage,crew;
    day<<"DAY "<<status_.day()<<"  "<<std::setfill('0')<<std::setw(2)<<hour<<":"<<std::setw(2)<<minute<<"  "<<(status_.is_daylight()?"LIGHT":"NIGHT");
    power<<"POWER "<<std::fixed<<std::setprecision(1)<<energy<<"/"<<energy_capacity<<" KWH";
    if(air_capacity>0)air<<"AIR "<<std::fixed<<std::setprecision(1)<<100*air_fraction<<"%";
    else air<<"AIR --";
    water<<"WATER "<<std::fixed<<std::setprecision(1)<<water_stored<<"/"<<water_capacity<<" L";
    if(storage_capacity>0)storage<<"STORAGE "<<std::fixed<<std::setprecision(1)<<storage_fraction*100<<"%";
    else storage<<"STORAGE --";
    crew<<"CREW "<<status_.crew_current()<<"/"<<status_.crew_capacity();

    // Colony telemetry and the complete inventory each own a readable row.
    constexpr float row_h=42;
    const float day_w=logical_w*0.20f;
    const float power_w=logical_w*0.22f;
    const float air_w=logical_w*0.12f;
    const float water_w=logical_w*0.21f;
    const float storage_w=logical_w*0.14f;
    const float water_x=day_w+power_w+air_w;
    const float storage_x=water_x+water_w;
    const float crew_x=storage_x+storage_w;
    const float crew_w=logical_w-crew_x;
    const float resource_start=0;
    constexpr std::size_t resource_count=kMaterialResourceCount+2;
    const float resource_w=logical_w/(float)resource_count;

    const std::array<float,5> split={day_w,day_w+power_w,water_x,storage_x,crew_x};
    for(float x:split) r.draw_line({x,8},{x,row_h-8},theme_.border,1.0f);

    if(active_game_phase_==GamePhase::RobotPreparation){
        game_label_fitted(r,{9,5,day_w-16,17},day.str(),0.60f,theme_);
        game_label_fitted(r,{9,25,day_w-16,13},"CASH EUR "+std::to_string(planning_.funds_eur),0.40f,theme_,true);
    }else game_label_fitted(r,{9,9,day_w-16,18},day.str(),0.66f,theme_);
    game_label_fitted(r,{day_w+7,5,power_w-14,14},power.str(),0.58f,theme_);
    game_label_fitted(r,{day_w+power_w+7,9,air_w-14,18},air.str(),0.58f,theme_);
    game_label_fitted(r,{water_x+7,5,water_w-14,14},water.str(),0.58f,theme_);
    game_label_fitted(r,{storage_x+7,9,storage_w-14,18},storage.str(),0.58f,theme_);
    game_label_fitted(r,{crew_x+7,9,crew_w-14,18},crew.str(),0.58f,theme_);
    const auto bar=[&](float x,float width,double fraction,Color color){
        r.draw_rect({x+7,33,width-14,6},{0.08f,0.13f,0.17f,1});
        r.draw_rect({x+7,33,(width-14)*(float)std::clamp(fraction,0.0,1.0),6},color);
    };
    bar(day_w,power_w,charge,charge<0.20?Color{1,0.38f,0.30f,1}:Color{1,0.78f,0.22f,1});
    bar(day_w+power_w,air_w,air_fraction,air_fraction<0.95?Color{1,0.38f,0.30f,1}:Color{0.25f,0.86f,0.98f,1});
    const double water_fraction=water_capacity>0?water_stored/water_capacity:0;
    bar(water_x,water_w,water_fraction,water_fraction<0.10?Color{1,0.38f,0.30f,1}:Color{0.25f,0.65f,1,1});
    bar(storage_x,storage_w,storage_fraction,storage_fraction>kStorageDomeFullWarning?Color{1,0.68f,0.10f,1}:theme_.accent);
    const auto rate=[&](float x,float width,double net,double load,const char* unit,const char* load_label){
        std::ostringstream text;
        if(paused)text<<"RATE PAUSED";
        else text<<"RATE "<<std::showpos<<std::fixed<<std::setprecision(1)<<(std::abs(net)<0.05?0:net)<<" "<<unit
                 <<std::noshowpos<<" / "<<load_label<<" "<<std::setprecision(1)<<std::max(0.0,load);
        game_label_fitted(r,{x+7,21,width-14,10},text.str(),0.43f,theme_,true);
    };
    std::ostringstream grid;
    if(paused)grid<<"GRID PAUSED";
    else grid<<"GRID "<<std::fixed<<std::setprecision(1)<<rates.power_delivered_kw<<"/"<<rates.power_load_kw<<" KW"
             <<" / BAT "<<std::showpos<<(std::abs(rates.battery_kw)<0.05?0.0:rates.battery_kw)<<" KW";
    game_label_fitted(r,{day_w+7,21,power_w-14,10},grid.str(),0.43f,theme_,true);
    rate(water_x,water_w,rates.water_net,rates.water_load,"L/D","USE");


    const float mouse_x=(float)ui_input_.mouse_x;
    const float mouse_y=(float)ui_input_.mouse_y;
    int hovered=-1;
    Rect hovered_rect{};
    r.draw_line({0,row_h},{logical_w,row_h},theme_.border,1);
    constexpr const char* names[]={"IRON","COPPER","SILICON","PLASTIC","WATER","FOOD","MEDICAL","ALUMINUM","OXYGEN","HYDROGEN"};
    std::array<std::string,resource_count> tips{};
    for(std::size_t i=0;i<resource_count;++i){
        const float x=resource_start+(float)i*resource_w;
        const Rect cell{x,row_h,resource_w,STATUS_BAR_H-row_h};
        if(i>0)r.draw_line({x,row_h+5},{x,STATUS_BAR_H-5},theme_.border,1);
        const Rect icon{x+5,row_h+3,16,24};
        std::string amount;
        if(i<kMaterialResourceCount){
            const auto material=(MaterialResource)i;
            draw_resource_icon(r,material,icon,resources_.amount(material)<0?Color{1,.38f,.30f,1}:theme_.accent);
            amount=amount_text(resources_.amount(material))+" KG";
            tips[i]=std::string(material_resource_name(material))+" / "+std::to_string(resources_.amount(material))+" KG AVAILABLE CARGO";
        }else{
            const double stock=i==kMaterialResourceCount?oxygen_stock:hydrogen;
            std::ostringstream exact;exact<<std::fixed<<std::setprecision(1)<<stock;
            amount=stock<10000?exact.str()+" KG":amount_text((std::int64_t)stock)+" KG";
            tips[i]=std::string(names[i])+" / "+exact.str()+" KG STORED / "+(i==kMaterialResourceCount?"TANKS + BOTTLED EXPORT STOCK; EXCLUDES CABIN AIR":"TANKS + GENERATORS; ALLOCATED VEHICLE FUEL EXCLUDED");
            r.draw_line({x+9,row_h+8},{x+21,row_h+8},theme_.accent,1.4f);
            r.draw_line({x+9,row_h+8},{x+9,row_h+23},theme_.accent,1.4f);
            r.draw_line({x+21,row_h+8},{x+21,row_h+23},theme_.accent,1.4f);
            r.draw_line({x+9,row_h+23},{x+21,row_h+23},theme_.accent,1.4f);
            r.draw_line({x+15,row_h+4},{x+15,row_h+8},theme_.accent,1.4f);
        }
        const float name_width=game_text_width(names[i],.42f,theme_);
        const float amount_width=game_text_width(amount,.49f,theme_);
        if(name_width+amount_width+33.f<=resource_w){
            game_label_fitted(r,{x+24,row_h+7,name_width+2,16},names[i],.42f,theme_,true);
            game_label_fitted(r,{x+resource_w-amount_width-5,row_h+7,amount_width+1,16},amount,.49f,theme_);
        }else{
            game_label_fitted(r,{x+23,row_h+2,resource_w-28,11},names[i],.40f,theme_,true);
            game_label_fitted(r,{x+23,row_h+14,resource_w-28,13},amount,.47f,theme_);
        }
        if(contains(cell,mouse_x,mouse_y)){hovered=(int)i;hovered_rect=cell;}
    }

    std::string status_tip;
    const Rect power_cell{day_w,0,power_w,row_h},air_cell{day_w+power_w,0,air_w,row_h},water_cell{water_x,0,water_w,row_h},storage_cell{storage_x,0,storage_w,row_h};
    if(contains(power_cell,mouse_x,mouse_y)){
        std::ostringstream tip;tip<<"BATTERY CHARGE "<<std::fixed<<std::setprecision(1)<<charge*100<<"% / LOAD "<<(paused?0:rates.power_load_kw)<<" KW / RATE IS NET STORED ENERGY";
        status_tip=energy_capacity>0?tip.str():"NO COMPLETED BATTERIES";hovered_rect=power_cell;
    }else if(contains(air_cell,mouse_x,mouse_y)){
        status_tip=air_capacity>0?"AVERAGE INDOOR AIR / WEIGHTED BY VOLUME":"NO COMPLETED INDOOR STRUCTURES";hovered_rect=air_cell;
    }
    if(contains(water_cell,mouse_x,mouse_y)){
        std::ostringstream tip;tip<<"TANK FILL "<<std::fixed<<std::setprecision(1)<<water_fraction*100<<"% / USE "<<(paused?0:rates.water_load)<<" L/D / RATE IS NET TANK FLOW";
        status_tip=water_capacity>0?tip.str():"NO COMPLETED WATER TANKS";hovered_rect=water_cell;
    }
    if(contains(storage_cell,mouse_x,mouse_y)){
        status_tip=storage_capacity>0?"STORAGE DOMES / "+std::to_string(storage_used)+" / "+
            std::to_string(storage_capacity)+" KG USED":"NO COMPLETED STORAGE DOMES";
        hovered_rect=storage_cell;
    }
    if(hovered>=0 || !status_tip.empty()){
        const std::string name=status_tip.empty()?tips[(std::size_t)hovered]:status_tip;
        const float tooltip_w=std::min(logical_w-8,std::max(72.0f,game_text_width(name,0.62f,theme_)+20.0f));
        const float tx=std::clamp(hovered_rect.x+(hovered_rect.w-tooltip_w)*0.5f,4.0f,logical_w-tooltip_w-4.0f);
        const Rect tooltip{tx,STATUS_BAR_H+4.0f,tooltip_w,28.0f};
        r.draw_rect(tooltip,kHudFill);
        r.draw_line({tooltip.x,tooltip.y},{tooltip.x+tooltip.w,tooltip.y},theme_.accent,1.5f);
        game_label_fitted(r,{tooltip.x+10,tooltip.y+5,tooltip.w-20,18},name,0.62f,theme_);
    }
}

} // namespace game
