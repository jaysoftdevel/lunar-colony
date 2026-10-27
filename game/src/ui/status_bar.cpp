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

void draw_resource_icon(Renderer& r,MaterialResource resource,Rect box,Color c){
    const float x=box.x,y=box.y,w=box.w,h=box.h;
    const float cx=x+w*0.5f,cy=y+h*0.5f;
    switch(resource){
        case MaterialResource::Iron:
            // Compact ingot.
            r.draw_rect({x+3,y+8,w-6,h-14},{c.r,c.g,c.b,0.15f});
            r.draw_line({x+3,y+8},{x+w-3,y+8},c,1.8f);
            r.draw_line({x+3,y+h-6},{x+w-3,y+h-6},c,1.8f);
            r.draw_line({x+3,y+8},{x+7,y+3},c,1.4f);
            r.draw_line({x+w-3,y+8},{x+w-7,y+3},c,1.4f);
            r.draw_line({x+7,y+3},{x+w-7,y+3},c,1.4f);
            break;
        case MaterialResource::Copper:
            // Copper coil / wound conductor.
            for(int i=0;i<3;++i){
                const float yy=y+5.0f+i*6.0f;
                r.draw_line({x+4,yy},{x+w-4,yy+3.0f},c,1.7f);
                r.draw_line({x+w-4,yy+3.0f},{x+4,yy+6.0f},c,1.7f);
            }
            r.draw_line({x+4,y+4},{x+4,y+h-3},c,1.0f);
            r.draw_line({x+w-4,y+7},{x+w-4,y+h-3},c,1.0f);
            break;
        case MaterialResource::Silicon:
            // Microchip die with pins.
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
            // Polymer pellets / moulded stock.
            r.draw_rect({x+3,y+5,7,7},{c.r,c.g,c.b,0.14f});
            r.draw_rect({x+12,y+10,8,8},{c.r,c.g,c.b,0.14f});
            r.draw_rect({x+5,y+18,9,7},{c.r,c.g,c.b,0.14f});
            r.draw_line({x+3,y+5},{x+10,y+5},c,1.2f);
            r.draw_line({x+12,y+10},{x+20,y+10},c,1.2f);
            r.draw_line({x+5,y+18},{x+14,y+18},c,1.2f);
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
    std::ostringstream day,power,water,crew;
    day<<"DAY "<<status_.day()<<"  "<<std::setfill('0')<<std::setw(2)<<hour<<":"<<std::setw(2)<<minute<<"  "<<(status_.is_daylight()?"LIGHT":"NIGHT");
    power<<"POWER "<<status_.power_current()<<"/"<<status_.power_capacity();
    water<<"WATER "<<status_.water_current()<<"/"<<status_.water_capacity();
    crew<<"CREW "<<status_.crew_current()<<"/"<<status_.crew_capacity();

    const float day_w=logical_w*0.24f;
    const float power_w=logical_w*0.13f;
    const float water_w=logical_w*0.13f;
    const float crew_w=logical_w*0.11f;
    const float resource_start=day_w+power_w+water_w+crew_w;
    const float resource_w=(logical_w-resource_start)/4.0f;

    const std::array<float,5> split={day_w,day_w+power_w,day_w+power_w+water_w,resource_start,logical_w};
    for(std::size_t i=0;i+1<split.size();++i) r.draw_line({split[i],8},{split[i],STATUS_BAR_H-8},theme_.border,1.0f);

    game_label(r,{9,12},day.str(),0.70f,theme_);
    game_label(r,{day_w+9,12},power.str(),0.64f,theme_);
    game_label(r,{day_w+power_w+9,12},water.str(),0.64f,theme_);
    game_label(r,{day_w+power_w+water_w+9,12},crew.str(),0.64f,theme_);

    const float mouse_x=(float)ui_input_.mouse_x;
    const float mouse_y=(float)ui_input_.mouse_y;
    int hovered=-1;
    Rect hovered_rect{};
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto resource=(MaterialResource)i;
        const float x=resource_start+(float)i*resource_w;
        const Rect cell{x,0,resource_w,STATUS_BAR_H};
        if(i>0) r.draw_line({x,8},{x,STATUS_BAR_H-8},theme_.border,1.0f);
        const Rect icon{x+7,8,24,28};
        const bool negative=resources_.amount(resource)<0;
        const Color icon_color=negative?Color{1.0f,0.38f,0.30f,1.0f}:theme_.accent;
        draw_resource_icon(r,resource,icon,icon_color);
        game_label(r,{x+35,13},amount_text(resources_.amount(resource)),0.65f,theme_);
        if(contains(cell,mouse_x,mouse_y)){ hovered=(int)i; hovered_rect=cell; }
    }

    if(hovered>=0){
        const auto resource=(MaterialResource)hovered;
        const std::string name=material_resource_name(resource);
        const float tooltip_w=std::max(72.0f,text_width(name,0.62f,theme_)+20.0f);
        const float tx=std::clamp(hovered_rect.x+(hovered_rect.w-tooltip_w)*0.5f,4.0f,logical_w-tooltip_w-4.0f);
        const Rect tooltip{tx,STATUS_BAR_H+4.0f,tooltip_w,28.0f};
        r.draw_rect(tooltip,{0.025f,0.035f,0.045f,0.96f});
        r.draw_line({tooltip.x,tooltip.y},{tooltip.x+tooltip.w,tooltip.y},theme_.accent,1.5f);
        game_label(r,{tooltip.x+10,tooltip.y+7},name,0.62f,theme_);
    }
}

} // namespace game
