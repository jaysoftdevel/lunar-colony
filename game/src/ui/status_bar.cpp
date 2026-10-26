#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/world/world_labels.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace game {
using namespace eng;
using namespace layout;
using namespace world_math;
using namespace world_labels;

void GameApp::render_status_bar(Renderer& r){
    const float logical_w=ui_width(r);
    const float cell=logical_w/5.0f;
    r.draw_rect({0,0,logical_w,STATUS_BAR_H},{0.025f,0.035f,0.045f,0.97f});
    r.draw_line({0,STATUS_BAR_H-1},{logical_w,STATUS_BAR_H-1},theme_.border,1.0f);

    const int total_minutes=(int)std::floor(status_.hour()*60.0f);
    const int hour=(total_minutes/60)%24;
    const int minute=total_minutes%60;
    std::ostringstream day,power,water,resources,crew;
    day<<"DAY "<<status_.day()<<"  "<<std::setfill('0')<<std::setw(2)<<hour<<":"<<std::setw(2)<<minute<<"  "<<(status_.is_daylight()?"LIGHT":"NIGHT");
    power<<"POWER "<<status_.power_current()<<"/"<<status_.power_capacity();
    water<<"WATER "<<status_.water_current()<<"/"<<status_.water_capacity();
    resources<<"RES "<<status_.resources();
    crew<<"CREW "<<status_.crew_current()<<"/"<<status_.crew_capacity();

    const std::string values[5]={day.str(),power.str(),water.str(),resources.str(),crew.str()};
    for(int i=0;i<5;++i){
        const float x=cell*i;
        if(i>0) r.draw_line({x,8},{x,STATUS_BAR_H-8},theme_.border,1.0f);
        game_label(r,{x+9,12},values[i],0.72f,theme_,false);
    }
}

} // namespace game
