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

void GameApp::render_build_icon(Renderer& r,BuildIcon icon,Rect q,Color c) const {
    const float x=q.x,y=q.y,w=q.w,h=q.h;
    const float cx=x+w*0.5f;
    const float cy=y+h*0.5f;
    switch(icon){
        case BuildIcon::CategoryHabitats:
            r.draw_line({x+5,y+h-8},{x+w-5,y+h-8},c,2.0f);
            r.draw_line({x+7,y+h-9},{x+12,y+17},c,2.0f);
            r.draw_line({x+12,y+17},{cx,y+7},c,2.0f);
            r.draw_line({cx,y+7},{x+w-12,y+17},c,2.0f);
            r.draw_line({x+w-12,y+17},{x+w-7,y+h-9},c,2.0f);
            r.draw_rect({cx-4,cy+5,8,10},{c.r,c.g,c.b,0.16f});
            break;
        case BuildIcon::CategoryResources:
            r.draw_line({cx,y+5},{cx,y+h-5},c,2.2f);
            r.draw_line({x+6,cy},{x+w-6,cy},c,2.2f);
            r.draw_rect({cx-7,cy-7,14,14},{c.r,c.g,c.b,0.10f});
            r.draw_line({x+9,y+10},{x+w-9,y+h-10},c,1.6f);
            r.draw_line({x+w-9,y+10},{x+9,y+h-10},c,1.6f);
            break;
        case BuildIcon::AirLock:
            r.draw_rect({x+8,y+12,w-16,h-20},{c.r,c.g,c.b,0.10f});
            r.draw_line({x+8,y+12},{x+w-8,y+12},c,2.0f);
            r.draw_line({x+8,y+h-8},{x+w-8,y+h-8},c,2.0f);
            r.draw_line({x+8,y+12},{x+8,y+h-8},c,2.0f);
            r.draw_line({x+w-8,y+12},{x+w-8,y+h-8},c,2.0f);
            r.draw_rect({cx-7,cy-7,14,14},{c.r,c.g,c.b,0.16f});
            r.draw_line({cx-8,cy},{cx+8,cy},c,2.0f);
            r.draw_line({cx,cy-8},{cx,cy+8},c,2.0f);
            break;
        case BuildIcon::OxygenDome:
            r.draw_line({x+5,y+h-9},{x+w-5,y+h-9},c,2.0f);
            r.draw_line({x+7,y+h-10},{x+11,y+17},c,2.0f);
            r.draw_line({x+11,y+17},{cx,y+6},c,2.0f);
            r.draw_line({cx,y+6},{x+w-11,y+17},c,2.0f);
            r.draw_line({x+w-11,y+17},{x+w-7,y+h-10},c,2.0f);
            r.draw_rect({x+5,y+h-22,8,12},{c.r,c.g,c.b,0.18f});
            r.draw_rect({x+15,y+h-25,6,15},{c.r,c.g,c.b,0.18f});
            r.draw_line({cx,y+6},{cx,y+h-9},c,1.0f);
            break;
        case BuildIcon::CantineDome:
            r.draw_line({x+5,y+h-8},{x+w-5,y+h-8},c,2.0f);
            r.draw_line({x+6,y+h-9},{x+12,y+16},c,2.0f);
            r.draw_line({x+12,y+16},{cx,y+5},c,2.0f);
            r.draw_line({cx,y+5},{x+w-12,y+16},c,2.0f);
            r.draw_line({x+w-12,y+16},{x+w-6,y+h-9},c,2.0f);
            r.draw_line({x+13,y+18},{x+w-13,y+18},c,1.0f);
            r.draw_line({x+9,y+27},{x+w-9,y+27},c,1.0f);
            r.draw_line({cx-9,cy+7},{cx+9,cy+7},c,2.0f);
            break;
        case BuildIcon::BioDome:
            r.draw_line({x+4,y+h-8},{x+w-4,y+h-8},c,2.0f);
            r.draw_line({x+6,y+h-9},{x+10,y+15},c,2.0f);
            r.draw_line({x+10,y+15},{cx,y+6},c,2.0f);
            r.draw_line({cx,y+6},{x+w-10,y+15},c,2.0f);
            r.draw_line({x+w-10,y+15},{x+w-6,y+h-9},c,2.0f);
            for(int i=0;i<3;++i){
                const float px=x+14.0f+i*10.0f;
                r.draw_line({px,y+h-9},{px,y+20},c,1.5f);
                r.draw_line({px,y+26},{px-5,y+22},c,1.5f);
                r.draw_line({px,y+30},{px+5,y+26},c,1.5f);
            }
            break;
        case BuildIcon::CrewQuarters:
            r.draw_rect({x+5,y+14,w-10,h-23},{c.r,c.g,c.b,0.10f});
            r.draw_line({x+5,y+14},{x+w-5,y+14},c,2.0f);
            r.draw_line({x+5,y+h-9},{x+w-5,y+h-9},c,2.0f);
            r.draw_line({x+5,y+14},{x+5,y+h-9},c,2.0f);
            r.draw_line({x+w-5,y+14},{x+w-5,y+h-9},c,2.0f);
            for(int i=0;i<3;++i){
                const float px=x+12.0f+i*11.0f;
                r.draw_rect({px,cy-4,7,7},{c.r,c.g,c.b,0.20f});
            }
            r.draw_line({x+13,y+12},{x+13,y+h-7},c,1.0f);
            r.draw_line({x+w-13,y+12},{x+w-13,y+h-7},c,1.0f);
            break;
        case BuildIcon::Corridor:
            r.draw_line({x+6,cy+8},{x+14,cy+8},c,2.0f);
            r.draw_line({x+14,cy+8},{x+20,cy-5},c,2.0f);
            r.draw_line({x+20,cy-5},{x+w-7,cy-5},c,2.0f);
            r.draw_line({x+6,cy+14},{x+14,cy+14},c,2.0f);
            r.draw_line({x+14,cy+14},{x+22,cy+1},c,2.0f);
            r.draw_line({x+22,cy+1},{x+w-7,cy+1},c,2.0f);
            r.draw_line({x+6,cy+8},{x+6,cy+14},c,2.0f);
            r.draw_line({x+w-7,cy-5},{x+w-7,cy+1},c,2.0f);
            break;
        case BuildIcon::SolarPanel:
            r.draw_line({x+5,y+12},{x+w-5,y+19},c,2.0f);
            r.draw_line({x+5,y+22},{x+w-5,y+29},c,2.0f);
            r.draw_line({x+5,y+12},{x+5,y+22},c,1.5f);
            r.draw_line({x+w-5,y+19},{x+w-5,y+29},c,1.5f);
            r.draw_line({cx,cy+3},{cx,y+h-7},c,2.0f);
            r.draw_line({cx,cy+7},{x+10,y+h-7},c,1.5f);
            r.draw_line({cx,cy+7},{x+w-10,y+h-7},c,1.5f);
            break;
        case BuildIcon::WaterDriller:
            r.draw_line({cx-8,y+6},{cx-8,y+h-8},c,2.0f);
            r.draw_line({cx+8,y+6},{cx+8,y+h-8},c,2.0f);
            r.draw_line({cx-8,y+6},{cx+8,y+6},c,2.0f);
            r.draw_line({cx, y+10},{cx,y+h-8},c,2.0f);
            for(int i=0;i<4;++i){
                const float yy=y+20.0f+i*5.0f;
                r.draw_line({cx-5,yy},{cx+5,yy+3},c,1.2f);
            }
            r.draw_line({x+7,y+h-8},{x+w-7,y+h-8},c,2.0f);
            break;
        case BuildIcon::PowerStorage:
            for(int i=0;i<3;++i){
                const float px=x+6.0f+i*11.0f;
                r.draw_rect({px,y+10,8,h-19},{c.r,c.g,c.b,0.10f});
                r.draw_line({px,y+10},{px+8,y+10},c,1.5f);
                r.draw_line({px,y+h-9},{px+8,y+h-9},c,1.5f);
                r.draw_line({px,y+10},{px,y+h-9},c,1.5f);
                r.draw_line({px+8,y+10},{px+8,y+h-9},c,1.5f);
            }
            r.draw_line({x+4,y+h-6},{x+w-4,y+h-6},c,2.0f);
            break;
        case BuildIcon::WaterStorage:
            for(int i=0;i<2;++i){
                const float px=x+12.0f+i*15.0f;
                r.draw_line({px-5,y+15},{px-5,y+h-9},c,1.5f);
                r.draw_line({px+5,y+15},{px+5,y+h-9},c,1.5f);
                r.draw_line({px-5,y+15},{px,y+9},c,1.5f);
                r.draw_line({px,y+9},{px+5,y+15},c,1.5f);
                r.draw_line({px-5,y+h-9},{px+5,y+h-9},c,1.5f);
            }
            r.draw_line({x+5,y+h-6},{x+w-5,y+h-6},c,2.0f);
            break;
        case BuildIcon::PowerCable:
            r.draw_line({x+5,cy-7},{x+w-5,cy-7},c,2.0f);
            r.draw_line({x+5,cy},{x+w-5,cy},c,2.0f);
            r.draw_line({x+5,cy+7},{x+w-5,cy+7},c,2.0f);
            r.draw_rect({x+4,cy-12,6,24},{c.r,c.g,c.b,0.10f});
            r.draw_rect({x+w-10,cy-12,6,24},{c.r,c.g,c.b,0.10f});
            break;
        case BuildIcon::WaterPipe:
            r.draw_line({x+5,cy},{x+w-5,cy},c,3.0f);
            r.draw_line({x+9,cy-7},{x+9,cy+7},c,2.0f);
            r.draw_line({x+w-9,cy-7},{x+w-9,cy+7},c,2.0f);
            r.draw_line({cx-5,cy-5},{cx+5,cy-5},c,1.5f);
            r.draw_line({cx-5,cy+5},{cx+5,cy+5},c,1.5f);
            break;
    }
}

} // namespace game
