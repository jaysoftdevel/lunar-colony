#include "game/game_app.hpp"
#include "engine/ui_metrics.hpp"
#include <cmath>
#include <initializer_list>

namespace game {
using eng::Color;
using eng::Rect;
using eng::Renderer;
using eng::Vec2;

void GameApp::render_build_icon(Renderer& renderer,BuildIcon icon,Rect bounds,Color ink) const {
    const eng::ui::IconCanvas canvas(renderer,bounds,{24,24});
    const Color fill{ink.r,ink.g,ink.b,ink.a*.15f};
    const auto stroke=[&](std::initializer_list<Vec2> points,float width=1.65f){
        auto point=points.begin();
        for(auto next=point+1;next<points.end();++point,++next)
            canvas.draw_line(*point,*next,ink,width);
    };
    const auto box=[&](Rect q){
        canvas.draw_rect(q,fill);
        const float right=q.x+q.w,bottom=q.y+q.h;
        stroke({{q.x,q.y},{right,q.y},{right,bottom},{q.x,bottom},{q.x,q.y}});
    };
    const auto oval=[&](float x,float y,float rx,float ry,float width=1.55f){
        constexpr float tau=6.28318530718f;
        Vec2 last{x+rx,y};
        for(int i=1;i<=16;++i){
            const float angle=tau*i/16.f;
            const Vec2 next{x+rx*std::cos(angle),y+ry*std::sin(angle)};
            canvas.draw_line(last,next,ink,width);
            last=next;
        }
    };
    const auto roof=[&]{
        stroke({{2,20},{2,11},{5,7},{12,3},{19,7},{22,11},{22,20},{2,20}});
    };
    const auto drop=[&](float x,float y,float s=1.0f){
        stroke({{x,y-4*s},{x-3*s,y},{x-3*s,y+2*s},{x-1.5f*s,y+3.5f*s},
                {x+1.5f*s,y+3.5f*s},{x+3*s,y+2*s},{x+3*s,y},{x,y-4*s}});
    };
    const auto tank=[&]{
        stroke({{5,7},{6,4},{9,3},{15,3},{18,4},{19,7},{19,18},
                {17,21},{7,21},{5,18},{5,7}});
        stroke({{10,3},{10,1.5f},{14,1.5f},{14,3}});
    };
    const auto gas=[&](bool hydrogen,float x,float y){
        if(hydrogen){
            stroke({{x-2,y-3},{x-2,y+3}},1.4f);
            stroke({{x+2,y-3},{x+2,y+3}},1.4f);
            stroke({{x-2,y},{x+2,y}},1.4f);
        }else oval(x,y,2.25f,3.f,1.4f);
        stroke({{x+4,y+1},{x+5,y},{x+7,y},{x+7,y+1.3f},{x+4,y+4},{x+7.5f,y+4}},1.25f);
    };
    const auto bolt=[&](float x,float y){
        stroke({{x+1.5f,y-5},{x-2.5f,y+.5f},{x+.5f,y+.5f},{x-1.0f,y+5},{x+3.5f,y-1},{x+.5f,y-1}});
    };
    const auto drill=[&]{
        stroke({{3,19},{3,5},{15,5},{15,9}});
        box({6,3,7,5});
        stroke({{9.5f,8},{9.5f,19},{7.5f,16},{11.5f,14},{7.5f,12},{11.5f,10}});
        stroke({{1.5f,21},{14,21}},1.8f);
    };
    switch(icon){
        case BuildIcon::CategoryHabitats:
            roof(); box({9,13,6,7}); stroke({{6,10},{9,10}},1.4f); stroke({{15,10},{18,10}},1.4f);
            break;
        case BuildIcon::CategoryResources:
            bolt(5.3f,12);
            stroke({{11,8},{16.5f,5},{22,8},{22,16},{16.5f,19},{11,16},{11,8},{16.5f,11},{22,8}});
            stroke({{16.5f,11},{16.5f,19}});
            break;
        case BuildIcon::CategoryFacilities:
        case BuildIcon::Factory:
            stroke({{3,21},{3,12},{9,8},{9,12},{15,8},{15,12},{18,12},{18,3},
                    {21,3},{21,21},{3,21}});
            stroke({{6,17},{8,17}}); stroke({{11,17},{13,17}}); stroke({{16,17},{18,17}});
            break;
        case BuildIcon::AirLock:
            box({3,3,18,18}); box({8,7,8,14});
            stroke({{12,7},{12,21}}); stroke({{5.5f,11},{5.5f,13}},2.2f);
            stroke({{18.5f,11},{18.5f,13}},2.2f);
            break;
        case BuildIcon::OxygenDome:
            oval(11.5f,12,9,9);
            gas(false,9.4f,11.5f);
            stroke({{18.5f,5.5f},{21.5f,5.5f},{21.5f,2.5f}},1.4f);
            break;
        case BuildIcon::CantineDome:
            stroke({{5,3},{5,8},{7.5f,10},{10,8},{10,3}});
            stroke({{7.5f,3},{7.5f,21}});
            oval(17,7,3,4.5f); stroke({{17,11.5f},{17,21}},1.8f);
            break;
        case BuildIcon::BioDome:
            stroke({{12,21},{12,12},{14,8},{20,4}});
            stroke({{12,15},{7,15},{3,11},{3,6},{8,7},{12,11}});
            stroke({{12,13},{17,13},{21,9},{21,3},{16,4},{12,8}});
            stroke({{7,21},{17,21}});
            break;
        case BuildIcon::CrewQuarters:
            stroke({{3,6},{3,20},{3,17},{21,17},{21,20}});
            box({3,11,18,6}); box({4,8,6,3});
            stroke({{10,11},{10,17}});
            break;
        case BuildIcon::ProcessingFacility:
            box({3,7,18,14}); stroke({{6,7},{8,3},{16,3},{18,7}});
            stroke({{7,17},{9,12},{15,12},{17,17},{7,17}});
            stroke({{10,15},{14,15}},1.35f);
            break;
        case BuildIcon::Manufactory:
            stroke({{4,21},{20,21},{20,18},{4,18},{4,21}});
            stroke({{7,18},{7,11},{13,5},{19,10}},2.25f);
            oval(7,11,1.3f,1.3f); oval(13,5,1.5f,1.5f);
            stroke({{17,11},{18,14},{21,14},{22,11}});
            break;
        case BuildIcon::SickBay:
            box({3,6,18,15}); stroke({{8,6},{8,3},{16,3},{16,6}});
            canvas.draw_rect({10.8f,9,2.4f,9},ink);
            canvas.draw_rect({7.5f,12.3f,9,2.4f},ink);
            break;
        case BuildIcon::Laboratory:
            stroke({{8,3},{16,3}});
            stroke({{10,3},{10,9},{4.5f,18},{5,21},{19,21},{19.5f,18},{14,9},{14,3}});
            stroke({{7,15},{17,15}},1.45f); oval(13,17.5f,1.1f,1.1f,1.1f);
            break;
        case BuildIcon::Corridor:
            stroke({{3,19},{3,10},{5,7},{9,7},{11,10},{11,19},{3,19}});
            stroke({{5,7},{15,4},{19,4},{21,7},{21,16},{11,19}});
            stroke({{9,7},{19,4}}); stroke({{11,10},{21,7}});
            break;
        case BuildIcon::SolarPanel:
            stroke({{3,9},{16,9},{20,17},{2,17},{3,9}});
            stroke({{9,9},{9,17}},1.3f); stroke({{14,9},{15.5f,17}},1.3f);
            stroke({{2.5f,13},{18,13}},1.3f);
            stroke({{11,17},{11,21},{7,21},{16,21}});
            oval(19,5,2.1f,2.1f,1.25f);
            stroke({{19,1},{19,2}},1.1f); stroke({{22,5},{23,5}},1.1f);
            break;
        case BuildIcon::WaterDriller:
            drill(); drop(19,15);
            break;
        case BuildIcon::PowerStorage:
            box({3,6,16,12});
            canvas.draw_rect({19,9,2.5f,6},ink);
            bolt(10.5f,12);
            break;
        case BuildIcon::WaterStorage:
            tank(); drop(12,11.5f);
            stroke({{7,17},{17,17}},1.4f);
            break;
        case BuildIcon::OreDriller:
            drill(); stroke({{19,12},{22.5f,15},{20,20},{16,20},{15.5f,16},{19,12}});
            stroke({{19,12},{19,17},{16,20}},1.2f);
            break;
        case BuildIcon::WaterPurifier:
            drop(5,7,.8f); stroke({{10,6},{21,6},{17,12},{17,19},{14,21},{14,12},{10,6}});
            stroke({{12,9},{19,9}},1.4f);
            break;
        case BuildIcon::Antenna:
            stroke({{3,7},{4,12},{8,16},{13,17},{19,15},{3,7}});
            stroke({{11,12},{17,6}}); oval(17,6,1.25f,1.25f);
            stroke({{11,17},{10,21},{6,21},{16,21}});
            stroke({{18.5f,2},{21,3.5f},{22.5f,6}},1.4f);
            break;
        case BuildIcon::LandingPad:
            oval(12,12,10,9.5f);
            stroke({{8.5f,7},{8.5f,17}},2.1f); stroke({{15.5f,7},{15.5f,17}},2.1f);
            stroke({{8.5f,12},{15.5f,12}},2.1f);
            break;
        case BuildIcon::StorageDome:
            box({3,6,18,14}); stroke({{3,6},{7,3},{17,3},{21,6}});
            stroke({{3,12},{21,12}}); stroke({{12,6},{12,12}});
            stroke({{8,12},{8,20}}); stroke({{16,12},{16,20}});
            break;
        case BuildIcon::SupplyChannel:
            stroke({{3,8},{12,8},{16,12},{21,12}},1.8f);
            stroke({{3,12},{10,12},{14,16},{21,16}},1.8f);
            stroke({{3,6},{3,14}},2.3f); stroke({{21,10},{21,18}},2.3f);
            break;
        case BuildIcon::Walkway:
            stroke({{3,20},{3,13},{8,7},{21,7},{16,13},{3,13}});
            stroke({{16,13},{16,20}}); stroke({{21,7},{21,15}});
            for(float x:{6.5f,10.f,13.5f}) stroke({{x,13},{x+4.8f,7.5f}},1.1f);
            break;
        case BuildIcon::RobotWorkshop:
            roof();
            stroke({{6,15},{7.5f,12},{16.5f,12},{18,15},{18,17},{6,17},{6,15}});
            oval(8,17.5f,1.45f,1.45f,1.2f); oval(16,17.5f,1.45f,1.45f,1.2f);
            break;
        case BuildIcon::HydrogenGenerator:
            box({2,5,20,14}); gas(true,6.8f,11.5f); bolt(17.4f,12);
            break;
        case BuildIcon::HydrogenTank:
        case BuildIcon::OxygenTank:
            tank(); gas(icon==BuildIcon::HydrogenTank,10.2f,11.7f);
            break;
        case BuildIcon::ConveyorBelt:
            stroke({{5,12},{19,12},{22,14},{22,17},{19,19},{5,19},{2,17},{2,14},{5,12}});
            for(float x:{6.f,12.f,18.f}) oval(x,15.5f,1.2f,1.2f,1.3f);
            stroke({{8,12},{8,6},{15,6},{15,12}});
            stroke({{15,4},{21,4},{19,2},{21,4},{19,6}},1.4f);
            break;
    }
}

} // namespace game
