#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include <array>
#include <cmath>

namespace game {
namespace {
using namespace runtime;
enum class Need {Health,Oxygen,Food,Water,Rest};

void need_icon(SceneRenderer& r,Need need,Rect bounds,Color ink){
    const ui::IconCanvas c(r,bounds,{24,24});
    const auto line=[&](Vec2 a,Vec2 b){c.draw_line(a,b,ink,2.f);};
    switch(need){
        case Need::Health:
            c.draw_rect({9,3,6,18},ink);c.draw_rect({3,9,18,6},ink);break;
        case Need::Oxygen:
            line({12,2},{12,10});line({12,8},{7,12});line({12,8},{17,12});
            for(float sign:{-1.f,1.f}){
                const auto p=[&](float x,float y){return Vec2{12+sign*x,y};};
                line(p(2,10),p(5,6));line(p(5,6),p(9,13));line(p(9,13),p(9,19));
                line(p(9,19),p(7,22));line(p(7,22),p(2,20));line(p(2,20),p(2,10));
            }break;
        case Need::Food:
            for(float x:{3.f,6.f,9.f})line({x,3},{x,9});
            line({3,9},{9,9});line({6,9},{6,22});
            c.draw_rect({15,3,4,10},ink);line({18,12},{18,22});break;
        case Need::Water:{
            constexpr std::array<Vec2,9> drop{{{12,2},{5,12},{4,16},{6,20},{12,23},{18,20},{20,16},{19,12},{12,2}}};
            for(std::size_t i=1;i<drop.size();++i)line(drop[i-1],drop[i]);
            line({8,15},{8,18});line({8,18},{11,20});break;
        }
        case Need::Rest:
            line({3,5},{3,22});line({22,11},{22,22});line({3,17},{22,17});
            c.draw_rect({5,9,5,5},ink);c.draw_rect({11,11,10,4},ink);break;
    }
}
}

void GameApp::render_first_person_alerts(runtime::SceneRenderer& r) const {
    using namespace runtime;using namespace layout;
    if(screen_!=Screen::Playfield||first_person_mode_!=FirstPersonMode::Active||
       controlled_astronaut_<0||!astronaut_present(std::size_t(controlled_astronaut_)))return;
    const auto& actor=test_astronauts_[controlled_astronaut_];
    bool in_airlock=false;const auto* air=astronaut_air_at(actor.position,in_airlock);
    const bool low_oxygen=actor.suited?actor.suit_oxygen_kg<kSuitOxygenCapacityKg*.20:
                          !air||air->breathability()<20.f;
    struct Alert {Need need;const char* caption;bool critical;};
    const std::array alerts{
        Alert{Need::Health,"LOW HEALTH",actor.health<.20f},
        Alert{Need::Oxygen,"LOW OXYGEN",low_oxygen},
        Alert{Need::Food,"LOW FOOD",actor.hunger>.80f},
        Alert{Need::Water,"LOW WATER",actor.thirst>.80f},
        Alert{Need::Rest,"LOW REST",actor.fatigue>kCrewExhausted*.80f}
    };
    const int count=int(std::count_if(alerts.begin(),alerts.end(),[](const auto& a){return a.critical;}));
    if(!count)return;

    r.set_2d(ui_scale_);
    const float width=ui_width(r),height=ui_height(r);
    // UI time continues while simulation time is paused. One smooth flash per
    // second avoids rapid strobing; captions remain fully visible throughout.
    const float pulse=.5f+.5f*float(std::sin(ui_time_seconds_*6.283185307179586));
    const auto border=[&](float inset,float thickness,float alpha){
        const Color red{1.f,.025f,.02f,alpha};
        r.draw_rect({inset,inset,width-2*inset,thickness},red);
        r.draw_rect({inset,height-inset-thickness,width-2*inset,thickness},red);
        r.draw_rect({inset,inset+thickness,thickness,height-2*(inset+thickness)},red);
        r.draw_rect({width-inset-thickness,inset+thickness,thickness,height-2*(inset+thickness)},red);
    };
    border(0,7,.12f+.78f*pulse);border(7,9,.025f+.12f*pulse);border(16,12,.01f+.045f*pulse);

    // Lower-left notifications leave the reticle, crew controls, entrance
    // controls and bottom build/config bar available, even at maximum UI zoom.
    constexpr float margin=20,gap=6,card_h=40;
    const float available=astronaut_status_rect().x-margin-12;
    const int columns=std::min(count,available>=426.f?2:1),rows=(count+columns-1)/columns;
    const float card_w=std::min(270.f,(available-gap*(columns-1))/columns);
    const float bottom_margin=show_fps_?52.f:margin;
    const float top=height-BUILD_BAR_H-bottom_margin-rows*card_h-(rows-1)*gap;
    const Color red{1.f,.12f,.10f,1.f};
    int index=0;
    for(const auto& alert:alerts){
        if(!alert.critical)continue;
        const Rect card{margin+(index%columns)*(card_w+gap),top+(index/columns)*(card_h+gap),card_w,card_h};++index;
        r.draw_rect(card,{.075f,.012f,.018f,.94f});
        r.draw_rect({card.x,card.y,3,card.h},red);
        need_icon(r,alert.need,{card.x+9,card.y+7,26,26},red);
        const float font=ui::fit_text(alert.caption,2.2f,{card.w-49,card.h-12});
        const Vec2 position{card.x+44,card.y+(card.h-ui::text_extent(alert.caption,font).y)*.5f};
        r.draw_text(position,alert.caption,font,red);
        // The atlas font has one weight: an offset second stroke gives these
        // emergency captions a bold face without platform-specific fonts.
        r.draw_text({position.x+.45f*font,position.y},alert.caption,font,red);
    }
}
} // namespace game
