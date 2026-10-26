#pragma once
#include "engine/engine.hpp"
#include <algorithm>
#include <string>

namespace game::layout {
constexpr float UI_SCALES[]={1.0f,1.25f,1.5f,1.75f,2.0f};
constexpr int UI_SCALE_COUNT=(int)(sizeof(UI_SCALES)/sizeof(UI_SCALES[0]));
constexpr float SCREEN_MARGIN=22.0f;
constexpr float PANEL_PADDING=22.0f;
constexpr float CONTROL_GAP=10.0f;
constexpr float SECTION_GAP=18.0f;
constexpr float STATUS_BAR_H=46.0f;
constexpr float BUILD_BAR_H=80.0f;
constexpr float BUILD_ICON_SIZE=50.0f;
constexpr float BUILD_ICON_GAP=9.0f;
constexpr float PLAYFIELD_MARGIN=12.0f;
constexpr float GAME_FONT_BOOST=1.40f;

struct CommonResolution { int width; int height; };
constexpr CommonResolution COMMON_RESOLUTIONS[]={
    {1024,576},{1024,768},{1280,720},{1280,800},{1280,1024},{1366,768},
    {1440,900},{1600,900},{1680,1050},{1920,1080},{1920,1200},{2560,1080},
    {2560,1440},{2560,1600},{3440,1440},{3840,2160}
};

inline float text_width(const std::string& text,float scale,const eng::ui::Theme& theme){
    return (float)text.size()*6.0f*scale*theme.font_scale;
}
inline void game_label(eng::Renderer& r,eng::Vec2 pos,const std::string& text,float scale,
                       const eng::ui::Theme& theme,bool muted=false){
    eng::ui::label(r,pos,text,scale*GAME_FONT_BOOST,theme,muted);
}
} // namespace game::layout
