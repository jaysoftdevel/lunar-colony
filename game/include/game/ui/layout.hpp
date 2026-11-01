#pragma once
#include "engine/engine.hpp"
#include "engine/ui_metrics.hpp"
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

// At equal aspect ratio the logical canvas is identical at 720p, 1080p,
// 1440p and 4K. Different aspect ratios gain usable space without stretching.
inline constexpr float UI_REFERENCE_WIDTH=1920.0f,UI_REFERENCE_HEIGHT=1080.0f;
inline float resolution_scale(int width,int height,float user_zoom){
    return std::max(0.01f,std::min(std::max(1,width)/UI_REFERENCE_WIDTH,
                                  std::max(1,height)/UI_REFERENCE_HEIGHT)*user_zoom);
}
inline eng::Vec2 pointer_to_ui(eng::Vec2 pixels,float scale){return {pixels.x/scale,pixels.y/scale};}

struct CommonResolution { int width; int height; };
constexpr CommonResolution COMMON_RESOLUTIONS[]={
    {1024,576},{1024,768},{1280,720},{1280,800},{1280,1024},{1366,768},
    {1440,900},{1600,900},{1680,1050},{1920,1080},{1920,1200},{2560,1080},
    {2560,1440},{2560,1600},{3440,1440},{3840,2160}
};

inline float text_width(const std::string& text,float scale,const eng::ui::Theme& theme){
    return eng::ui::text_extent(text,eng::ui::font_scale(scale,theme)).x;
}
inline void game_label(eng::Renderer& r,eng::Vec2 pos,const std::string& text,float scale,
                       const eng::ui::Theme& theme,bool muted=false){
    eng::ui::label(r,pos,text,scale*GAME_FONT_BOOST,theme,muted);
}
inline float game_text_width(const std::string& text,float scale,const eng::ui::Theme& theme){return text_width(text,scale*GAME_FONT_BOOST,theme);}
inline void game_label_fitted(eng::Renderer& r,eng::Rect bounds,const std::string& text,float scale,
                             const eng::ui::Theme& theme,bool muted=false){
    eng::ui::label_fitted(r,bounds,text,scale*GAME_FONT_BOOST,theme,muted);
}
} // namespace game::layout
