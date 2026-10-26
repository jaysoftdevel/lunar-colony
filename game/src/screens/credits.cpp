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

void GameApp::render_credits(Renderer& r){
    ui::title(r,{32,26},"CREDITS",theme_);
    const float w=std::min(610.0f,ui_width(r)-2.0f*SCREEN_MARGIN);
    Rect p={(ui_width(r)-w)*0.5f,100,w,220};
    ui::panel(r,p,theme_);
    const float x=p.x+PANEL_PADDING;
    ui::label(r,{x,p.y+28},"GAME PROTOTYPE",1.28f,theme_);
    ui::label(r,{x,p.y+76},"NATIVE C++20 / X11 / OPENGL",0.92f,theme_,true);
    ui::label(r,{x,p.y+108},"ENGINE AND GAME IMPLEMENTATION ARE SEPARATE",0.78f,theme_,true);
    if(ui::button(r,ui_input_,{x,p.y+p.h-PANEL_PADDING-40,150,40},"BACK",theme_)) screen_=Screen::MainMenu;
}

} // namespace game
