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

void GameApp::update(float dt,const InputState& in){
    input_=in;
    if(dt>0.000001f){
        const float instant=1.0f/dt;
        fps_=fps_<=0.0f?instant:(fps_*0.90f+instant*0.10f);
    }

    if(screen_==Screen::Playfield){
        if(in.escape_pressed){
            // ESC cancels the most specific active gameplay operation first.
            if(build_catalog_.selected()!=nullptr){
                build_catalog_.clear_selection();
                placement_rotation_degrees_=0.0f;
                corridor_build_start_.reset();
                utility_build_start_.reset();
                return;
            }
            if(corridor_link_source_>=0){
                corridor_link_source_=-1;
                return;
            }
            screen_=Screen::MainMenu;
            return;
        }

        InputState camera_input=in;
        if(const BuildObjectDefinition* selected=build_catalog_.selected(); selected && (selected->kind==BuildObjectKind::Habitat || selected->kind==BuildObjectKind::Resource)){
            constexpr float step=15.0f;
            if(in.wheel>0.0f) placement_rotation_degrees_=wrap_degrees(placement_rotation_degrees_+step);
            if(in.wheel<0.0f) placement_rotation_degrees_=wrap_degrees(placement_rotation_degrees_-step);
            if(in.key_n_pressed) placement_rotation_degrees_=wrap_degrees(placement_rotation_degrees_-step);
            if(in.key_m_pressed) placement_rotation_degrees_=wrap_degrees(placement_rotation_degrees_+step);
            camera_input.wheel=0.0f;
        }
        camera_.update(camera_input,playfield_.size(),viewport_height_);
        status_.update(dt);

        // World selection and manual corridor linking use the playfield ray.
        // HUD clicks are excluded so buttons never select geometry behind them.
        const float logical_y=(float)in.mouse_y/ui_scale_;
        const bool over_world=logical_y>=STATUS_BAR_H && logical_y<((float)viewport_height_/ui_scale_-BUILD_BAR_H);
        if(in.left_pressed && build_catalog_.selected()==nullptr && over_world && !mouse_over_selection_panel(in)){
            Vec3 hit{};
            if(camera_.intersect_playfield(in.mouse_x,in.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)){
                if(corridor_link_source_>=0){
                    const int target=pick_corridor(hit,1.70f);
                    if(target>=0 && target!=corridor_link_source_){
                        if(corridor_network_.connect_corridors((std::size_t)corridor_link_source_,(std::size_t)target,placed_habitats_,habitat_models_)){
                            selected_corridor_=(int)corridor_network_.connections().size()-1;
                            selected_habitat_=-1;
                            selected_resource_=-1;
                            selection_kind_=SelectionKind::Corridor;
                            corridor_link_source_=-1;
                        }
                    }
                }else{
                    select_world_at(in.mouse_x,in.mouse_y);
                }
            }
        }
    } else if(in.escape_pressed && screen_!=Screen::MainMenu) {
        screen_=Screen::MainMenu;
    }
}

} // namespace game
