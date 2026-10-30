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

float GameApp::game_speed_multiplier() const {
    switch(game_speed_){
        case GameSpeed::Normal: return 1.0f;
        case GameSpeed::Quick: return 2.0f;
        case GameSpeed::Fast: return 4.0f;
        case GameSpeed::Pause: return 0.0f;
    }
    return 1.0f;
}

void GameApp::update(float dt,const InputState& in){
    input_=in;
    if(dt>0.000001f){
        const float instant=1.0f/dt;
        fps_=fps_<=0.0f?instant:(fps_*0.90f+instant*0.10f);
    }

    if(screen_==Screen::Playfield){
        const float sim_dt=dt*game_speed_multiplier();
        if(in.key_v_pressed){
            selection_marquee_active_=false;
            selection_marquee_dragged_=false;
            if(first_person_mode_==FirstPersonMode::Off) enter_first_person_drop_mode();
            else exit_first_person_mode();
            return;
        }

        if(first_person_mode_==FirstPersonMode::DropPending){
            status_.update(sim_dt);
            update_construction(sim_dt);
            if(in.escape_pressed){ exit_first_person_mode(); return; }
            camera_.update(in,playfield_.size(),viewport_height_);
            const float logical_y=(float)in.mouse_y/ui_scale_;
            const bool over_world=logical_y>=STATUS_BAR_H && logical_y<((float)viewport_height_/ui_scale_-BUILD_BAR_H);
            if(in.left_pressed && over_world){
                Vec3 hit{};
                if(camera_.intersect_playfield(in.mouse_x,in.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)){
                    // POV placement is exterior-only.  We still resolve the
                    // local surface height before validation so Air Lock access
                    // stairs and terrain are entered at the correct elevation;
                    // pressurised structures/corridors themselves are rejected.
                    Vec3 spawn{hit.x,first_person_floor_height(hit),hit.z};
                    if(first_person_spawn_valid(spawn)){
                        first_person_.spawn(spawn,camera_.azimuth_degrees());
                        first_person_mode_=FirstPersonMode::Active;
                        app_.set_relative_mouse_mode(true);
                    }
                }
            }
            return;
        }

        if(first_person_mode_==FirstPersonMode::Active){
            if(in.escape_pressed){ exit_first_person_mode(); return; }
            status_.update(sim_dt);
            update_construction(sim_dt);
            update_first_person(dt,in);
            return;
        }

        // RMB is a fast cancel for every active build/placement tool. Consume
        // the click before camera panning so cancelling never nudges the view.
        if(in.right_pressed && build_catalog_.selected()!=nullptr){
            selection_marquee_active_=false;
            selection_marquee_dragged_=false;
            build_catalog_.clear_selection();
            placement_rotation_degrees_=0.0f;
            placement_habitat_size_=HabitatSizeTier::Small;
            placement_resource_size_=ResourceSizeTier::Small;
            placement_storage_size_=StorageDomeSize::Small;
            corridor_build_start_.reset();
            utility_build_start_.reset();
            walkway_build_start_.reset();
            walkway_build_target_.reset();
            corridor_link_source_=-1;
            return;
        }

        if(in.escape_pressed){
            selection_marquee_active_=false;
            selection_marquee_dragged_=false;
            // ESC cancels the most specific active gameplay operation first.
            if(build_catalog_.selected()!=nullptr){
                build_catalog_.clear_selection();
                placement_rotation_degrees_=0.0f;
                placement_habitat_size_=HabitatSizeTier::Small;
                placement_resource_size_=ResourceSizeTier::Small;
                placement_storage_size_=StorageDomeSize::Small;
                corridor_build_start_.reset();
                utility_build_start_.reset();
                walkway_build_start_.reset();
                walkway_build_target_.reset();
                return;
            }
            if(corridor_link_source_>=0){
                corridor_link_source_=-1;
                return;
            }
            screen_=Screen::MainMenu;
            return;
        }

        if(in.delete_pressed && build_catalog_.selected()==nullptr && corridor_link_source_<0 && !corridor_build_start_ && !utility_build_start_ && !walkway_build_start_){
            delete_selected_world_object();
            return;
        }

        InputState camera_input=in;
        if(const BuildObjectDefinition* selected=build_catalog_.selected(); selected && (selected->kind==BuildObjectKind::Habitat || selected->kind==BuildObjectKind::Resource)){
            constexpr float step=15.0f;
            if(selected->kind==BuildObjectKind::Habitat){
                auto cycle_habitat=[&](int delta){
                    if(habitat_has_extended_size_tiers(selected->habitat_kind)){
                        constexpr HabitatSizeTier order[]{HabitatSizeTier::Small,HabitatSizeTier::Standard,HabitatSizeTier::Mid,HabitatSizeTier::Big};
                        int idx=0; for(int i=0;i<4;++i) if(order[i]==placement_habitat_size_) idx=i;
                        idx=(idx+delta+4)%4; placement_habitat_size_=order[idx];
                    }else{
                        constexpr HabitatSizeTier order[]{HabitatSizeTier::Small,HabitatSizeTier::Standard};
                        int idx=placement_habitat_size_==HabitatSizeTier::Small?0:1;
                        idx=(idx+delta+2)%2; placement_habitat_size_=order[idx];
                    }
                };
                if(in.wheel>0.0f) cycle_habitat(+1);
                if(in.wheel<0.0f) cycle_habitat(-1);
                camera_input.wheel=0.0f;
            }else if(selected->resource_kind==ResourceKind::StorageDome){
                auto cycle_storage=[&](int delta){
                    int v=(int)placement_storage_size_;
                    v=(v+delta+4)%4;
                    placement_storage_size_=(StorageDomeSize)v;
                };
                if(in.wheel>0.0f) cycle_storage(+1);
                if(in.wheel<0.0f) cycle_storage(-1);
                camera_input.wheel=0.0f;
            }else{
                auto cycle_resource=[&](int){
                    placement_resource_size_=placement_resource_size_==ResourceSizeTier::Small?ResourceSizeTier::Standard:ResourceSizeTier::Small;
                };
                if(in.wheel!=0.0f) cycle_resource(in.wheel>0.0f?1:-1);
                camera_input.wheel=0.0f;
            }
            // Mouse wheel is exclusively a structure-size control while a
            // structure is being placed. Rotation is keyboard-only (N/M).
            if(in.key_n_pressed) placement_rotation_degrees_=wrap_degrees(placement_rotation_degrees_-step);
            if(in.key_m_pressed) placement_rotation_degrees_=wrap_degrees(placement_rotation_degrees_+step);
        }
        camera_.update(camera_input,playfield_.size(),viewport_height_);
        status_.update(sim_dt);
        update_construction(sim_dt);

        // World selection uses a click-or-drag marquee whenever no placement
        // tool is active. A short click keeps the old single-object selection;
        // dragging draws a screen-space pane and selects every ground resource
        // package whose projected center lies inside it.
        const float logical_y=(float)in.mouse_y/ui_scale_;
        const bool over_world=logical_y>=STATUS_BAR_H && logical_y<((float)viewport_height_/ui_scale_-BUILD_BAR_H);
        const bool selection_idle=build_catalog_.selected()==nullptr && corridor_link_source_<0 &&
                                  !corridor_build_start_ && !utility_build_start_ && !walkway_build_start_;
        if(selection_idle && in.left_pressed && over_world && !mouse_over_selection_panel(in)){
            Vec3 hit{};
            if(camera_.intersect_playfield(in.mouse_x,in.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)){
                selection_marquee_active_=true;
                selection_marquee_dragged_=false;
                selection_marquee_start_x_=in.mouse_x;
                selection_marquee_start_y_=in.mouse_y;
            }
        }
        if(selection_marquee_active_){
            const int dx=in.mouse_x-selection_marquee_start_x_;
            const int dy=in.mouse_y-selection_marquee_start_y_;
            if(std::sqrt((float)(dx*dx+dy*dy))>=6.0f*ui_scale_) selection_marquee_dragged_=true;
            if(!in.left_down) finish_selection_marquee();
        }
    } else if(in.escape_pressed && screen_!=Screen::MainMenu) {
        screen_=Screen::MainMenu;
    }
}

} // namespace game
