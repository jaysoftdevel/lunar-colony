#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/world/world_labels.hpp"
#include "game/world/lunar_look.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace game {
using namespace game::runtime;
using namespace layout;
using namespace world_math;
using namespace world_labels;

void GameApp::render_playfield(SceneRenderer& r){
    const float aspect=(float)r.width()/std::max(1,r.height());
    const bool fpv=first_person_mode_==FirstPersonMode::Active;
    const Mat4 view=fpv?first_person_.view_projection(aspect):
        lander_.in_flight()?camera_.campus_view_projection(aspect):camera_.view_projection(aspect);
    r.set_3d(view);
    const bool meshed=mesh_layout_active();
    const auto sun=status_.sun_pose();
    bool lunar=false;
    std::vector<lunar_look::Fixture> corridor_fixtures;
    // The lunar world is fundamental scenery, including on the Classic path.
    // Quality settings control its detail, not whether sky/terrain exist.
    if(!meshed){
        const Vec3 eye=fpv?first_person_.eye_position():camera_.eye_position();
        Vec3 focus=fpv?eye+first_person_.look_direction()*12.0f:camera_.target();
        focus.y=0;
        LunarScene scene{view,eye,focus,sun.direction,sun.irradiance_fraction,playfield_.size()*0.5f};
        scene.seconds=static_cast<float>(ui_time_seconds_);
        scene.first_person=fpv;
        for(const auto& corridor:corridor_network_.connections())
            lunar_look::append_corridor_fixtures(corridor_fixtures,corridor);
        if(graphics_.enabled(GraphicsFeature::LocalLights)){
            std::vector<LunarLight> lights;
            if(lander_.active()&&lander_.phase>=LanderPhase::OpeningRamp){
                for(float z:{-1.4f,1.4f})lights.push_back({lander_world(lander_,{0,4.25f,z}),{.84f,.91f,1.f,1},5.5f,45.f});
            }
            for(const auto& fixture:corridor_fixtures)lights.push_back(fixture.light());
            for(std::size_t i=0;i<placed_habitats_.size();++i){const auto& habitat=placed_habitats_[i];
                if(!habitat.construction.complete())continue;
                const auto& mesh=habitat_models_.mesh(habitat);
                if(lunar_look::powered(habitat.supply,habitat.power_enabled))lights.push_back(lunar_look::habitat_fixture(habitat,mesh).light());
                const float level=interior_light_level(i);
                if(level>0)lights.push_back({habitat.position+Vec3{0,mesh.walkable_floor_y+2.05f,0},{.80f,.89f,1.f,1},9.f,25.f*level});
            }
            for(const auto& pad:placed_resources_){
                if(pad.kind!=ResourceKind::LandingPad || !pad.construction.complete() ||
                   !lunar_look::powered(pad.supply,pad.power_enabled)) continue;
                for(const auto& fixture:lunar_look::pad_fixtures(pad,resource_models_.mesh(pad)))
                    lights.push_back(fixture.light());
            }
            for(std::size_t i=0;i<placed_resources_.size();++i){
                const auto& home=placed_resources_[i];
                const float level=interior_light_level(i,true);if(level<=0)continue;
                if(home.kind==ResourceKind::RobotWorkshop){
                    for(const auto& fixture:lunar_look::workshop_fixtures(home)){auto light=fixture.light();light.power*=level;lights.push_back(light);}
                }else if(home.kind==ResourceKind::StorageDome){
                    const auto& mesh=resource_models_.mesh(home);
                    lights.push_back({home.position+Vec3{0,mesh.walkable_floor_y+2.4f,0},{.80f,.89f,1.f,1},mesh.footprint_x*.7f,25.f*level});
                }
            }
            scene.light_count=std::min(lights.size(),scene.lights.size());
            const auto distance_to_focus=[&](const LunarLight& light){
                const Vec3 offset=light.position-focus;
                return dot(offset,offset);
            };
            std::partial_sort(lights.begin(),lights.begin()+scene.light_count,lights.end(),
                [&](const LunarLight& a,const LunarLight& b){ return distance_to_focus(a)<distance_to_focus(b); });
            std::copy_n(lights.begin(),scene.light_count,scene.lights.begin());
        }
        // The shared renderer currently exposes point lights, not spotlights.
        // Keep the point-light origin at the visor rather than offsetting it through nearby walls.
        if(fpv && controlled_astronaut_>=0 && astronaut_views_[controlled_astronaut_].suit_light &&
           test_astronauts_[controlled_astronaut_].health>0){
            const auto slot=std::min(scene.light_count,scene.lights.size()-1);
            scene.lights[slot]={eye,{.86f,.93f,1.f,1},12.f,44.f};
            scene.light_count=std::min(scene.light_count+1,scene.lights.size());
        }
        lunar=r.begin_lunar_scene(scene);
    }
    if(!meshed){
        // Stylized game-day orbit: Earth remains above the lunar horizon and
        // its longitude/position advance with the same clock as solar power.
        const float angle=(status_.hour()-6.f)*6.2831853f/24.f;
        const float elevation=.62f+.20f*std::sin(angle);
        const Vec3 direction{std::cos(angle)*std::cos(elevation),std::sin(elevation),std::sin(angle)*std::cos(elevation)};
        const Vec3 eye=fpv?first_person_.eye_position():camera_.eye_position();
        const float distance=lander_.in_flight()?220.f:650.f, radius=distance*.039f;
        const Vec3 center=eye+direction*distance;
        constexpr int rings=40, sectors=80;
        constexpr float pi=3.14159265f;
        const auto longitude=[](int i){return -pi+2*pi*i/sectors;};
        const auto latitude=[](int j){return -pi*.5f+pi*j/rings;};
        // Coastlines, clouds and ice are fixed in globe coordinates.
        static const auto terrain_type=[&]{
            struct Coast{float lon,lat,w,h;};
            constexpr Coast land[]{{-105,49,39,26},{-86,22,16,22},{-61,-15,23,36},{-44,72,15,11},{19,6,26,34},{48,48,52,23},{94,48,43,25},{80,19,16,19},{117,7,22,11},{136,-25,22,15},{-5,52,15,15}};
            std::array<unsigned char,rings*sectors> types{};
            for(int j=0;j<rings;++j)for(int i=0;i<sectors;++i){
                const float u=(longitude(i)+longitude(i+1))*.5f,v=(latitude(j)+latitude(j+1))*.5f;
                const float lon=u*180/pi,lat=v*180/pi;
                const float noise=.12f*std::sin(u*21+v*14)+.09f*std::cos(u*33-v*23);
                float coast=10;
                for(const auto& l:land){
                    const float x=(lon-l.lon)/l.w,y=(lat-l.lat)/l.h;
                    coast=std::min(coast,x*x+y*y);
                }
                int type=coast+noise<1?1:0;
                if(std::abs(lat)>72+5*std::sin(5*u))type=2;
                if(std::sin(u*13+std::sin(v*9)*2)+std::cos(v*19-u*4)>1.52f)type=3;
                types[j*sectors+i]=static_cast<unsigned char>(type);
            }
            return types;
        }();
        std::array<Vec2,sectors+1> meridians;
        std::array<Vec2,sectors> midpoints;
        for(int i=0;i<=sectors;++i){
            const float u=longitude(i)+angle*.25f;
            meridians[i]={std::cos(u),std::sin(u)};
            if(i<sectors){
                const float mid=(longitude(i)+longitude(i+1))*.5f+angle*.25f;
                midpoints[i]={std::cos(mid),std::sin(mid)};
            }
        }
        // Share edge positions across adjoining faces, including both seam endpoints.
        const auto make_ring=[&](int j){
            std::array<Vec3,sectors+1> row;
            const float v=latitude(j),cv=std::cos(v),sv=std::sin(v);
            for(int i=0;i<=sectors;++i)
                row[i]=center+Vec3{cv*meridians[i].x,sv,cv*meridians[i].y}*radius;
            return row;
        };
        static thread_local std::array<std::vector<Vec3>,20> batches;
        for(auto& batch:batches)batch.clear();
        auto lower=make_ring(0);
        for(int j=0;j<rings;++j){
            const auto upper=make_ring(j+1);
            const float v=(latitude(j)+latitude(j+1))*.5f,cv=std::cos(v),sv=std::sin(v);
            for(int i=0;i<sectors;++i){
                const Vec3 normal{cv*midpoints[i].x,sv,cv*midpoints[i].y};
                const int shade=std::clamp(int((.20f+.80f*std::max(0.f,dot(normal,sun.direction)))*5),0,4);
                auto& dst=batches[terrain_type[j*sectors+i]*5+shade];
                dst.insert(dst.end(),{lower[i],upper[i],upper[i+1],lower[i],upper[i+1],lower[i+1]});
            }
            lower=upper;
        }
        constexpr Color colors[]{{.045f,.16f,.43f,1},{.22f,.35f,.16f,1},{.70f,.78f,.82f,1},{.47f,.56f,.67f,1}};
        for(int i=0;i<20;++i){auto c=colors[i/5];const float shade=.12f+.22f*(i%5);c.r*=shade;c.g*=shade;c.b*=shade;r.draw_lunar_surface(batches[i],LunarMaterial::Celestial,c);}
    }
    const bool show_build_guides=meshed || build_catalog_.selected() || storage_gate_placement_active_;
    if(!meshed && !lunar){
        if(lander_.in_flight()) r.draw_lunar_surface({{-900,-0.035f,-900},{900,-0.035f,-900},{900,-0.035f,900},
            {-900,-0.035f,-900},{900,-0.035f,900},{-900,-0.035f,900}},LunarMaterial::Regolith,{0.055f,0.075f,0.088f,1});
        playfield_.draw_surface(r,sun.irradiance_fraction);
    }
    if(show_build_guides)
        r.draw_lines_3d(playfield_.grid_lines(),meshed?Color{0.22f,0.43f,0.47f,1}:Color{0.35f,0.64f,0.71f,0.36f},1.0f);
    // Mark the inner edge of the permanent rover perimeter lane. This is a
    // placement reserve only; robots remain free to use the entire band.
    const float lane_half=(float)playfield_.size()*0.5f-geometry::kPlayfieldRobotPerimeterLane;
    const std::vector<Vec3> perimeter_lane{
        {-lane_half,0.018f,-lane_half},{ lane_half,0.018f,-lane_half},
        { lane_half,0.018f,-lane_half},{ lane_half,0.018f, lane_half},
        { lane_half,0.018f, lane_half},{-lane_half,0.018f, lane_half},
        {-lane_half,0.018f, lane_half},{-lane_half,0.018f,-lane_half}};
    if(show_build_guides)
        r.draw_lines_3d(perimeter_lane,{0.72f,0.52f,0.18f,0.72f},1.35f);
    if(first_person_mode_!=FirstPersonMode::Active)render_ore_deposits(r);
    render_ground_resources(r);
    render_corridors(r);
    if(lunar)for(const auto& fixture:corridor_fixtures)lunar_look::draw_fixture(r,fixture,0.f);
    render_utilities(r);
    render_walkways(r);
    render_habitats(r);
    render_resources(r);
    if(show_build_guides)render_build_footprints(r);
    render_lunar_lander(r);
    render_robot_blockers(r);
    render_stash(r);
    render_trader(r);
    render_builder_robot(r);
    render_gate_areas(r);
    render_gate_connection_frames(r);
    if(astronauts_active())render_test_astronauts(r);

    if(first_person_mode_==FirstPersonMode::Active){
        placement_tip_title_.clear();placement_tip_detail_.clear();placement_tip_cost_={};
        if(!first_person_pointer_over_hud(input_)){
            render_corridor_link_preview(r);render_corridor_placement(r);
            render_habitat_placement(r);render_resource_placement(r);render_utility_placement(r);render_walkway_placement(r);
        }
        app_.set_world_cursor(false);app_.set_cursor_visible(first_person_cursor_free());
        r.set_2d(ui_scale_);
        render_construction_status_frames(r);
        render_supply_indicators(r);
        render_status_bar(r);
        render_progress_tree(r);
        render_build_bar(r);
        render_first_person_hud(r);
        render_astronaut_status(r,view);
        render_placement_tooltip(r);
        return;
    }

    placement_tip_title_.clear();
    placement_tip_detail_.clear();
    placement_tip_cost_={};
    const bool over_overlay=mouse_over_playfield_overlay(input_);
    if(!over_overlay){
        render_interior_unit_preview(r);
        render_corridor_link_preview(r);
        render_corridor_placement(r);
        render_habitat_placement(r);
        render_resource_placement(r);
        render_utility_placement(r);
        render_walkway_placement(r);
        if(game_session_active_ && input_.left_pressed && !placement_tip_detail_.empty())
            activity_log_.record(ui_time_seconds_,LogEvent::Incident,
                "Placement blocked / "+placement_tip_title_+" / "+placement_tip_detail_);
    }
    const auto queue=first_person_mode_==FirstPersonMode::Off?prepare_build_queue_view():BuildQueueView{};
    if(queue.hovered_row>=0) render_build_queue_highlight(r,queue.jobs[static_cast<std::size_t>(queue.hovered_row)].target);
    render_playfield_crosshair(r);

    const InputState hud_input=ui_input_;
    if(over_overlay){ui_input_.left_pressed=false;ui_input_.wheel=0.f;}
    r.set_2d(ui_scale_);
    render_construction_status_frames(r);
    render_supply_indicators(r);
    render_status_bar(r);
    render_build_bar(r);
    render_ore_survey(r);
    render_selection_marquee(r);
    if(selection_kind_!=SelectionKind::None) render_selection_panel(r);
    if(astronauts_active())render_astronaut_status(r,camera_.view_projection(aspect));

    if(selection_kind_==SelectionKind::None){
    const BuildObjectDefinition* active_build=build_catalog_.selected();
    const char* controls=lander_.early_build_placement()&&!active_build?"ROBOTS UNLOADING / QUEUE BUILDINGS NOW / CONNECTIONS AFTER DEPLOYMENT":storage_gate_placement_active_?"STORAGE GATE / MOVE AROUND DOME / LMB PLACE EXTENSION / ESC CANCEL":
        corridor_link_source_>=0?"LMB TARGET CORRIDOR / ESC CANCEL":
        (active_build && active_build->kind==BuildObjectKind::Corridor?
            (corridor_build_start_?"LMB SET CORRIDOR END / ESC CANCEL":"LMB SET CORRIDOR START / ESC CANCEL"):
        (active_build && active_build->kind==BuildObjectKind::Utility?
            (active_build->utility_kind==UtilityKind::ConveyorBelt?
                (utility_build_start_?
                    (placed_resources_[utility_build_start_->index].kind==ResourceKind::StorageDome?
                        "LMB SELECT LANDING PAD / ESC CANCEL":"LMB SELECT PROCESSOR, FACTORY OR STORAGE DOME / ROVERS COLLECT DRILLER ORE / ESC CANCEL"):
                    "LMB SELECT ORE DRILLER OR STORAGE DOME / ESC CANCEL"):
                (utility_build_start_?"MOVE TARGET TO ADJUST PORT / LMB CONNECT / ESC CANCEL":"LMB SELECT SUPPLY SOURCE / ESC CANCEL")):
        (active_build && active_build->kind==BuildObjectKind::Walkway?
            (walkway_build_start_?"MOVE TO ADJUST WALKWAY BLUEPRINT / LMB BUILD / ESC CANCEL":"LMB SET WALKWAY START / ESC CANCEL"):
        (active_build && (active_build->kind==BuildObjectKind::Habitat || active_build->kind==BuildObjectKind::Resource)?
            "LMB PLACE / SHIFT REPEAT / WHEEL SIZE / N-M ROTATE / ESC CANCEL":
        (active_build?"LMB PLACE / SHIFT REPEAT / N-M ROTATE / ESC CANCEL":astronaut_test_active()?
            "LMB SELECT CREW / LMB SET WAYPOINT / RMB DESELECT / RMB DRAG PAN / WHEEL ZOOM / MMB ORBIT":
            "LMB SELECT / DRAG SELECT RESOURCE BOXES / RMB DESELECT / RMB DRAG PAN / WHEEL ZOOM / MMB ORBIT / V FPV / ESC MENU")))));
    const auto control_lines=ui::wrap_text(controls,406,ui::font_scale(0.52f*GAME_FONT_BOOST,theme_));
    const bool crew_test=astronaut_test_active();
    Rect info={PLAYFIELD_MARGIN,STATUS_BAR_H+PLAYFIELD_MARGIN,430,
               34.0f+13.0f*(float)control_lines.size()+(crew_test?17.f:0.f)};
    r.draw_rect(info,kHudFill);
    game_label(r,{info.x+12,info.y+8},astronaut_test_active()?"STAGE 3 - TEST ASTRONAUTS":"PLAYFIELD "+std::to_string(playfield_.size())+" X "+std::to_string(playfield_.size()),0.78f,theme_);
    for(std::size_t i=0;i<control_lines.size();++i)game_label(r,{info.x+12,info.y+28+13*(float)i},control_lines[i],0.52f,theme_,true);

    if(crew_test){
        std::ostringstream stock;
        stock<<std::fixed<<std::setprecision(1)<<"SPAWN CACHE  O2 "<<astronaut_cache_.oxygen_kg<<" KG / FOOD "
             <<astronaut_cache_.food_kg<<" KG / WATER "<<astronaut_cache_.water_litres<<" L";
        game_label_fitted(r,{info.x+12,info.y+info.h-21,406,15},stock.str(),.46f,theme_,true);
    }else{
    Rect robot_info={info.x,info.y+info.h+6.0f,504,31};
    r.draw_rect(robot_info,kHudFill);
    std::size_t working=0,constructors=0,carriers=0;
    std::int64_t fleet_cargo=0;
    std::int64_t fleet_capacity=0;
    for(const auto& robot:builder_robots_){
        if(robot.role==RobotRole::Carrier) ++carriers; else ++constructors;
        if(robot.mode!=BuilderRobotMode::Idle) ++working;
        fleet_cargo+=robot.cargo.total_amount();
        fleet_capacity+=robot.payload_capacity_kg;
    }
    std::ostringstream robot_line;
    robot_line<<"ROBOTS  BR-02 "<<constructors<<"  CR-01 "<<carriers<<"  ACTIVE "<<working
              <<"  CARGO "<<fleet_cargo<<"/"<<fleet_capacity<<" kg";
    for(const auto& robot:builder_robots_){
        if(robot.action!=BuilderRobotAction::None){
            robot_line<<"  |  "<<robot_role_name(robot.role)<<" "<<builder_robot_action_name(robot.action);
            break;
        }
    }
    game_label_fitted(r,{robot_info.x+12,robot_info.y+7,robot_info.w-24,16},robot_line.str(),0.47f,theme_);
    }
    }
    ui_input_=hud_input;
    render_progress_tree(r);
    if(first_person_mode_==FirstPersonMode::Off) render_build_queue(r,queue);
    if constexpr(build::debug_features){
        if(debug_frame_visible())render_debug_frame(r);
    }
    if(activity_log_visible())render_activity_log(r);
    render_placement_tooltip(r);
}

void GameApp::render_selection_marquee(SceneRenderer& r){
    if(!selection_marquee_active_ || !selection_marquee_dragged_ || build_catalog_.selected()!=nullptr || corridor_link_source_>=0) return;
    const float x0=(float)selection_marquee_start_x_/ui_scale_;
    const float y0=(float)selection_marquee_start_y_/ui_scale_;
    const float x1=(float)input_.mouse_x/ui_scale_;
    const float y1=(float)input_.mouse_y/ui_scale_;
    Rect box{std::min(x0,x1),std::min(y0,y1),std::abs(x1-x0),std::abs(y1-y0)};
    const float top=STATUS_BAR_H;
    const float bottom=ui_height(r)-BUILD_BAR_H;
    if(box.y<top){ box.h-=top-box.y; box.y=top; }
    if(box.y+box.h>bottom) box.h=std::max(0.0f,bottom-box.y);
    if(box.w<=0.0f || box.h<=0.0f) return;
    r.draw_rect(box,{0.18f,0.58f,0.82f,0.12f});
    const Color c{0.32f,0.82f,1.00f,0.95f};
    r.draw_line({box.x,box.y},{box.x+box.w,box.y},c,1.6f);
    r.draw_line({box.x+box.w,box.y},{box.x+box.w,box.y+box.h},c,1.6f);
    r.draw_line({box.x+box.w,box.y+box.h},{box.x,box.y+box.h},c,1.6f);
    r.draw_line({box.x,box.y+box.h},{box.x,box.y},c,1.6f);
}

void GameApp::render_first_person_hud(SceneRenderer& r){
    if(first_person_mode_==FirstPersonMode::Active){
        const float cx=first_person_cursor_free()?float(input_.mouse_x)/ui_scale_:ui_width(r)*.5f,cy=first_person_cursor_free()?float(input_.mouse_y)/ui_scale_:ui_height(r)*.5f;
        if(!first_person_cursor_free()){
            const Color cross{.18f,1.f,.48f,1.f};
            for(const auto& line:std::array<std::pair<Vec2,Vec2>,4>{{{{cx-15,cy},{cx-4,cy}},{{cx+4,cy},{cx+15,cy}},{{cx,cy-15},{cx,cy-4}},{{cx,cy+4},{cx,cy+15}}}}){
                r.draw_line(line.first,line.second,{.015f,.02f,.025f,1},6.f);r.draw_line(line.first,line.second,cross,3.f);}
            r.draw_rect({cx-1.5f,cy-1.5f,3,3},{1,.90f,.16f,1});

        }
        const float help_width=controlled_astronaut_>=0?std::min(528.f,astronaut_status_rect().x-2*PLAYFIELD_MARGIN):528.f;
        Rect help{PLAYFIELD_MARGIN,STATUS_BAR_H+PLAYFIELD_MARGIN,help_width,49.0f};
        r.draw_rect(help,kHudFill);
        game_label(r,{help.x+12.0f,help.y+8.0f},(controlled_astronaut_>=0 && astronaut_views_[controlled_astronaut_].suit_light)?"FPV / L LIGHT ON":"FPV / L LIGHT OFF",0.64f,theme_);
        game_label_fitted(r,{help.x+12.f,help.y+28.f,help.w-24.f,15.f},
            help.w<400?(build_catalog_.selected()?"LMB PLACE / N-M ROTATE / RMB CANCEL":"WASD / SHIFT / SPACE / ALT / V EXIT"):
            build_catalog_.selected()?"LMB PLACE / WHEEL SIZE / N-M ROTATE / ALT CURSOR / TAB TOGGLE / RMB CANCEL":
            "WASD WALK / SHIFT SPRINT / SPACE JUMP / ALT CURSOR / TAB TOGGLE / V EXIT",.43f,theme_,true);

        const std::string environment=first_person_environment_label();
        Rect env{help.x,help.y+help.h+8.0f,help.w,31.0f};
        const bool exterior=environment.find("EXTERIOR")!=std::string::npos;
        r.draw_rect(env,exterior?Color{0.20f,0.055f,0.045f,kHudOpacity}:Color{0.035f,0.095f,0.085f,kHudOpacity});
        game_label_fitted(r,{env.x+10,env.y+8,env.w-20,20},environment,0.48f,theme_);
        const auto feet=first_person_.feet_position();
        for(std::size_t i=0;i<placed_habitats_.size();++i){
            const auto& h=placed_habitats_[i];if(h.kind!=HabitatKind::AirLock || !h.construction.complete())continue;
            const auto& mesh=habitat_models_.mesh(h);const auto local=rotate_y(feet-h.position,-h.rotation_degrees);
            if(std::abs(local.x)>mesh.footprint_x*0.5f+0.2f || std::abs(local.z)>mesh.footprint_z*0.5f)continue;
            const airlock::Cycle idle{};const auto& cycle=i<airlock_cycles_.size()?airlock_cycles_[i]:idle;
            Rect status{env.x,env.y+env.h,env.w,44};r.draw_rect(status,{0.035f,0.075f,0.085f,kHudOpacity});
            std::ostringstream text;text<<"CENTER "<<std::fixed<<std::setprecision(0)<<cycle.pressure*100<<"% OF INSIDE / "<<airlock::phase(cycle);
            game_label_fitted(r,{status.x+10,status.y+6,status.w-20,16},text.str(),0.46f,theme_);
            game_label_fitted(r,{status.x+10,status.y+25,status.w-20,13},"APPROACH NEXT DOOR / WAIT FOR PRESSURE / WALK EITHER DIRECTION",0.39f,theme_,true);
            env.h+=status.h;break;
        }

        const auto manual=manual_door_label();
        if(!manual.empty()){
            Rect control{env.x,env.y+env.h+6,env.w,30};
            r.draw_rect(control,kHudFill);game_label_fitted(r,{control.x+10,control.y+8,control.w-20,18},manual,.48f,theme_);
            render_door_interaction(r);
            env.h+=36;
        }
        if(first_person_carried_resource_){
            const auto& carried=*first_person_carried_resource_;
            std::ostringstream line;
            line<<"CARRYING  "<<material_resource_name(carried.resource)<<"  "<<carried.amount<<" kg";
            Rect carry{PLAYFIELD_MARGIN,env.y+env.h+8.0f,300.0f,44.0f};
            r.draw_rect(carry,{0.035f,0.055f,0.065f,kHudOpacity});
            game_label(r,{carry.x+10.0f,carry.y+8.0f},line.str(),0.54f,theme_);
            game_label(r,{carry.x+10.0f,carry.y+26.0f},"LMB DROP  |  DROP INSIDE DOME TO STORE",0.39f,theme_,true);
        }

        MaterialResource hover_resource=MaterialResource::Iron;
        std::int64_t hover_amount=0;
        float hover_condition=1.0f;
        const char* hover_source=nullptr;
        if(first_person_hover_ground_resource_>=0 && (std::size_t)first_person_hover_ground_resource_<ground_resource_piles_.size()){
            const auto& box=ground_resource_piles_[(std::size_t)first_person_hover_ground_resource_];
            hover_resource=box.resource;
            hover_amount=box.amount;
            hover_condition=box.condition;
            hover_source="GROUND PACKAGE";
        }else if(first_person_hover_storage_resource_>=0 && first_person_hover_storage_box_>=0 &&
                 (std::size_t)first_person_hover_storage_resource_<placed_resources_.size()){
            const auto& dome=placed_resources_[(std::size_t)first_person_hover_storage_resource_];
            if((std::size_t)first_person_hover_storage_box_<dome.stored_boxes.size()){
                const auto& box=dome.stored_boxes[(std::size_t)first_person_hover_storage_box_];
                hover_resource=box.resource;
                hover_amount=box.amount;
                hover_condition=box.condition;
                hover_source="STORAGE PACKAGE";
            }
        }
        if(hover_source){
            const float panel_w=245.0f,panel_h=86.0f;
            Rect hover{std::min(cx+18.0f,ui_width(r)-panel_w-12.0f),
                       std::min(cy+18.0f,ui_height(r)-panel_h-12.0f),panel_w,panel_h};
            r.draw_rect(hover,kHudFill);
            std::ostringstream amount;
            amount<<hover_amount<<" / "<<resource_package_capacity_kg(hover_resource)<<" kg CARRY";
            std::ostringstream condition;
            condition<<"CONDITION  "<<(int)std::lround(std::clamp(hover_condition,0.0f,1.0f)*100.0f)<<"%";
            game_label(r,{hover.x+10.0f,hover.y+8.0f},material_resource_name(hover_resource),0.58f,theme_);
            game_label(r,{hover.x+10.0f,hover.y+29.0f},amount.str(),0.46f,theme_);
            game_label(r,{hover.x+10.0f,hover.y+47.0f},condition.str(),0.43f,theme_,true);
            game_label(r,{hover.x+10.0f,hover.y+65.0f},std::string(hover_source)+"  |  LMB GRAB",0.39f,theme_,true);
        }
        return;
    }
}

void GameApp::render_playfield_crosshair(SceneRenderer& r){
    const float top_px=STATUS_BAR_H*ui_scale_;
    const float bottom_px=(ui_height(r)-BUILD_BAR_H)*ui_scale_;
    if((float)input_.mouse_y<top_px || (float)input_.mouse_y>=bottom_px ||
       mouse_over_selection_panel(input_) || mouse_over_playfield_overlay(input_) || astronaut_status_hover(input_)){
        app_.set_world_cursor(false);
        app_.set_cursor_visible(true);
        return;
    }

    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,r.width(),r.height(),playfield_.size(),hit)){
        app_.set_world_cursor(false);
        app_.set_cursor_visible(true);
        return;
    }

    // Use the same ground-footprint hit tests as world selection. Keep the
    // ground reticle while a placement or other world-position tool owns input.
    const bool positioning=build_catalog_.selected() || interior_unit_tool_ ||
        storage_gate_placement_active_ || corridor_link_source_>=0 ||
        debug_move_pick_robot_>=0 || lander_.awaiting_site || lander_.awaiting_stash ||
        selection_marquee_dragged_;
    if(!positioning && (pick_habitat(hit)>=0 || pick_resource(hit)>=0 ||
        pick_corridor(hit)>=0 || pick_walkway(hit)>=0 || pick_utility(hit)>=0 ||
        (lander_.active() && lunar_lander_segment_blocked(lander_,hit,hit,0.f)))){
        app_.set_world_cursor(false); // native arrow, no screen-space crosshair
        app_.set_cursor_visible(true);
        return;
    }

    // A world-space target, not a camera-facing native cursor. Every stroke
    // is in the playfield plane, so orbit and zoom preserve its orientation.
    app_.set_world_cursor(false);app_.set_cursor_visible(false);
    hit.y=.045f;
    const float radius=std::clamp(camera_.distance()*.014f,.38f,2.8f);
    std::vector<Vec3> strokes;constexpr float pi=3.14159265f;
    for(int quadrant=0;quadrant<4;++quadrant){
        for(int j=0;j<12;++j){const float a=quadrant*pi*.5f+.20f+(pi*.5f-.40f)*j/12,b=quadrant*pi*.5f+.20f+(pi*.5f-.40f)*(j+1)/12;
            strokes.insert(strokes.end(),{hit+Vec3{std::cos(a)*radius,0,std::sin(a)*radius},hit+Vec3{std::cos(b)*radius,0,std::sin(b)*radius}});}
        const float t=quadrant*pi*.5f;const Vec3 d{std::cos(t),0,std::sin(t)};strokes.insert(strokes.end(),{hit+d*(radius*.34f),hit+d*(radius*1.32f)});
    }
    r.draw_lines_3d(strokes,{.015f,.02f,.025f,1},5.f);r.draw_lines_3d(strokes,{.18f,1,.48f,1},2.5f);
    r.draw_lines_3d({hit+Vec3{-radius*.10f,0,0},hit+Vec3{radius*.10f,0,0},hit+Vec3{0,0,-radius*.10f},hit+Vec3{0,0,radius*.10f}},{1,.90f,.16f,1},3.f);
}

} // namespace game
