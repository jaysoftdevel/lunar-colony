#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/world/storage_loading_gate.hpp"
#include <limits>
#include "game/util/world_math.hpp"
#include <cmath>
#include <iomanip>
#include <sstream>

namespace game {

using namespace eng;
using namespace world_math;

void GameApp::update_ore_logistics(float dt){
    for(auto& drill:placed_resources_){
        if(drill.kind!=ResourceKind::OreDriller || !drill.construction.complete() || drill.supply.power_fraction<=0)continue;
        const auto mix=ore::deposit(drill.position);
        const double amount=ore::extraction_kg_per_day*dt/240.0*drill.supply.power_fraction*
            (drill.size==ResourceSizeTier::Small?.5:1.0);
        for(std::size_t i=0;i<mix.size();++i){
            auto& stock=drill.mined_ore_kg[i];
            const bool reserved=std::any_of(builder_robots_.begin(),builder_robots_.end(),[&](const auto& robot){
                const auto& haul=robot.ore_haul;
                return haul.loaded && haul.source==drill.placement_sequence && haul.kind==i;
            });
            const double capacity=ore::stock_capacity_kg-(reserved?ore::box_kg:0);
            stock+=std::min(mix[i]*amount,std::max(0.0,capacity-stock));
        }
    }

    auto& connections=utility_network_.mutable_connections();
    for(std::size_t index=0;index<connections.size();++index){
        auto& belt=connections[index];
        if(belt.kind!=UtilityKind::ConveyorBelt || !belt.construction.complete())continue;
        const auto route=conveyor_route(belt.endpoint_a,belt.endpoint_b,placed_habitats_,placed_resources_);
        if(route!=ConveyorRoute::OreProcessor && route!=ConveyorRoute::OreStorage)continue;
        auto& drill=placed_resources_[belt.endpoint_a.index];
        auto* dome=route==ConveyorRoute::OreStorage?&placed_resources_[belt.endpoint_b.index]:nullptr;
        const auto* plant=route==ConveyorRoute::OreProcessor?&placed_habitats_[belt.endpoint_b.index]:nullptr;
        if(!drill.construction.complete() || drill.supply.power_fraction<=0 ||
           (dome && !dome->construction.complete()) || (plant && !plant->construction.complete()))continue;
        if(belt.ore_cargo==ore::no_cargo){
            for(std::size_t offset=0;offset<ore::materials.size();++offset){
                const auto kind=(belt.ore_next+offset)%ore::materials.size();
                if(drill.mined_ore_kg[kind]<ore::box_kg)continue;
                drill.mined_ore_kg[kind]-=ore::box_kg;
                belt.ore_cargo=static_cast<std::uint8_t>(kind);
                belt.ore_next=static_cast<std::uint8_t>((kind+1)%ore::materials.size());
                belt.ore_distance_m=0;
                break;
            }
        }
        if(belt.ore_cargo==ore::no_cargo)continue;
        const float length=utility_network_.connection_length(index);
        belt.ore_distance_m=std::min(length,belt.ore_distance_m+ore::belt_speed_mps*dt);
        if(belt.ore_distance_m<length)continue;
        if(dome){
            if(storage_dome_used_kg(*dome)+ore::box_kg>storage_dome_capacity_kg(dome->storage_size))continue;
            ++dome->stored_ore_boxes[belt.ore_cargo];
        }else{
            if(plant->supply.power_fraction<=0 || plant->supply.breathability()<95)continue;
            // Only a processor converts ore into construction stock.
            const auto& port=belt.endpoint_b;
            const Vec3 side{-port.normal.z,0,port.normal.x};
            const Vec3 output=port.position+port.normal*2.0f+side*1.7f;
            if(!spawn_produced_resource(ore::materials[belt.ore_cargo],ore::refined_box_kg,output,false))continue;
        }
        belt.ore_cargo=ore::no_cargo;
        belt.ore_distance_m=0;
    }
}

void GameApp::update_ore_hauling(float dt,const std::function<bool(std::size_t,Vec3)>& clear,
    const std::function<void(std::size_t,Vec3)>& move,const std::function<void(std::size_t)>& stop){
    const auto has_conveyor=[&](std::size_t drill){
        return std::any_of(utility_network_.connections().begin(),utility_network_.connections().end(),[&](const auto& link){
            return link.kind==UtilityKind::ConveyorBelt && link.construction.complete() && link.endpoint_a.index==drill;
        });
    };
    const auto plant_ready=[](const PlacedHabitat& plant){
        return (plant.kind==HabitatKind::ProcessingFacility || plant.kind==HabitatKind::Factory) &&
            plant.construction.complete() && plant.supply.power_fraction>0 && plant.supply.breathability()>=95;
    };
    const auto dome_ready=[](const PlacedResource& dome){
        return dome.kind==ResourceKind::StorageDome && dome.construction.complete() &&
            storage_dome_used_kg(dome)+ore::box_kg<=storage_dome_capacity_kg(dome.storage_size) &&
            storage_loading::has_ready_gate(dome);
    };
    const auto resource_index=[&](std::uint64_t sequence){
        for(std::size_t i=0;i<placed_resources_.size();++i)
            if(sequence && placed_resources_[i].placement_sequence==sequence)return i;
        return kNoRobotIndex;
    };
    const auto plant_index=[&](std::uint64_t sequence){
        for(std::size_t i=0;i<placed_habitats_.size();++i)
            if(sequence && placed_habitats_[i].placement_sequence==sequence)return i;
        return kNoRobotIndex;
    };

    for(std::size_t ri=0;ri<builder_robots_.size();++ri){
        auto& rover=builder_robots_[ri];
        auto& haul=rover.ore_haul;
        if(rover.role!=RobotRole::Carrier || rover.operation_retry.replan_pending() || trader_robot_busy(ri))continue;
        const auto finish=[&]{
            stop(ri);
            haul={};
            rover.mode=BuilderRobotMode::Idle;
            rover.action=BuilderRobotAction::None;
            rover.interaction_position_valid=false;
        };

        const auto find_destination=[&]{
            // Prefer processing while a receiver can work. Storage is the fallback.
            std::uint64_t best=0;
            float nearest=std::numeric_limits<float>::max();
            const auto consider=[&](const auto& item){
                if(!item.placement_sequence)return;
                for(const auto& other:builder_robots_)
                    if(&other!=&rover && other.ore_haul.active() && other.ore_haul.destination==item.placement_sequence)return;
                const float distance=distance_xz(rover.position,item.position);
                if(distance>=nearest)return;
                nearest=distance;
                best=item.placement_sequence;
            };
            for(const auto& plant:placed_habitats_)if(plant_ready(plant))consider(plant);
            if(!best)for(const auto& dome:placed_resources_)if(dome_ready(dome))consider(dome);
            return best;
        };

        if(!haul.active()){
            if(rover.mode!=BuilderRobotMode::Idle || rover.target.valid() || !rover.cargo.empty() ||
               rover.action!=BuilderRobotAction::None || rover.debug_move_active || rover.drop_exit_pending ||
               rover.starter_charge_active || rover.battery_charge_kwh<rover.battery_capacity_kwh*.2f)continue;

            float nearest=std::numeric_limits<float>::max();
            for(std::size_t i=0;i<placed_resources_.size();++i){
                const auto& drill=placed_resources_[i];
                if(drill.kind!=ResourceKind::OreDriller || !drill.construction.complete() ||
                   !drill.placement_sequence || has_conveyor(i))continue;
                const auto stock=std::max_element(drill.mined_ore_kg.begin(),drill.mined_ore_kg.end());
                if(*stock<ore::box_kg)continue;
                bool leased=false;
                for(const auto& other:builder_robots_)
                    if(&other!=&rover && other.ore_haul.active() && other.ore_haul.source==drill.placement_sequence)leased=true;
                const float distance=distance_xz(rover.position,drill.position);
                if(leased || distance>=nearest)continue;
                nearest=distance;
                haul.source=drill.placement_sequence;
                haul.kind=static_cast<std::uint8_t>(stock-drill.mined_ore_kg.begin());
            }
            if(!haul.active())continue;
            haul.destination=find_destination();
            if(!haul.destination){haul={};continue;}
        }

        if(!clear(ri,rover.position))continue; // Safety relocation still owns the rover.
        const auto drill=resource_index(haul.source);
        auto plant=plant_index(haul.destination);
        auto dome=resource_index(haul.destination);
        if(!((plant!=kNoRobotIndex && plant_ready(placed_habitats_[plant])) ||
             (dome!=kNoRobotIndex && dome_ready(placed_resources_[dome])))){
            const auto replacement=find_destination();
            if(replacement!=haul.destination){
                haul.destination=replacement;
                haul.goal_valid=false;
                rover.action=BuilderRobotAction::None;
            }
            plant=plant_index(haul.destination);
            dome=resource_index(haul.destination);
        }

        const bool return_crate=haul.loaded && !haul.destination;
        if(return_crate && drill==kNoRobotIndex){
            stop(ri);
            haul.goal_valid=false;
            rover.action=BuilderRobotAction::None;
            continue;
        }
        if(!haul.loaded && (!haul.destination || drill==kNoRobotIndex || has_conveyor(drill) ||
           placed_resources_[drill].mined_ore_kg[haul.kind]<ore::box_kg)){finish();continue;}

        rover.mode=haul.loaded?BuilderRobotMode::Delivering:BuilderRobotMode::Fetching;
        rover.workshop_returning=false;
        rover.workshop_parked=false;
        const bool at_driller=!haul.loaded || return_crate;
        Vec3 focus{};
        if(!haul.goal_valid){
            float nearest=std::numeric_limits<float>::max();
            const auto consider=[&](Vec3 point){
                const float distance=distance_xz(rover.position,point);
                if(distance<nearest && clear(ri,point)){
                    nearest=distance;
                    haul.goal=point;
                    haul.goal_valid=true;
                }
            };
            if(at_driller && drill!=kNoRobotIndex){
                const auto& source=placed_resources_[drill];
                const auto& mesh=resource_models_.mesh(source);
                // Approach the outdoor stack from its free side, outside the crate rows.
                const float x=(static_cast<float>(haul.kind)-1.5f)*.5f;
                for(const float side:{0.f,-1.f,1.f,-2.f,2.f})
                    consider(source.position+rotate_y({x+side,0,mesh.footprint_z*.5f+2.5f},source.rotation_degrees));
            }else if(plant!=kNoRobotIndex){
                const auto& receiver=placed_habitats_[plant];
                const auto& mesh=habitat_models_.mesh(receiver);
                for(int side=0;side<8;++side){
                    const float angle=side*.7853981634f;
                    const Vec3 local{std::sin(angle)*(mesh.footprint_x*.5f+2.2f),0,
                        std::cos(angle)*(mesh.footprint_z*.5f+2.2f)};
                    consider(receiver.position+rotate_y(local,receiver.rotation_degrees));
                }
            }else if(dome!=kNoRobotIndex){
                const auto& storage=placed_resources_[dome];
                for(const int face:storage_loading::faces(storage,true))
                    if(face>=0)consider(storage_loading::service(storage,face));
            }
        }
        if(!haul.goal_valid){
            stop(ri);
            if(!haul.loaded)finish();
            continue;
        }
        if(at_driller && drill!=kNoRobotIndex){
            const auto& source=placed_resources_[drill];
            const auto& mesh=resource_models_.mesh(source);
            focus=source.position+rotate_y({(static_cast<float>(haul.kind)-1.5f)*.5f,.035f,
                mesh.footprint_z*.5f+.74f},source.rotation_degrees);
        }else if(plant!=kNoRobotIndex)focus=placed_habitats_[plant].position;
        else if(dome!=kNoRobotIndex)focus=placed_resources_[dome].position;
        rover.interaction_position=haul.goal;
        rover.interaction_focus_position=focus;
        rover.interaction_position_valid=true;

        if(distance_xz(rover.position,haul.goal)>.32f){
            rover.action=BuilderRobotAction::None;
            move(ri,haul.goal);
            continue;
        }
        stop(ri);
        const auto action=haul.loaded?BuilderRobotAction::PutDown:BuilderRobotAction::Pickup;
        if(rover.action!=action){
            rover.action=action;
            rover.action_elapsed=0;
            rover.action_duration=1.5f;
            rover.action_amount=ore::box_kg;
            rover.action_material=ore::materials[haul.kind];
            rover.action_focus_position=focus;
        }
        rover.action_elapsed+=dt;
        if(rover.action_elapsed<rover.action_duration)continue;

        if(!haul.loaded){
            placed_resources_[drill].mined_ore_kg[haul.kind]-=ore::box_kg;
            haul.loaded=true;
            haul.goal_valid=false;
            rover.action=BuilderRobotAction::None;
        }else if(return_crate && drill!=kNoRobotIndex){
            auto& stock=placed_resources_[drill].mined_ore_kg[haul.kind];
            if(stock+ore::box_kg<=ore::stock_capacity_kg+ore::box_kg){stock+=ore::box_kg;finish();}
        }else if(dome!=kNoRobotIndex){
            ++placed_resources_[dome].stored_ore_boxes[haul.kind];
            finish();
        }else if(plant!=kNoRobotIndex){
            const Vec3 outward=normalize(haul.goal-focus);
            const Vec3 side{-outward.z,0,outward.x};
            const bool deposited=spawn_produced_resource(ore::materials[haul.kind],ore::refined_box_kg,
                haul.goal+side*2.0f,false);
            record_robot_operation_result(rover,deposited,haul.destination);
            if(deposited)finish();
            else rover.action=BuilderRobotAction::None;
        }
    }
}

bool GameApp::conveyor_route_clear(const std::vector<Vec3>& path) const {
    const float limit=playfield_.size()*.5f-ore::belt_half_width;
    for(const auto& point:path)
        if(std::abs(point.x)>limit || std::abs(point.z)>limit)return false;
    const auto clear=[&](const auto& connections,float half_width){
        for(std::size_t i=1;i<path.size();++i){
            const auto belt=path[i]-path[i-1];
            if(std::hypot(belt.x,belt.z)<.001f)continue;
            const auto belt_box=footprint((path[i]+path[i-1])*.5f,
                std::atan2(belt.x,belt.z)*57.29578f,2*ore::belt_half_width,std::hypot(belt.x,belt.z));
            for(const auto& link:connections)for(std::size_t j=1;j<link.path.size();++j){
                const auto run=link.path[j]-link.path[j-1];
                if(overlaps(belt_box,footprint((link.path[j]+link.path[j-1])*.5f,
                    std::atan2(run.x,run.z)*57.29578f,2*half_width,std::hypot(run.x,run.z)),.08f))return false;
            }
        }
        return true;
    };
    return gate_route_clear(path,ore::belt_half_width) &&
        clear(corridor_network_.connections(),geometry::kCorridorHalfWidth) &&
        clear(walkway_network_.connections(),geometry::kWalkwayHalfWidth);
}

bool GameApp::ore_survey_location(Vec3& point) const {
    const auto* tool=build_catalog_.selected();
    if(tool && tool->kind==BuildObjectKind::Resource && tool->resource_kind==ResourceKind::OreDriller){
        if(camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),point)){
            point={std::round(point.x),0,std::round(point.z)};
            return true;
        }
        return false;
    }
    if(selection_kind_!=SelectionKind::Resource || selected_resource_<0 ||
       (std::size_t)selected_resource_>=placed_resources_.size())return false;
    const auto& drill=placed_resources_[(std::size_t)selected_resource_];
    if(drill.kind!=ResourceKind::OreDriller || drill.construction.complete())return false;
    point=drill.position;
    return true;
}

void GameApp::render_ore_deposits(Renderer& r) const {
    Vec3 survey{};
    if(!ore_survey_location(survey))return;
    constexpr int cells=32;
    const float step=(float)playfield_.size()/cells;
    const float half=step*cells*.5f;
    std::array<std::vector<Vec3>,ore::materials.size()> samples;
    for(int z=0;z<cells;++z)for(int x=0;x<cells;++x){
        const Vec3 point{-half+(x+.5f)*step,.035f,-half+(z+.5f)*step};
        const auto mix=ore::deposit(point);
        const auto best=std::max_element(mix.begin(),mix.end());
        const float arm=step*(.10f+.32f*(float)*best);
        auto& lines=samples[(std::size_t)(best-mix.begin())];
        lines.insert(lines.end(),{point-Vec3{arm,0,0},point+Vec3{arm,0,0},point-Vec3{0,0,arm},point+Vec3{0,0,arm}});
    }
    for(std::size_t i=0;i<samples.size();++i){
        auto color=ore::colors[i];color.a=.67f;
        r.draw_lines_3d(samples[i],color,2.0f);
    }
}

void GameApp::render_ore_survey(Renderer& r) const {
    Vec3 survey{};
    if(!ore_survey_location(survey))return;
    const auto mix=ore::deposit(survey);
    const float right=progress_tree_visible()?progress_tree_panel_rect().x-8.f:
        debug_frame_visible()?debug_frame_rect().x-8.f:ui_width(r)-layout::PLAYFIELD_MARGIN;
    const float left=selection_kind_==SelectionKind::None?layout::PLAYFIELD_MARGIN+430.f:selection_panel_rect().x+selection_panel_rect().w;
    const float width=std::min(242.f,std::max(116.f,right-left-8.f));
    const Rect panel{right-width,layout::STATUS_BAR_H+34.f,width,107.f};
    layout::hud_panel(r,panel,theme_);
    layout::game_label_fitted(r,{panel.x+10,panel.y+7,panel.w-20,17},"ORE SURVEY / DOMINANT DEPOSITS",.52f,theme_);
    for(std::size_t i=0;i<mix.size();++i){
        const float y=panel.y+26+16*(float)i;
        r.draw_rect({panel.x+12,y+3,9,9},ore::colors[i]);
        std::ostringstream text;
        text<<ore::names[i]<<"   "<<std::fixed<<std::setprecision(1)<<100*mix[i]<<"%";
        layout::game_label_fitted(r,{panel.x+29,y,panel.w-41,15},text.str(),.48f,theme_);
    }
}

} // namespace game
