#include "game/game_app.hpp"
#include "game/util/world_math.hpp"
#include "game/world/robot_workshop.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <numeric>
#include <string_view>

namespace game::sound {
namespace {
constexpr std::uint64_t music_key=1;
}

Soundscape::Soundscape(){
    if(const char* level=std::getenv("GAME_AUDIO_VOLUME")){
        char* end{};
        const float value=std::strtof(level,&end);
        if(end!=level && !*end && std::isfinite(value)) set_volume(value/100.0f);
    }
    if(const char* enabled=std::getenv("GAME_AUDIO"); enabled && std::string_view(enabled)=="0") volume_=0;
}

void Soundscape::set_volume(float volume){ volume_=std::clamp(volume,0.0f,1.0f); }

void Soundscape::set_music_enabled(bool enabled){
    if(music_enabled_==enabled) return;
    music_enabled_=enabled;
    if(!enabled) audio_.stop_loop(music_key);
}

void Soundscape::update_music(){
    if(!music_enabled_) return;
    if(!theme_requested_){
        theme_job_=std::async(std::launch::async,synthesize_theme);
        theme_requested_=true;
    }
    if(theme_job_.valid() && theme_job_.wait_for(std::chrono::seconds(0))==std::future_status::ready){
        const auto pcm=theme_job_.get();
        theme_clip_=audio_.add_stereo(pcm,theme_sample_rate);
    }
    // Submit the same protected voice in the menu and playfield to carry the
    // musical phrase through screen changes. It uses no spatial attenuation.
    if(theme_clip_) audio_.loop(music_key,theme_clip_,{{},.30f,1.0f,1.0f,true,false,12});
}

bool Soundscape::begin(eng::Vec3 listener,eng::Vec3 direction,eng::Vec3 up){
    ++frame_;
    if(!initialized_ && volume_>0.0f) initialized_=audio_.initialize();
    if(!initialized_) return false;
    audio_.begin(listener,direction,up,volume_*volume_);
    if(volume_>0.0f) update_music();
    if(frame_%256==0){
        std::erase_if(observations_,[&](const auto& item){ return frame_-item.second.frame>2; });
    }
    return volume_>0.0f;
}

void Soundscape::finish(float dt){ if(initialized_) audio_.finish(dt); }

eng::Audio::Clip Soundscape::clip(Cue cue,std::size_t variant){
    const auto i=static_cast<std::size_t>(cue);
    if(!attempted_[i][variant]){
        attempted_[i][variant]=true;
        const auto pcm=synthesize(cue,static_cast<std::uint32_t>(variant));
        clips_[i][variant]=audio_.add_mono(pcm,sample_rate);
    }
    return clips_[i][variant];
}

void Soundscape::play(Cue cue,const eng::SoundVoice& voice){
    if(const auto variants=variant_count(cue)) audio_.play(clip(cue,variant_counter_++%variants),voice);
}

void Soundscape::loop(std::uint64_t key,Cue cue,const eng::SoundVoice& voice,std::size_t variant){
    if(const auto variants=variant_count(cue)) audio_.loop(key,clip(cue,variant%variants),voice);
}

std::optional<std::uint64_t> Soundscape::observe(std::uint64_t key,std::uint64_t value){
    auto& old=observations_[key];
    const auto result=old.frame && old.frame+1==frame_?std::optional{old.value}:std::nullopt;
    old={frame_,value};
    return result;
}

} // namespace game::sound

namespace game {
namespace {
using eng::Vec3;
using sound::Cue;

enum Channel : std::uint64_t {
    Global=1, Ambient, RobotMotor, RobotTool, RobotEvent, RobotAlarm, Habitat, Resource, Gate,
    Corridor, Utility, Walkway, BeltMotor, BeltCargo, Door, Airlock, CrewFeet, CrewSuit, CrewEvent, CrewAlarm
};
constexpr std::uint64_t key(Channel channel,std::uint64_t id){
    return (channel<<56)|(id&0x00ffffffffffffffull);
}

std::uint64_t position_key(Vec3 p){
    const auto x=static_cast<std::uint64_t>(std::lround(p.x*100));
    const auto y=static_cast<std::uint64_t>(std::lround(p.y*100));
    const auto z=static_cast<std::uint64_t>(std::lround(p.z*100));
    return (x*0x9e3779b185ebca87ull)^(y*0xc2b2ae3d27d4eb4full)^(z*0x165667b19e3779f9ull);
}

Cue habitat_loop(HabitatKind kind){
    switch(kind){
        case HabitatKind::OxygenGeneratorDome: return Cue::Pump;
        case HabitatKind::BioDome: return Cue::Vent;
        case HabitatKind::ProcessingFacility:
        case HabitatKind::Factory:
        case HabitatKind::Manufactory: return Cue::Mill;
        case HabitatKind::Laboratory:
        case HabitatKind::SickBay: return Cue::Chiller;
        default: return Cue::Cabin;
    }
}

Cue resource_loop(ResourceKind kind){
    switch(kind){
        case ResourceKind::WaterDriller:
        case ResourceKind::OreDriller: return Cue::Drill;
        case ResourceKind::WaterPurifier: return Cue::Pump;
        case ResourceKind::HydrogenGenerator: return Cue::Mill;
        case ResourceKind::HydrogenTank:
        case ResourceKind::OxygenTank:
        case ResourceKind::PowerStorage: return Cue::Chiller;
        case ResourceKind::RobotWorkshop: return Cue::Vent;
        default: return Cue::Count;
    }
}

Vec3 center_of(const std::vector<Vec3>& path){
    return path.empty()?Vec3{}:path[path.size()/2];
}
}

void GameApp::update_soundscape(float dt,const eng::InputState& input){
    if(dt<=0.0f) return;
    const bool world=screen_==Screen::Playfield;
    const bool campus=world && active_game_phase_==GamePhase::Planning;
    const bool lunar=world && !campus;
    const bool fpv=lunar && first_person_mode_==FirstPersonMode::Active;
    const Vec3 feet=first_person_.feet_position();
    const Vec3 camera_eye=camera_.eye_position();
    const Vec3 listener=fpv?first_person_.eye_position():world?camera_.target()+Vec3{0,2,0}:Vec3{};
    Vec3 forward=fpv?first_person_.look_direction():eng::normalize(camera_.target()-camera_eye);
    Vec3 right=eng::normalize(eng::cross(forward,{0,1,0}));
    if(eng::length(right)<0.01f) right={1,0,0};
    if(eng::length(forward)<0.01f) forward={0,0,-1};
    const Vec3 up=eng::normalize(eng::cross(right,forward));

    auto& audio=soundscape_;
    const bool active=audio.begin(listener,forward,up);
    if(!active){ audio.tracking_feet=false; audio.finish(dt); return; }
    audio.warning_cooldown=std::max(0.0f,audio.warning_cooldown-dt);
    audio.completion_cooldown=std::max(0.0f,audio.completion_cooldown-dt);
    const auto relative=[](float gain,float pitch=1.0f,std::uint8_t priority=3){
        return eng::SoundVoice{{},gain,pitch,1.0f,true,false,priority};
    };
    const auto ui_sound=[&](Cue cue,float gain=.26f){ audio.play(cue,relative(gain)); };
    const auto screen_before=audio.observe(key(Global,1),static_cast<std::uint64_t>(screen_));
    const auto phase_before=audio.observe(key(Global,2),static_cast<std::uint64_t>(active_game_phase_));
    const bool screen_changed=screen_before && *screen_before!=static_cast<std::uint64_t>(screen_);
    const bool phase_changed=phase_before && *phase_before!=static_cast<std::uint64_t>(active_game_phase_);
    if(screen_changed) ui_sound(world?Cue::Radio:Cue::Pick,.21f);
    else if(input.escape_pressed || input.right_pressed) ui_sound(Cue::Cancel,.16f);
    else if(input.left_pressed && (!lunar || input.mouse_y<float(viewport_height_)*.082f ||
             input.mouse_y>float(viewport_height_)*.90f || mouse_over_selection_panel(input))) ui_sound(Cue::Pick,.15f);

    if(phase_changed && lunar && *phase_before==static_cast<std::uint64_t>(GamePhase::Planning) &&
       active_game_phase_==GamePhase::RobotPreparation){
        audio.launch_seconds=3.6f;
        ui_sound(Cue::Radio,.38f);
    }
    if(world && audio.launch_seconds>0.0f){
        audio.launch_seconds=std::max(0.0f,audio.launch_seconds-dt);
        const float rise=std::min(1.0f,(3.6f-audio.launch_seconds)*2.8f);
        audio.loop(key(Ambient,1),Cue::Exhaust,relative(.34f*rise*std::min(1.0f,audio.launch_seconds),.87f,4));
    }
    if(campus){
        const float level=planning_.view==PlanningView::Site?.23f:.075f;
        audio.loop(key(Ambient,2),Cue::Campus,relative(level,1.0f,0));
        const auto funds=static_cast<std::uint64_t>(std::max<std::int64_t>(0,planning_.funds_eur));
        if(const auto previous=audio.observe(key(Global,3),funds); previous && funds>*previous) ui_sound(Cue::Success,.34f);
        const auto tech=std::accumulate(planning_.technologies.begin(),planning_.technologies.end(),0u);
        if(const auto previous=audio.observe(key(Global,4),tech); previous && tech>*previous) ui_sound(Cue::Build,.30f);
        if(const auto previous=audio.observe(key(Global,5),planning_.mission_paid); previous && !*previous && planning_.mission_paid)
            ui_sound(Cue::Success,.34f);
    }
    if(!lunar){
        audio.tracking_feet=false;
        audio.finish(dt);
        return;
    }

    bool lock=false;
    const auto* air=fpv?astronaut_air_at(feet,lock):nullptr;
    bool cabin=air && air->breathability()>=kCrewBreathableAir;
    if(cabin && lock){
        for(const auto& cycle:airlock_cycles_){
            const auto h=std::find_if(placed_habitats_.begin(),placed_habitats_.end(),[&](const PlacedHabitat& item){
                return item.placement_sequence==cycle.identity;
            });
            if(h==placed_habitats_.end()) continue;
            const auto local=world_math::rotate_y(feet-h->position,-h->rotation_degrees);
            const auto& mesh=habitat_models_.mesh(*h);
            if(std::abs(local.x)<airlock::partition_x(mesh) && std::abs(local.z)<mesh.footprint_z*.5f){
                cabin=cycle.pressure>=.57f;
                break;
            }
        }
    }
    const bool paused=game_speed_==GameSpeed::Pause;
    const auto nearby=[&](Vec3 p,float radius){
        const auto d=p-listener;
        return eng::dot(d,d)<radius*radius;
    };
    const auto world_voice=[&](Vec3 p,float gain,float range,float pitch=1.0f,std::uint8_t priority=1,bool indoor=false){
        const bool direct=fpv && cabin && indoor;
        const float level=gain*(direct?1.0f:fpv?.27f:.52f);
        return eng::SoundVoice{p,level,pitch,range*(fpv&&!direct?.62f:1.0f),false,!direct,priority};
    };
    const auto alarm=[&](float level){
        if(audio.warning_cooldown>0.0f) return;
        ui_sound(Cue::Alarm,level);
        audio.warning_cooldown=2.2f;
    };
    const auto completion=[&](Vec3 p){
        if(audio.completion_cooldown>0.0f || !nearby(p,72.0f)) return;
        audio.play(Cue::Build,world_voice(p,.70f,75,1,3,true));
        audio.completion_cooldown=.65f;
    };

    audio.loop(key(Ambient,3),fpv?(cabin?Cue::Cabin:Cue::Suit):Cue::Receiver,
        relative(fpv?(cabin?.22f:.25f):.105f,1.0f,2));
    if(fpv && !cabin){
        audio.loop(key(Ambient,4),Cue::Breath,relative(.23f,1.0f,2));
    }

    const auto view=static_cast<std::uint64_t>(first_person_mode_);
    if(const auto previous=audio.observe(key(Global,6),view); previous && *previous!=view) ui_sound(Cue::Radio,.31f);
    if(fpv){
        const bool grounded=first_person_.grounded();
        if(audio.tracking_feet){
            const auto move=feet-audio.feet;
            const float stride=std::hypot(move.x,move.z);
            if(grounded && !audio.grounded)
                audio.play(Cue::BodyLand,relative(cabin?.46f:.34f,cabin?1.05f:.86f,4));
            if(grounded && stride<2.0f) audio.step_distance+=stride;
            if(grounded && audio.step_distance>=.69f){
                audio.step_distance=std::fmod(audio.step_distance,.69f);
                audio.play(air?Cue::FootDeck:Cue::FootDust,relative(cabin?.40f:.28f,cabin?1.0f:.90f,3));
            }
        }else audio.step_distance=0;
        audio.feet=feet;
        audio.grounded=grounded;
        audio.tracking_feet=true;
        const bool carrying=first_person_carried_resource_.has_value();
        if(const auto old=audio.observe(key(Global,7),carrying); old && bool(*old)!=carrying)
            ui_sound(carrying?Cue::Lift:Cue::Crate,.44f);
    }else audio.tracking_feet=false;

    if(const auto previous=audio.observe(key(Global,8),next_blueprint_placement_sequence_);
       previous && next_blueprint_placement_sequence_>*previous && input.left_pressed) ui_sound(Cue::Place,.34f);
    const std::uint64_t selection=static_cast<std::uint64_t>(selection_kind_)*1000003ull+
        static_cast<std::uint64_t>(selected_astronaut_+1)*131ull+
        static_cast<std::uint64_t>(selected_habitat_+selected_resource_+selected_robot_+
            selected_corridor_+selected_utility_+selected_walkway_+selected_ground_resource_+8);
    if(const auto previous=audio.observe(key(Global,9),selection); previous && *previous!=selection && input.left_pressed)
        ui_sound(Cue::Pick,.23f);
    if(input.left_pressed && !placement_tip_detail_.empty())
        ui_sound(Cue::Reject,.21f);

    for(std::size_t i=0;i<placed_habitats_.size();++i){
        const auto& h=placed_habitats_[i];
        const auto id=key(Habitat,h.placement_sequence?h.placement_sequence:i+1);
        const bool built=h.construction.complete();
        const std::uint64_t state=std::uint64_t(h.supply.warnings)<<1 | std::uint64_t(built);
        if(const auto previous=audio.observe(id,state); previous){
            if(built && !(*previous&1)) completion(h.position);
            if(built && (h.supply.warnings & (NoPower|LowAir)) & ~(*previous>>1) &&
               (selected_habitat_==static_cast<int>(i) || nearby(h.position,6.0f))) alarm(.32f);
        }
        if(built && h.power_enabled && h.supply.power_fraction>=.5 && nearby(h.position,31.0f)){
            const Cue cue=habitat_loop(h.kind);
            const bool idle_pump=cue==Cue::Pump && (paused || h.supply.oxygen_produced_kg_per_day<=0.0);
            audio.loop(id,idle_pump?Cue::Vent:cue,world_voice(h.position+Vec3{0,1.8f,0},.24f,31,1,1,true));
        }
    }

    for(std::size_t i=0;i<placed_resources_.size();++i){
        const auto& item=placed_resources_[i];
        const auto seq=item.placement_sequence?item.placement_sequence:i+1;
        const auto id=key(Resource,seq);
        const bool built=item.construction.complete();
        if(const auto old=audio.observe(id,built); old && !*old && built) completion(item.position);
        if(!built || !nearby(item.position,48.0f)) continue;
        const Cue cue=resource_loop(item.kind);
        if(cue!=Cue::Count && item.power_enabled && item.supply.power_fraction>=.5 && (!paused || cue!=Cue::Drill))
            audio.loop(id,cue,world_voice(item.position+Vec3{0,1.0f,0},cue==Cue::Drill?.48f:.24f,47,1,1));
        if(item.kind==ResourceKind::SolarPanel && item.solar_tracking_enabled && item.solar_visual_initialized && !paused){
            const auto sun=status_.sun_pose();
            const auto target=sun.above_horizon?sun.direction:solar_tracking::fixed_panel_normal(item.rotation_degrees);
            if(eng::dot(target,item.solar_visual_normal)<.99985f)
                audio.loop(id,Cue::Motor,world_voice(item.position+Vec3{0,3.2f,0},.13f,19,1.33f));
        }
        if(item.kind==ResourceKind::RobotWorkshop || item.kind==ResourceKind::StorageGate){
            const float open=item.kind==ResourceKind::RobotWorkshop?item.workshop_gate_open:item.storage_extension_open;
            const auto now=static_cast<std::uint64_t>(std::clamp(open,0.0f,1.0f)*1024.0f);
            if(const auto old=audio.observe(key(Gate,seq),now); old){
                if(now && !*old) audio.play(Cue::Hatch,world_voice(item.position,.44f,27,.84f,2));
                if(!now && *old) audio.play(Cue::Seal,world_voice(item.position,.48f,27,.84f,2));
            }
        }
    }

    const auto network=[&](const auto& links,Channel type){
        for(std::size_t i=0;i<links.size();++i){
            const auto& link=links[i];
            const bool built=link.construction.complete();
            const auto seq=link.placement_sequence?link.placement_sequence:i+1;
            if(const auto old=audio.observe(key(type,seq),built); old && !*old && built) completion(center_of(link.path));
        }
    };
    network(corridor_network_.connections(),Corridor);
    network(walkway_network_.connections(),Walkway);
    network(utility_network_.connections(),Utility);
    for(const auto& link:utility_network_.connections()){
        if(!link.construction.complete() || link.kind!=UtilityKind::ConveyorBelt) continue;
        const bool cargo=link.ore_cargo<ore::no_cargo;
        const auto* lot=static_cast<const TradeLot*>(nullptr);
        if(trader_.phase==TradePhase::Loading){
            const auto it=std::find_if(trader_.lots.begin(),trader_.lots.end(),[&](const TradeLot& batch){
                return batch.conveyor_sequence==link.placement_sequence && batch.remaining>0 && batch.conveyor_distance_m>0;
            });
            if(it!=trader_.lots.end()) lot=&*it;
        }
        const bool present=cargo || lot;
        const auto last_cargo=audio.observe(key(BeltCargo,link.placement_sequence),present);
        if(last_cargo && bool(*last_cargo)!=present && !paused)
            audio.play(Cue::Ore,world_voice(present?link.endpoint_a.position:link.endpoint_b.position,.40f,27,1,2));
        const float distance=cargo?link.ore_distance_m:lot?lot->conveyor_distance_m:0.0f;
        const auto meters=static_cast<std::uint64_t>(std::max(distance,0.0f)*10000.0f);
        const auto last_distance=audio.observe(key(BeltMotor,link.placement_sequence),meters);
        if(!paused && present && (!last_distance || meters!=*last_distance)){
            const Vec3 p=link.cargo_position(distance);
            audio.loop(key(BeltMotor,link.placement_sequence),Cue::Belt,world_voice(p,.34f,31));
        }
    }

    for(const auto& door:pressure_doors_){
        const auto id=key(Door,position_key(door.door.wall_position));
        const auto value=static_cast<std::uint64_t>(std::clamp(door.open_fraction,0.0f,1.0f)*1024.0f);
        if(const auto old=audio.observe(id,value); old){
            if(value && !*old) audio.play(Cue::Hatch,world_voice(door.door.position,.65f,22,1,3,true));
            if(!value && *old) audio.play(Cue::Seal,world_voice(door.door.position,.66f,22,1,3,true));
        }
    }
    for(const auto& cycle:airlock_cycles_){
        if(!cycle.equalizing) continue;
        const auto h=std::find_if(placed_habitats_.begin(),placed_habitats_.end(),[&](const PlacedHabitat& item){
            return item.placement_sequence==cycle.identity;
        });
        if(h!=placed_habitats_.end())
            audio.loop(key(Airlock,cycle.identity),Cue::Equalize,world_voice(h->position+Vec3{0,1.2f,0},.52f,21,1,3,true),cycle.target>.5f);
    }

    for(std::size_t i=0;i<builder_robots_.size();++i){
        const auto& rover=builder_robots_[i];
        const auto p=rover.visual_initialized?rover.visual_position:rover.position;
        const float speed=rover.current_speed_mps;
        if(!paused && speed>.12f)
            audio.loop(key(RobotMotor,i+1),Cue::Motor,
                world_voice(p,.23f+.24f*std::min(speed/6.0f,1.0f),39,.76f+.095f*speed),
                rover.role==RobotRole::Carrier);
        const auto action=static_cast<std::uint64_t>(rover.action);
        const auto old=audio.observe(key(RobotEvent,i+1),action);
        if(!paused && old && *old!=action){
            if(rover.action==BuilderRobotAction::Pickup) audio.play(Cue::Lift,world_voice(p,.55f,29,1,2));
            if(rover.action==BuilderRobotAction::PutDown || rover.action==BuilderRobotAction::ApplyResources)
                audio.play(Cue::Crate,world_voice(p,.53f,29,1,2));
        }
        if(!paused && (rover.action==BuilderRobotAction::Welding || rover.action==BuilderRobotAction::BuildUp ||
                       rover.action==BuilderRobotAction::Dismantle))
            audio.loop(key(RobotTool,i+1),rover.action==BuilderRobotAction::Welding?Cue::Weld:Cue::Mill,
                world_voice(p,.41f,29,1.0f,2));
        const bool low=rover.battery_capacity_kwh>0 && rover.battery_charge_kwh/rover.battery_capacity_kwh<.08f;
        if(const auto last=audio.observe(key(RobotAlarm,i+1),low); last && low && !*last &&
           (selected_robot_==static_cast<int>(i) || nearby(p,12.0f))) alarm(.33f);
    }

    if(astronaut_test_active()){
        for(std::size_t i=0;i<test_astronauts_.size();++i){
            const auto& actor=test_astronauts_[i];
            const auto phase=static_cast<std::uint64_t>(std::clamp(actor.stride_phase,0.0f,.99999f)*2.0f);
            const auto previous=audio.observe(key(CrewFeet,i+1),phase);
            const auto previous_suit=audio.observe(key(CrewSuit,i+1),actor.suited);
            const auto activity=static_cast<std::uint64_t>(actor.activity);
            const auto previous_activity=audio.observe(key(CrewEvent,i+1),activity);
            if(!paused && previous && *previous!=phase && actor.speed>.13f){
                const bool inside=!actor.suited;
                audio.play(inside?Cue::FootDeck:Cue::FootDust,
                    world_voice(actor.position,.42f,16,.91f+.12f*std::min(actor.speed,2.0f),2,inside));
            }
            if(previous_suit && bool(*previous_suit)!=actor.suited)
                audio.play(Cue::Seal,world_voice(actor.position,.31f,14,1.28f,2,!actor.suited));
            if(previous_activity && *previous_activity!=activity && actor.activity==AstronautActivity::Knockout)
                audio.play(Cue::BodyLand,world_voice(actor.position,.47f,17,.91f,3,!actor.suited));
            if(previous_activity && *previous_activity!=activity && actor.activity==AstronautActivity::Manual)
                ui_sound(Cue::Radio,.24f);
            const bool critical=(actor.suited && actor.suit_oxygen_kg<kSuitOxygenCapacityKg*.10) ||
                actor.health<.28f || actor.hunger>=.96f || actor.thirst>=.96f;
            if(const auto old=audio.observe(key(CrewAlarm,i+1),critical); old && critical && !*old &&
               (selected_astronaut_==static_cast<int>(i) || nearby(actor.position,12.0f))) alarm(.39f);
        }
    }

    const auto lander_phase=static_cast<std::uint64_t>(lander_.phase);
    const auto last_lander=audio.observe(key(Global,10),lander_phase);
    if(lander_.active() && !lander_.awaiting_site && !lander_.awaiting_stash){
        if(lander_.in_flight() && !paused){
            const auto flight=lunar_lander_flight(lander_.elapsed);
            const float power=std::clamp(.64f+flight.thrust_acceleration*.12f,.6f,1.0f);
            audio.loop(key(Ambient,5),Cue::Exhaust,
                world_voice(lander_.site+flight.offset,power,420,.82f+.18f*power,5));
        }
        if(last_lander && *last_lander==static_cast<std::uint64_t>(LanderPhase::Approach) && !lander_.in_flight())
            audio.play(Cue::Touchdown,world_voice(lander_.site,.98f,120,.91f,5));
        if(last_lander && *last_lander!=lander_phase && lander_.phase==LanderPhase::OpeningRamp)
            audio.play(Cue::Hatch,world_voice(lander_.site,.74f,62,.64f,3));
        if(!paused && (lander_.phase==LanderPhase::OpeningRamp || lander_.phase==LanderPhase::UnloadingCargo))
            audio.loop(key(Ambient,6),Cue::Pump,world_voice(lander_.site,.43f,55,.76f,2));
        const bool salvaging=lander_.salvage_requested && lander_.salvage_seconds>0.0f;
        if(!paused && salvaging) audio.loop(key(Ambient,7),Cue::Weld,world_voice(lander_.site,.51f,43,.88f,2));
    }

    const auto trade_phase=static_cast<std::uint64_t>(trader_.phase);
    const auto last_trade=audio.observe(key(Global,11),trade_phase);
    const auto pad=trader_pad_index();
    if(trader_.active() && pad<placed_resources_.size()){
        const auto p=placed_resources_[pad].position;
        if(!paused && (trader_.phase==TradePhase::Approach || trader_.phase==TradePhase::Departing)){
            const float time=std::clamp(trader_.elapsed/(trader_.phase==TradePhase::Approach?24.0f:18.0f),0.0f,1.0f);
            const float travel=trader_.phase==TradePhase::Approach?1.0f-time:time;
            audio.loop(key(Ambient,8),Cue::Exhaust,world_voice(p+Vec3{0,75*travel*travel,0},.66f,122,.98f,4));
        }
        if(last_trade && *last_trade!=trade_phase && trader_.phase==TradePhase::Docked)
            audio.play(Cue::Latch,world_voice(p,.76f,77,.89f,4));
        const auto loaded=static_cast<std::uint64_t>(std::accumulate(trader_.loaded.begin(),trader_.loaded.end(),std::int64_t{}));
        if(const auto old=audio.observe(key(Global,12),loaded); old && loaded>*old)
            audio.play(Cue::Crate,world_voice(trader_.dock,.59f,52,1,3));
    }
    if(last_trade && *last_trade!=trade_phase && trader_.phase==TradePhase::Complete && trader_.last_payment_eur>0)
        ui_sound(Cue::Success,.40f);
    audio.finish(dt);
}

} // namespace game
