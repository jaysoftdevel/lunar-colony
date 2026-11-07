#include "game/game_app.hpp"
#include <algorithm>
#include <cmath>
#include <map>

namespace game {
namespace {
double scale(const PlacedResource& r){return r.size==ResourceSizeTier::Small?0.5:1.0;}
double habitat_scale(const PlacedHabitat& h){
    return h.size==HabitatSizeTier::Small?0.5:h.size==HabitatSizeTier::Mid?1.5:h.size==HabitatSizeTier::Big?2.0:1.0;
}
double battery_capacity(const PlacedResource& r){return power_storage_capacity_kwh(r.size);}
double solar_buffer_capacity(const PlacedResource& r){return solar_panel_buffer_capacity_kwh(r.size);}
double tank_capacity(const PlacedResource& r){return 2000.0*scale(r);}
double driller_buffer_capacity(const PlacedResource& r){return water_driller_buffer_capacity_litres(r.size);}
double hydrogen_buffer_capacity(const PlacedHabitat& h){return kOxygenGeneratorHydrogenBufferKg*habitat_scale(h);}
double hydrogen_generator_peak_kw(const PlacedResource& r){return kHydrogenGeneratorPeakKw*scale(r);}
double load_kw(const PlacedResource& r){
    switch(r.kind){
        case ResourceKind::WaterDriller:return 4.0*scale(r);
        case ResourceKind::OreDriller:return 8.0*scale(r);
        case ResourceKind::WaterPurifier:return 0.5*scale(r);
        case ResourceKind::RobotWorkshop:return 0.6;
        case ResourceKind::Antenna:return 0.15*scale(r);
        case ResourceKind::LandingPad:return 0.1*scale(r);
        case ResourceKind::StorageDome:return 0.2;
        // Fuel-cell output is already specified as net electrical power; its
        // balance-of-plant losses are included in kFuelCellElectricalEfficiency.
        case ResourceKind::HydrogenGenerator:return 0.0;
        default:return 0;
    }
}
double load_kw(const PlacedHabitat& h){
    switch(h.kind){
        case HabitatKind::OxygenGeneratorDome:return 2.8*habitat_scale(h);
        case HabitatKind::ProcessingFacility:return 6;
        case HabitatKind::Factory:return 4;
        case HabitatKind::Manufactory:return 5;
        case HabitatKind::BioDome:return 1.5;
        case HabitatKind::Laboratory:return 1.2;
        case HabitatKind::SickBay:return 0.8;
        default:return 0.3;
    }
}
double path_length(const std::vector<eng::Vec3>& path){double n=0;for(std::size_t i=1;i<path.size();++i)n+=eng::length(path[i]-path[i-1]);return n;}
double hydroponics_power_kw(const PlacedHabitat& h,bool light_phase){
    if(h.kind!=HabitatKind::BioDome || !light_phase) return 0.0;
    double total=0.0;
    const std::size_t count=hydroponics_bay_count(h.size);
    for(std::size_t i=0;i<count;++i) total+=hydroponic_profile(h.hydroponics[i].plant).power_kw;
    return total;
}
double hydroponics_water_litres_per_day(const PlacedHabitat& h,bool light_phase){
    if(h.kind!=HabitatKind::BioDome || !light_phase) return 0.0;
    double total=0.0;
    const std::size_t count=hydroponics_bay_count(h.size);
    for(std::size_t i=0;i<count;++i) total+=hydroponic_profile(h.hydroponics[i].plant).water_litres_per_day;
    return total;
}
}

const UtilityServiceSnapshot& GameApp::update_supply_simulation(float dt){
    const auto& topology=utility_network_.evaluate_services(corridor_network_,placed_habitats_,placed_resources_);
    auto& services=live_supply_services_;
    services=topology;
    const auto sun=status_.sun_pose();
    const bool hydroponics_light_phase=hydroponics_lights_on(status_.hour());
    struct Component {std::vector<std::size_t> tanks,batteries,solars,drillers,hydrogen_generators,hydrogen_tanks,oxygen_tanks,workshops,habitats,resources;double solar_kw=0,load_kw=0,power=0;};
    std::map<std::size_t,Component> groups;
    // Charging is a real network load. Clear transient telemetry before this
    // dispatch pass; powered workshop bays set it again only when energy is
    // actually transferred into a rover pack.
    for(auto& robot:builder_robots_) robot.battery_charging=false;
    const double tick_hours=std::max(0.0f,dt)/10.0;
    const auto workshop_charge_request_kw=[&](std::size_t resource_index){
        double kw=0.0;
        for(const auto& robot:builder_robots_){
            if(!robot.workshop_parked || robot.workshop_index!=resource_index) continue;
            const double deficit=std::max(0.0,(double)robot.battery_capacity_kwh-(double)robot.battery_charge_kwh);
            if(deficit<=1e-6) continue;
            const double input_needed=deficit/(double)kRobotWorkshopChargeEfficiency;
            const double cap=tick_hours>1e-9?input_needed/tick_hours:(double)kRobotWorkshopChargePowerKw;
            kw+=std::min((double)kRobotWorkshopChargePowerKw,cap);
        }
        return kw;
    };

    std::vector<SupplyState*> nodes;
    std::vector<eng::Vec3> positions;
    std::vector<double> runs,initial_oxygen;
    const auto node_count=topology.node_components.size();
    nodes.reserve(node_count);
    positions.reserve(node_count);
    runs.reserve(node_count);
    initial_oxygen.reserve(node_count);
    const auto add_node=[&](SupplyState& s,eng::Vec3 p,double volume,double run){
        s.volume_m3=volume;s.warnings=0;s.power_fraction=0;
        s.oxygen_kg=std::clamp(s.oxygen_kg,0.0,volume*kOxygenKgPerM3);
        s.oxygen_produced_kg_per_day=0;s.water_processed_litres_per_day=0;
        s.hydrogen_produced_kg_per_day=0;s.hydrogen_vented_kg_per_day=0;
        s.hydrogen_consumed_kg_per_day=0;s.hydrogen_power_kw=0;
        s.oxygen_net_kg_per_day=0;s.air_change_percentage_points_per_day=0;
        s.power_demand_kw=0;s.water_demand_litres_per_day=0;s.oxygen_demand_kg_per_day=0;
        nodes.push_back(&s);positions.push_back(p);runs.push_back(run);initial_oxygen.push_back(s.oxygen_kg);
    };
    for(std::size_t i=0;i<placed_habitats_.size();++i){
        auto& h=placed_habitats_[i];
        const bool complete=h.construction.complete();
        double volume=0.0;
        if(complete){
            const auto& m=habitat_models_.mesh(h);
            // Interior envelope reduced for the shell, equipment and curved roof.
            volume=m.footprint_x*m.footprint_z*std::max(0.0f,m.height-m.walkable_floor_y)*0.65;
        }
        add_node(h.supply,h.position,volume,0);
        if(complete){
            if(h.kind==HabitatKind::OxygenGeneratorDome)
                h.supply.hydrogen_kg=std::clamp(h.supply.hydrogen_kg,0.0,hydrogen_buffer_capacity(h));
            if(h.power_enabled){
                h.supply.power_demand_kw=load_kw(h)+hydroponics_power_kw(h,hydroponics_light_phase);
                h.supply.water_demand_litres_per_day=hydroponics_water_litres_per_day(h,hydroponics_light_phase);
            }
            auto& g=groups[services.habitats[i].connected?services.habitats[i].component:2000000+i];g.habitats.push_back(i);g.load_kw+=h.supply.power_demand_kw;
        }
    }
    for(std::size_t i=0;i<placed_resources_.size();++i){
        auto& r=placed_resources_[i];
        if(r.kind==ResourceKind::SolarPanel){
            const eng::Vec3 fixed=solar_tracking::fixed_panel_normal(r.rotation_degrees);
            const eng::Vec3 target=(r.solar_tracking_enabled && sun.above_horizon)?sun.direction:fixed;
            if(!r.solar_visual_initialized || eng::length(r.solar_visual_normal)<0.5f){
                r.solar_visual_normal=fixed;
                r.solar_visual_initialized=true;
            }
            // Critically damped-looking slew: never snap to a new sun position,
            // even when game speed changes or tracking is toggled.
            const float alpha=1.0f-std::exp(-0.8f*std::max(0.0f,dt));
            if(r.power_enabled && alpha>0.0f){
                r.solar_visual_normal=eng::normalize(r.solar_visual_normal*(1.0f-alpha)+eng::normalize(target)*alpha);
            }
        }
        // External equipment has a small sealed service manifold: its ports
        // relay air between pipes without treating the exterior as a cabin.
        const bool complete=r.construction.complete();
        double volume=complete?0.02:0.0;
        if(complete && r.kind==ResourceKind::StorageDome){
            const auto& m=resource_models_.mesh(r);
            volume=m.footprint_x*m.footprint_z*std::max(0.0f,m.height-m.walkable_floor_y)*0.65;
        }
        add_node(r.supply,r.position,volume,0);
        if(!complete)continue;
        // A standalone battery/tank/solar plant is its own component, never a global pool.
        const auto key=services.resources[i].connected?services.resources[i].component:nodes.size()+1000000;
        auto& g=groups[key];
        g.resources.push_back(i);
        if(!r.power_enabled) continue;
        r.supply.power_demand_kw=load_kw(r);
        if(r.kind==ResourceKind::RobotWorkshop){
            g.workshops.push_back(i);
            r.supply.power_demand_kw+=workshop_charge_request_kw(i);
        }
        g.load_kw+=r.supply.power_demand_kw;
        if(r.kind==ResourceKind::PowerStorage){r.supply.energy_kwh=std::clamp(r.supply.energy_kwh,0.0,battery_capacity(r));g.batteries.push_back(i);}
        if(r.kind==ResourceKind::SolarPanel){r.supply.energy_kwh=std::clamp(r.supply.energy_kwh,0.0,solar_buffer_capacity(r));g.solars.push_back(i);}
        if(r.kind==ResourceKind::WaterStorage){r.supply.water_litres=std::clamp(r.supply.water_litres,0.0,tank_capacity(r));g.tanks.push_back(i);}
        if(r.kind==ResourceKind::WaterDriller){r.supply.water_litres=std::clamp(r.supply.water_litres,0.0,driller_buffer_capacity(r));g.drillers.push_back(i);}
        if(r.kind==ResourceKind::HydrogenGenerator)g.hydrogen_generators.push_back(i);
        if(r.kind==ResourceKind::OxygenTank){
            r.stored_oxygen_kg=std::clamp(r.stored_oxygen_kg,0.0,oxygen_tank_capacity_kg(r.size));
            g.oxygen_tanks.push_back(i);
        }
        if(r.kind==ResourceKind::HydrogenTank){
            r.supply.hydrogen_kg=std::clamp(r.supply.hydrogen_kg,0.0,hydrogen_tank_capacity_kg(r.size));
            g.hydrogen_tanks.push_back(i);
        }
        if(r.kind==ResourceKind::LandingPad){
            r.rocket_fuel_kg=std::clamp(r.rocket_fuel_kg,0.0,landing_pad_rocket_fuel_capacity_kg());
            r.lander_fuel_kg=std::clamp(r.lander_fuel_kg,0.0,landing_pad_lander_fuel_capacity_kg());
        }
        if(r.kind==ResourceKind::SolarPanel){
            const double incidence=r.solar_tracking_enabled?1.0:solar_tracking::fixed_panel_incidence_fraction(sun.direction,r.rotation_degrees);
            g.solar_kw+=kSolarPeakKw*scale(r)*sun.irradiance_fraction*incidence;
        }
    }
    for(auto& u:utility_network_.mutable_connections()){
        const bool active=u.kind==UtilityKind::SupplyChannel && u.construction.complete();
        const double len=active?path_length(u.path):0.0;
        add_node(u.supply,u.path.empty()?eng::Vec3{}:u.path[u.path.size()/2],active?std::max(0.02,len*0.005):0.0,len);
    }
    for(auto& c:corridor_network_.mutable_connections()){
        const bool complete=c.construction.complete();
        const double len=complete?path_length(c.path):0.0;
        add_node(c.supply,c.path.empty()?eng::Vec3{}:c.path[c.path.size()/2],complete?std::max(0.1,len*4.0):0.0,len);
    }
    double initial_energy=0,initial_water=0,consumed_water=0,demanded_energy=0,supplied_energy=0;
    for(const auto& r:placed_resources_)if(r.construction.complete()){
        if(r.kind==ResourceKind::PowerStorage)initial_energy+=r.supply.energy_kwh;
        if(r.kind==ResourceKind::WaterStorage)initial_water+=r.supply.water_litres;
    }
    const auto water_total=[&](const Component& g){
        double v=0;
        for(auto i:g.tanks)v+=placed_resources_[i].supply.water_litres;
        for(auto i:g.drillers)v+=placed_resources_[i].supply.water_litres;
        return v;
    };
    const auto take_water=[&](Component& g,double wanted){
        double remaining=wanted;
        // Dedicated tanks are the primary network store. Driller buffers are a
        // local reserve and are consumed only after tank inventory is exhausted.
        for(auto i:g.tanks){auto& v=placed_resources_[i].supply.water_litres;const double q=std::min(v,remaining);v-=q;remaining-=q;if(remaining<=1e-12)break;}
        if(remaining>1e-12)for(auto i:g.drillers){auto& v=placed_resources_[i].supply.water_litres;const double q=std::min(v,remaining);v-=q;remaining-=q;if(remaining<=1e-12)break;}
        consumed_water+=wanted-remaining;return wanted-remaining;
    };
    const auto hydrogen_total=[&](const Component& g){
        double v=0;
        for(auto i:g.habitats)if(placed_habitats_[i].kind==HabitatKind::OxygenGeneratorDome && placed_habitats_[i].power_enabled)v+=placed_habitats_[i].supply.hydrogen_kg;
        for(auto i:g.hydrogen_tanks)v+=placed_resources_[i].supply.hydrogen_kg;
        return v;
    };
    const auto take_hydrogen=[&](Component& g,double wanted){
        double remaining=wanted;
        for(auto i:g.habitats){
            auto& h=placed_habitats_[i];if(h.kind!=HabitatKind::OxygenGeneratorDome || !h.power_enabled)continue;
            const double q=std::min(h.supply.hydrogen_kg,remaining);h.supply.hydrogen_kg-=q;remaining-=q;if(remaining<=1e-12)break;
        }
        if(remaining>1e-12)for(auto i:g.hydrogen_tanks){
            auto& stored=placed_resources_[i].supply.hydrogen_kg;
            const double q=std::min(stored,remaining);stored-=q;remaining-=q;
            if(remaining<=1e-12)break;
        }
        return wanted-remaining;
    };
    const auto store_in_hydrogen_tanks=[&](Component& g,double wanted){
        double remaining=wanted;
        for(auto i:g.hydrogen_tanks){
            auto& tank=placed_resources_[i];
            const double q=std::min(remaining,std::max(0.0,hydrogen_tank_capacity_kg(tank.size)-tank.supply.hydrogen_kg));
            tank.supply.hydrogen_kg+=q;remaining-=q;
            if(remaining<=1e-12)break;
        }
        return wanted-remaining;
    };
    // A zero-time refresh restores volume/percentages and input availability
    // immediately after loading a paused save, without processing any resources.
    for(auto& [key,g]:groups){
        (void)key;double available_kw=g.solar_kw;
        if(hydrogen_total(g)>1e-9)for(auto i:g.hydrogen_generators)available_kw+=hydrogen_generator_peak_kw(placed_resources_[i]);
        for(auto i:g.solars)if(placed_resources_[i].supply.energy_kwh>0)available_kw+=kSolarPeakKw*scale(placed_resources_[i]);
        for(auto i:g.batteries)if(placed_resources_[i].supply.energy_kwh>0)available_kw+=10.0*scale(placed_resources_[i]);
        g.power=g.load_kw>0?std::clamp(available_kw/g.load_kw,0.0,1.0):1.0;
    }
    std::size_t occupied=0;for(auto& h:placed_habitats_)if(h.construction.complete() && h.kind!=HabitatKind::OxygenGeneratorDome)++occupied;
    const double crew_per_occupied=occupied && !astronaut_test_active()?(double)std::max(0,status_.crew_current())/(double)occupied:0.0;
    for(auto& h:placed_habitats_)if(h.construction.complete() && h.kind!=HabitatKind::OxygenGeneratorDome){
        h.supply.oxygen_demand_kg_per_day+=crew_per_occupied*kCrewOxygenKgPerDay;
        h.supply.water_demand_litres_per_day+=crew_per_occupied*kCrewWaterLitresPerDay;
    }
    // Every pressurized volume has a small continuous seal/hatch leakage loss.
    // The loss is proportional to the oxygen actually present, so an empty
    // structure does not consume a full-pressure leak budget while filling.
    for(auto* s:nodes)if(s->volume_m3>0.05 && s->breathability()>=99.0)s->oxygen_demand_kg_per_day+=s->oxygen_kg*kPressurizedOxygenLeakFractionPerDay;
    const auto root=[&](std::size_t i){return topology.node_components[i];};
    struct CrewIntake {SupplyState* air{};Component* service{};bool at_cache{};};
    std::array<CrewIntake,kAstronautRoles.size()> intake{};
    if(astronaut_test_active()){
        std::vector<Component*> service_by_node(nodes.size());
        for(auto& [key,g]:groups){
            (void)key;
            for(auto i:g.habitats)service_by_node[root(i)]=&g;
            for(auto i:g.resources)service_by_node[root(placed_habitats_.size()+i)]=&g;
        }
        update_astronaut_suits();
        for(std::size_t i=0;i<test_astronauts_.size();++i){
            const auto& actor=test_astronauts_[i];
            bool in_lock=false;
            const auto* air=astronaut_air_at(actor.position,in_lock);
            const auto it=std::find(nodes.begin(),nodes.end(),air);
            if(it!=nodes.end() && !in_lock){
                intake[i].air=*it;
                intake[i].service=service_by_node[root((std::size_t)(it-nodes.begin()))];
                if(!actor.suited && actor.health>0.f)(*it)->oxygen_demand_kg_per_day+=kCrewOxygenKgPerDay;
                if(actor.health>0.f)(*it)->water_demand_litres_per_day+=kCrewWaterLitresPerDay;
            }
            intake[i].at_cache=!air && std::hypot(actor.position.x,actor.position.z)<=kCrewCacheRadius;
        }
    }
    const auto take=[](double& stock,double wanted){
        const double delivered=std::min(stock,wanted);
        stock-=delivered;
        return delivered;
    };
    // Substeps use actual game time and simultaneous, donor/receiver-limited air flux.
    // No frame-rate-dependent overdraw or same-tick propagation through an entire chain.
    struct Flux {std::size_t a,b;double mass;};
    std::vector<Flux> fluxes;
    std::vector<double> out,in,delta;
    double remaining=std::max(0.0f,dt);
    if(remaining>0.0){
        fluxes.reserve(topology.links.size());
        out.reserve(node_count);
        in.reserve(node_count);
        delta.reserve(node_count);
    }
    while(remaining>1e-8){
        const double step=std::min(remaining,0.1);remaining-=step;const double hours=step/10.0,days=step/240.0;
        for(auto& [key,g]:groups){
            (void)key;
            // Collect any buffered electrolysis by-product into connected tanks
            // before serving H2 electricity demand. Never move hydrogen between
            // disconnected components and never create hydrogen in this transfer.
            if(!g.hydrogen_tanks.empty())for(auto i:g.habitats){
                auto& h=placed_habitats_[i];
                if(h.kind!=HabitatKind::OxygenGeneratorDome || !h.power_enabled)continue;
                h.supply.hydrogen_kg-=store_in_hydrogen_tanks(g,h.supply.hydrogen_kg);
            }
            double generation=g.solar_kw*hours, demand=g.load_kw*hours;
            demanded_energy+=demand;

            // Dispatch fuel cells only for useful work: current component load
            // plus whatever connected batteries can accept during this step.
            // This preserves hydrogen when there is neither demand nor storage.
            double charge_input_room=0;
            for(auto i:g.batteries){
                const auto& r=placed_resources_[i];
                charge_input_room+=std::max(0.0,std::min((battery_capacity(r)-r.supply.energy_kwh)/0.95,10.0*scale(r)*hours));
            }
            // Local solar buffers bridge short production/storage gaps. They
            // discharge before hydrogen or the main battery, at no more than the
            // collector's rated peak output.
            double requested_buffer_energy=std::max(0.0,demand-generation);
            for(auto i:g.solars){
                if(requested_buffer_energy<=1e-12) break;
                auto& panel=placed_resources_[i];
                const double q=std::min({panel.supply.energy_kwh,requested_buffer_energy,kSolarPeakKw*scale(panel)*hours});
                panel.supply.energy_kwh-=q;generation+=q;requested_buffer_energy-=q;
            }

            double requested_h2_energy=std::max(0.0,demand+charge_input_room-generation);
            for(auto i:g.hydrogen_generators){
                if(requested_h2_energy<=1e-12)break;
                auto& generator=placed_resources_[i];
                const double max_energy=hydrogen_generator_peak_kw(generator)*hours;
                const double requested_energy=std::min(requested_h2_energy,max_energy);
                const double consumed=take_hydrogen(g,requested_energy/kHydrogenElectricalKwhPerKg);
                const double produced=consumed*kHydrogenElectricalKwhPerKg;
                generator.supply.hydrogen_consumed_kg_per_day+=consumed;
                generator.supply.hydrogen_power_kw+=produced; // kWh accumulator; normalized below
                generation+=produced;
                requested_h2_energy-=produced;
            }

            double supplied=std::min(generation,demand);generation-=supplied;
            for(auto i:g.batteries){auto& r=placed_resources_[i];const double q=std::min({r.supply.energy_kwh,demand-supplied,10.0*scale(r)*hours});r.supply.energy_kwh-=q;supplied+=q;}
            supplied_energy+=supplied;
            g.power=demand>0?std::clamp(supplied/demand,0.0,1.0):1.0;

            // Workshop chargers are ordinary consumers inside the dispatched
            // component. They receive the same brown-out fraction as every
            // other load and transfer only 92% of input energy into the pack.
            for(auto workshop_index:g.workshops){
                for(auto& robot:builder_robots_){
                    if(!robot.workshop_parked || robot.workshop_index!=workshop_index) continue;
                    const double deficit=std::max(0.0,(double)robot.battery_capacity_kwh-(double)robot.battery_charge_kwh);
                    if(deficit<=1e-9) continue;
                    const double delivered=std::min(deficit,(double)kRobotWorkshopChargePowerKw*hours*g.power*(double)kRobotWorkshopChargeEfficiency);
                    if(delivered>1e-10){
                        robot.battery_charge_kwh=(float)std::min((double)robot.battery_capacity_kwh,(double)robot.battery_charge_kwh+delivered);
                        robot.battery_charging=true;
                    }
                }
            }
            for(auto i:g.batteries){auto& r=placed_resources_[i];const double q=std::min({generation,(battery_capacity(r)-r.supply.energy_kwh)/0.95,10.0*scale(r)*hours});r.supply.energy_kwh+=q*0.95;generation-=q;}
            // Any otherwise-curtailed solar energy remains at the collector in
            // its small local buffer. External batteries still have priority.
            for(auto i:g.solars){
                if(generation<=1e-12) break;
                auto& panel=placed_resources_[i];
                const double q=std::min(generation,solar_buffer_capacity(panel)-panel.supply.energy_kwh);
                panel.supply.energy_kwh+=q;generation-=q;
            }
            for(auto i:g.drillers){
                auto& driller=placed_resources_[i];
                // If a tank becomes available after the driller was isolated,
                // automatically flush the local buffer into network storage.
                double buffered=driller.supply.water_litres;
                for(auto j:g.tanks){
                    if(buffered<=1e-12) break;
                    auto& tank=placed_resources_[j];
                    const double q=std::min(buffered,tank_capacity(tank)-tank.supply.water_litres);
                    tank.supply.water_litres+=q;buffered-=q;
                }
                driller.supply.water_litres=buffered;

                const double produced=500.0*scale(driller)*days*g.power;
                double left=produced;
                for(auto j:g.tanks){auto& tank=placed_resources_[j];const double q=std::min(left,tank_capacity(tank)-tank.supply.water_litres);tank.supply.water_litres+=q;left-=q;if(left<=1e-12)break;}
                if(left>1e-12){const double q=std::min(left,driller_buffer_capacity(driller)-driller.supply.water_litres);driller.supply.water_litres+=q;}
            }
            // Regulate reserve oxygen into low-pressure rooms in this component.
            // Tank mass is never part of the ordinary room/manifold airflow.
            for(auto i:g.oxygen_tanks){
                auto& tank=placed_resources_[i];
                double available=std::min(tank.stored_oxygen_kg,12.0*hours*scale(tank));
                const auto feed=[&](SupplyState& room){
                    const double deficit=.95*room.volume_m3*kOxygenKgPerM3-room.oxygen_kg;
                    const double transferred=std::min(available,std::max(0.0,deficit));
                    room.oxygen_kg+=transferred;
                    tank.stored_oxygen_kg-=transferred;
                    available-=transferred;
                };
                for(auto j:g.habitats) feed(placed_habitats_[j].supply);
                for(auto j:g.resources) if(placed_resources_[j].kind==ResourceKind::StorageDome)
                    feed(placed_resources_[j].supply);
            }
            const bool bottle_export=active_game_phase_==GamePhase::RobotPreparation&&trader_.oxygen_packaging;
            bool export_air_ready=bottle_export||!g.oxygen_tanks.empty();
            if(export_air_ready){
                for(auto hi:g.habitats)if(placed_habitats_[hi].supply.breathability()<99.0)export_air_ready=false;
                for(auto si:g.resources)if(placed_resources_[si].supply.volume_m3>.05&&placed_resources_[si].supply.breathability()<99.0)export_air_ready=false;
            }
            for(auto i:g.habitats){auto& h=placed_habitats_[i];
                if(h.kind==HabitatKind::OxygenGeneratorDome){
                    if(!h.power_enabled)continue;
                    const double deficit=h.supply.volume_m3*kOxygenKgPerM3-h.supply.oxygen_kg;
                    const double capacity=18.0*habitat_scale(h);
                    // Once a trader has been called, bottle genuine surplus
                    // electrolysis output separately from breathable cabin air.
                    // Filling the entire connected atmosphere takes priority.
                    double export_room=0;
                    if(export_air_ready){
                        for(auto j:g.oxygen_tanks){
                            const auto& tank=placed_resources_[j];
                            export_room+=std::max(0.0,oxygen_tank_capacity_kg(tank.size)-tank.stored_oxygen_kg);
                        }
                        if(bottle_export&&h.placement_sequence)
                            export_room+=std::max(0.0,500.0*habitat_scale(h)-trader_oxygen_stock_amount(h.placement_sequence));
                    }
                    const double oxygen=std::min({deficit+export_room,capacity*days*g.power,water_total(g)/kWaterPerOxygen});
                    const double processed_water=take_water(g,oxygen*kWaterPerOxygen);
                    const double produced_oxygen=processed_water/kWaterPerOxygen;
                    const double cabin=std::min(produced_oxygen,std::max(0.0,deficit));
                    h.supply.oxygen_kg+=cabin;
                    double surplus=produced_oxygen-cabin;
                    for(auto j:g.oxygen_tanks){
                        auto& tank=placed_resources_[j];
                        const double stored=std::min(surplus,std::max(0.0,oxygen_tank_capacity_kg(tank.size)-tank.stored_oxygen_kg));
                        tank.stored_oxygen_kg+=stored;
                        surplus-=stored;
                    }
                    if(surplus>0&&bottle_export&&h.placement_sequence)
                        trader_oxygen_stock(h.placement_sequence)+=surplus;
                    h.supply.water_processed_litres_per_day+=processed_water;
                    h.supply.oxygen_produced_kg_per_day+=produced_oxygen;
                    const double produced_hydrogen=produced_oxygen*kHydrogenPerOxygen;
                    const double sent_to_tank=store_in_hydrogen_tanks(g,produced_hydrogen);
                    const double local_buffer=std::min(produced_hydrogen-sent_to_tank,std::max(0.0,hydrogen_buffer_capacity(h)-h.supply.hydrogen_kg));
                    h.supply.hydrogen_kg+=local_buffer;
                    const double stored_hydrogen=sent_to_tank+local_buffer;
                    h.supply.hydrogen_produced_kg_per_day+=produced_hydrogen;
                    h.supply.hydrogen_vented_kg_per_day+=produced_hydrogen-stored_hydrogen;
                }else if(occupied){
                    h.supply.oxygen_kg=std::max(0.0,h.supply.oxygen_kg-crew_per_occupied*kCrewOxygenKgPerDay*days);
                    const double need=crew_per_occupied*kCrewWaterLitresPerDay*days;
                    if(take_water(g,need)+1e-9<need)h.supply.warnings|=NoWater;
                }

                // Hydroponics are independent bays. Grow lights/pumps are part
                // of the component electrical demand above; growth advances only
                // when the dome has usable air, power and water. Mature crops are
                // harvested automatically into the existing physical Food economy.
                if(h.kind==HabitatKind::BioDome && h.power_enabled){
                    const double air_factor=h.supply.breathability()>=95.0?1.0:(h.supply.breathability()>=80.0?0.35:0.0);
                    const std::size_t bay_count=hydroponics_bay_count(h.size);
                    for(std::size_t bay_index=0;bay_index<bay_count;++bay_index){
                        auto& bay=h.hydroponics[bay_index];
                        const auto& crop=hydroponic_profile(bay.plant);
                        if(bay.plant==HydroponicPlant::Empty || crop.harvest_food_kg<=0 || !hydroponics_light_phase || air_factor<=0.0 || g.power<=1e-9) continue;
                        const double water_need=crop.water_litres_per_day*days*g.power*air_factor;
                        const double water_received=water_need>1e-12?take_water(g,water_need):0.0;
                        const double water_fraction=water_need>1e-12?std::clamp(water_received/water_need,0.0,1.0):1.0;
                        if(water_fraction<0.999) h.supply.warnings|=NoWater;
                        const double growth_step=(days/std::max(0.1,crop.growth_days))*g.power*air_factor*water_fraction;
                        bay.growth_fraction=std::clamp(bay.growth_fraction+growth_step,0.0,2.0);
                        while(bay.growth_fraction>=1.0){
                            bay.growth_fraction-=1.0;
                            ++bay.harvests;
                            spawn_produced_resource(MaterialResource::Food,crop.harvest_food_kg,h.position);
                        }
                    }
                }
            }
        }
        if(astronaut_test_active())for(std::size_t i=0;i<test_astronauts_.size();++i){
            auto& actor=test_astronauts_[i];
            if(actor.health<=0.f)continue;
            auto& source=intake[i];
            const double air_need=kCrewOxygenKgPerDay*days;
            const double water_need=kCrewWaterLitresPerDay*days;
            const double food_need=kCrewFoodKgPerDay*days;
            const bool breathable=source.air && source.air->breathability()>=kCrewBreathableAir;
            const bool powered=source.service && source.service->power>=.99;
            // Leave the cabin's last breathable margin in the room. Beyond
            // that limit the crew must breathe from their own finite tanks.
            const auto cabin_oxygen=[&](double wanted){
                const double minimum=source.air->volume_m3*kOxygenKgPerM3*(kCrewBreathableAir/100.0);
                const double supplied=std::min(wanted,std::max(0.0,source.air->oxygen_kg-minimum));
                source.air->oxygen_kg-=supplied;
                return supplied;
            };
            double air=breathable && !actor.suited?cabin_oxygen(air_need):0.0;
            if(source.at_cache){
                actor.suit_oxygen_kg+=take(astronaut_cache_.oxygen_kg,kSuitOxygenCapacityKg-actor.suit_oxygen_kg);
            }else if(breathable && powered){
                double refill=std::min(kSuitOxygenCapacityKg-actor.suit_oxygen_kg,4.0*air_need);
                for(auto tank:source.service->oxygen_tanks){
                    const double amount=take(placed_resources_[tank].stored_oxygen_kg,refill);
                    actor.suit_oxygen_kg+=amount;refill-=amount;
                }
                actor.suit_oxygen_kg+=cabin_oxygen(refill);
            }
            air+=take(actor.suit_oxygen_kg,std::max(0.0,air_need-air));

            double water=powered && source.air?take_water(*source.service,water_need):0.0;
            if(source.at_cache)water+=take(astronaut_cache_.water_litres,water_need-water);
            if(source.air && actor.food_pack_kg<food_need && consume_colony_stock(MaterialResource::Food,1))
                actor.food_pack_kg+=1.0;
            double food=take(actor.food_pack_kg,food_need);
            if(source.at_cache)food+=take(astronaut_cache_.food_kg,food_need-food);
            actor.hunger=std::clamp(actor.hunger+(float)(days*(1.0-2.0*food/food_need)),0.f,1.f);
            actor.thirst=std::clamp(actor.thirst+(float)(2.0*days*(1.0-2.0*water/water_need)),0.f,1.f);
            float fatigue_rate=1.2f;
            if(actor.activity==AstronautActivity::Sleeping)fatigue_rate=-3.6f;
            if(actor.activity==AstronautActivity::Knockout)fatigue_rate=-1.8f;
            actor.fatigue=std::clamp(actor.fatigue+fatigue_rate*(float)days,0.f,kCrewExhausted);
            const double damage=48.0*std::max(0.0,1.0-air/air_need) +
                (actor.thirst>=1.f?1.0:0.0)+(actor.hunger>=1.f?.35:0.0);
            actor.health=std::max(0.f,actor.health-(float)(damage*days));
        }
        for(auto* s:nodes){
            if(s->volume_m3<=0.05 || s->oxygen_kg<=0 || s->breathability()<99.0)continue;
            const double leaked=s->oxygen_kg*kPressurizedOxygenLeakFractionPerDay*days;
            s->oxygen_kg=std::max(0.0,s->oxygen_kg-leaked);
        }
        fluxes.clear();
        out.assign(node_count,0.0);
        in.assign(node_count,0.0);
        for(const auto& [a,b]:topology.links){
            if(a>=nodes.size() || b>=nodes.size() || nodes[a]->volume_m3<=0 || nodes[b]->volume_m3<=0)continue;
            const double difference=nodes[a]->breathability()/100.0-nodes[b]->breathability()/100.0;
            const std::size_t from=difference>=0?a:b,to=difference>=0?b:a;
            const double route=std::max((double)eng::length(positions[a]-positions[b]),(runs[a]+runs[b])*0.5);
            const double mass=std::abs(difference)*12.0*hours/(1.0+route/10.0);
            fluxes.push_back({from,to,mass});out[from]+=mass;in[to]+=mass;
        }
        delta.assign(node_count,0.0);
        for(const auto& f:fluxes){
            const double space=nodes[f.b]->volume_m3*kOxygenKgPerM3-nodes[f.b]->oxygen_kg;
            const double factor=std::min({1.0,out[f.a]>0?nodes[f.a]->oxygen_kg/out[f.a]:1.0,in[f.b]>0?space/in[f.b]:1.0});
            const double mass=f.mass*std::max(0.0,factor);delta[f.a]-=mass;delta[f.b]+=mass;
        }
        for(std::size_t i=0;i<nodes.size();++i)nodes[i]->oxygen_kg+=delta[i];
    }
    double stored_power=0,power_capacity=0,stored_water=0,water_capacity=0;
    double instantaneous_load=0,instantaneous_delivered=0;
    for(auto& [key,g]:groups){
        (void)key;const bool water=water_total(g)>1e-6;
        instantaneous_load+=g.load_kw;
        instantaneous_delivered+=g.load_kw*g.power;
        for(auto i:g.habitats){auto& h=placed_habitats_[i];
            if(!h.power_enabled)continue;
            h.supply.power_fraction=g.power;
            services.habitats[i].power=g.power>=0.99;services.habitats[i].water=water;
            if(g.power<0.99)h.supply.warnings|=NoPower;
            if(!water)h.supply.warnings|=NoWater;
            if(h.supply.breathability()<95)h.supply.warnings|=LowAir;
            if((h.kind==HabitatKind::CantineDome || h.kind==HabitatKind::CrewQuarters) && resources_.amounts().food<std::max(1.0,status_.crew_current()*0.85))h.supply.warnings|=NoFood;
            if(h.kind==HabitatKind::SickBay && resources_.amounts().medical<std::max(1.0,status_.crew_current()*0.03))h.supply.warnings|=NoMedical;
        }
        for(auto i:g.resources){auto& r=placed_resources_[i];
            if(r.kind==ResourceKind::PowerStorage){stored_power+=r.supply.energy_kwh;power_capacity+=battery_capacity(r);}
            if(r.kind==ResourceKind::WaterStorage){stored_water+=r.supply.water_litres;water_capacity+=tank_capacity(r);}
            if(!r.power_enabled)continue;
            r.supply.power_fraction=g.power;
            services.resources[i].power=g.power>=0.99;services.resources[i].water=water;
            if(load_kw(r)>0 && g.power<0.99)r.supply.warnings|=NoPower;
            if(r.kind==ResourceKind::WaterPurifier && !water)r.supply.warnings|=NoWater;
            if(r.kind==ResourceKind::SolarPanel && g.batteries.empty() && r.supply.energy_kwh>=solar_buffer_capacity(r)-1e-6)r.supply.warnings|=NoStorage;
            if(r.kind==ResourceKind::WaterDriller && g.tanks.empty() && r.supply.water_litres>=driller_buffer_capacity(r)-1e-6)r.supply.warnings|=NoStorage;
            if(r.kind==ResourceKind::HydrogenGenerator && hydrogen_total(g)<=1e-9)r.supply.warnings|=NoHydrogen;
            if(r.kind==ResourceKind::PowerStorage && r.supply.energy_kwh<0.1*battery_capacity(r))r.supply.warnings|=LowStock;
            if(r.kind==ResourceKind::WaterStorage && r.supply.water_litres<0.1*tank_capacity(r))r.supply.warnings|=LowStock;
            if(r.kind==ResourceKind::OxygenTank&&r.stored_oxygen_kg<.10*oxygen_tank_capacity_kg(r.size))r.supply.warnings|=LowStock;
            if(r.kind==ResourceKind::HydrogenTank && r.supply.hydrogen_kg<0.10*hydrogen_tank_capacity_kg(r.size))r.supply.warnings|=LowStock;
            if(r.kind==ResourceKind::LandingPad && (r.rocket_fuel_kg<landing_pad_rocket_fuel_capacity_kg()-1e-5 || r.lander_fuel_kg<landing_pad_lander_fuel_capacity_kg()-1e-5) && hydrogen_total(g)<1e-8)r.supply.warnings|=NoHydrogen;
            if(r.kind==ResourceKind::StorageDome && r.supply.breathability()<95)r.supply.warnings|=LowAir;
        }
    }
    for(auto* s:nodes)s->water_to_fill_litres=std::max(0.0,s->volume_m3*kOxygenKgPerM3-s->oxygen_kg)*kWaterPerOxygen;
    std::vector<double> needed(nodes.size(),0);for(std::size_t i=0;i<nodes.size();++i)needed[root(i)]+=nodes[i]->water_to_fill_litres;
    for(std::size_t i=0;i<nodes.size();++i)nodes[i]->network_water_to_fill_litres=needed[root(i)];
    if(dt>0){
        const double per_day=240.0/(double)dt;
        for(std::size_t i=0;i<nodes.size();++i){auto& s=*nodes[i];
            s.oxygen_produced_kg_per_day*=per_day;s.water_processed_litres_per_day*=per_day;
            s.hydrogen_produced_kg_per_day*=per_day;s.hydrogen_vented_kg_per_day*=per_day;
            s.hydrogen_consumed_kg_per_day*=per_day;
            s.hydrogen_power_kw/=((double)dt/10.0);
            s.oxygen_net_kg_per_day=(s.oxygen_kg-initial_oxygen[i])*per_day;
            const double target=s.volume_m3*kOxygenKgPerM3;
            s.air_change_percentage_points_per_day=target>0?100.0*s.oxygen_net_kg_per_day/target:0;
        }
    }
    battery_net_kw_=dt>0?(stored_power-initial_energy)/((double)dt/10.0):0;
    water_net_litres_per_day_=dt>0?(stored_water-initial_water)/((double)dt/240.0):0;
    supply_power_delivered_kw_=dt>0?supplied_energy/((double)dt/10.0):instantaneous_delivered;
    supply_power_load_kw_=dt>0?demanded_energy/((double)dt/10.0):instantaneous_load;
    supply_water_load_litres_per_day_=dt>0?consumed_water/((double)dt/240.0):0;
    auto state=status_.state();state.power_current=(int)std::floor(stored_power);state.power_capacity=(int)power_capacity;
    state.water_current=(int)std::floor(stored_water);state.water_capacity=(int)water_capacity;
    if(astronaut_test_active())state.crew_current=(int)std::count_if(test_astronauts_.begin(),test_astronauts_.end(),
        [](const auto& actor){return actor.health>0.f;});
    state.crew_capacity=std::max(state.crew_current,crew_quarters_capacity());
    status_.restore(state);
    return services;
}
// Transfer hydrogen from a completed, connected storage tank to a vehicle
// serviced by a completed landing pad. No global hydrogen pool and no free fuel.
double GameApp::refuel_landing_pad(std::size_t pad_index,bool rocket){
    if(pad_index>=placed_resources_.size())return 0.0;
    auto& pad=placed_resources_[pad_index];
    if(pad.kind!=ResourceKind::LandingPad || !pad.construction.complete() || !pad.power_enabled)return 0.0;
    const auto& services=utility_network_.evaluate_services(corridor_network_,placed_habitats_,placed_resources_);
    if(pad_index>=services.resources.size() || !services.resources[pad_index].connected)return 0.0;
    // The fueling pump requires power as well as the H2 manifold.
    if(pad.supply.power_fraction<0.5)return 0.0;
    double& vehicle_fuel=rocket?pad.rocket_fuel_kg:pad.lander_fuel_kg;
    const double capacity=rocket?landing_pad_rocket_fuel_capacity_kg():landing_pad_lander_fuel_capacity_kg();
    double remaining=std::max(0.0,capacity-vehicle_fuel);
    const auto component=services.resources[pad_index].component;
    double added=0.0;
    for(std::size_t i=0;i<placed_resources_.size() && remaining>1e-9;++i){
        if(i>=services.resources.size() || !services.resources[i].connected || services.resources[i].component!=component)continue;
        auto& tank=placed_resources_[i];
        if(tank.kind!=ResourceKind::HydrogenTank || !tank.construction.complete() || !tank.power_enabled)continue;
        const double q=std::min({remaining,capacity-vehicle_fuel,std::max(0.0,tank.supply.hydrogen_kg)});
        tank.supply.hydrogen_kg-=q;vehicle_fuel+=q;remaining-=q;added+=q;
    }
    return added;
}

} // namespace game
