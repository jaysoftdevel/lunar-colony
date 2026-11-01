#include "game/game_app.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>

namespace game {
namespace {
double scale(const PlacedResource& r){return r.size==ResourceSizeTier::Small?0.5:1.0;}
double habitat_scale(const PlacedHabitat& h){
    return h.size==HabitatSizeTier::Small?0.5:h.size==HabitatSizeTier::Mid?1.5:h.size==HabitatSizeTier::Big?2.0:1.0;
}
double battery_capacity(const PlacedResource& r){return 60.0*scale(r);}
double tank_capacity(const PlacedResource& r){return 2000.0*scale(r);}
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
}

UtilityServiceSnapshot GameApp::update_supply_simulation(float dt){
    auto services=utility_network_.evaluate_services(corridor_network_,placed_habitats_,placed_resources_);
    struct Component {std::vector<std::size_t> tanks,batteries,drillers,hydrogen_generators,workshops,habitats,resources;double solar_kw=0,load_kw=0,power=0;};
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
        auto& h=placed_habitats_[i];const auto& m=habitat_models_.mesh(h);
        // Interior cubic envelope, reduced for pressure shell, equipment and curved roof.
        const double volume=h.construction.complete()?m.footprint_x*m.footprint_z*std::max(0.0f,m.height-m.walkable_floor_y)*0.65:0;
        add_node(h.supply,h.position,volume,0);
        if(h.construction.complete()){
            if(h.kind==HabitatKind::OxygenGeneratorDome)
                h.supply.hydrogen_kg=std::clamp(h.supply.hydrogen_kg,0.0,hydrogen_buffer_capacity(h));
            h.supply.power_demand_kw=load_kw(h);
            auto& g=groups[services.habitats[i].connected?services.habitats[i].component:2000000+i];g.habitats.push_back(i);g.load_kw+=h.supply.power_demand_kw;
        }
    }
    for(std::size_t i=0;i<placed_resources_.size();++i){
        auto& r=placed_resources_[i];const auto& m=resource_models_.mesh(r);
        // External equipment has a small sealed service manifold: its ports
        // relay air between pipes without treating the exterior as a cabin.
        add_node(r.supply,r.position,r.kind==ResourceKind::StorageDome && r.construction.complete()?m.footprint_x*m.footprint_z*std::max(0.0f,m.height-m.walkable_floor_y)*0.65:(r.construction.complete()?0.02:0),0);
        if(!r.construction.complete())continue;
        // A standalone battery/tank/solar plant is its own component, never a global pool.
        const auto key=services.resources[i].connected?services.resources[i].component:nodes.size()+1000000;
        auto& g=groups[key];
        r.supply.power_demand_kw=load_kw(r);
        if(r.kind==ResourceKind::RobotWorkshop){
            g.workshops.push_back(i);
            r.supply.power_demand_kw+=workshop_charge_request_kw(i);
        }
        g.resources.push_back(i);g.load_kw+=r.supply.power_demand_kw;
        if(r.kind==ResourceKind::PowerStorage){r.supply.energy_kwh=std::clamp(r.supply.energy_kwh,0.0,battery_capacity(r));g.batteries.push_back(i);}
        if(r.kind==ResourceKind::WaterStorage){r.supply.water_litres=std::clamp(r.supply.water_litres,0.0,tank_capacity(r));g.tanks.push_back(i);}
        if(r.kind==ResourceKind::WaterDriller)g.drillers.push_back(i);
        if(r.kind==ResourceKind::HydrogenGenerator)g.hydrogen_generators.push_back(i);
        if(r.kind==ResourceKind::SolarPanel){
            const auto sun=status_.sun_pose();
            g.solar_kw+=kSolarPeakKw*scale(r)*sun.irradiance_fraction;
        }
    }
    for(auto& u:utility_network_.mutable_connections()){
        const double len=path_length(u.path);add_node(u.supply,u.path.empty()?eng::Vec3{}:u.path[u.path.size()/2],u.construction.complete()?std::max(0.02,len*0.005):0,len);
    }
    for(auto& c:corridor_network_.mutable_connections()){
        const double len=path_length(c.path);add_node(c.supply,c.path.empty()?eng::Vec3{}:c.path[c.path.size()/2],c.construction.complete()?std::max(0.1,len*4.0):0,len);
    }
    double initial_energy=0,initial_water=0,consumed_water=0,demanded_energy=0;
    for(const auto& [key,g]:groups){(void)key;for(auto i:g.batteries)initial_energy+=placed_resources_[i].supply.energy_kwh;for(auto i:g.tanks)initial_water+=placed_resources_[i].supply.water_litres;}
    const auto water_total=[&](const Component& g){double v=0;for(auto i:g.tanks)v+=placed_resources_[i].supply.water_litres;return v;};
    const auto take_water=[&](Component& g,double wanted){
        double remaining=wanted;for(auto i:g.tanks){auto& v=placed_resources_[i].supply.water_litres;const double q=std::min(v,remaining);v-=q;remaining-=q;}consumed_water+=wanted-remaining;return wanted-remaining;
    };
    const auto hydrogen_total=[&](const Component& g){
        double v=0;for(auto i:g.habitats)if(placed_habitats_[i].kind==HabitatKind::OxygenGeneratorDome)v+=placed_habitats_[i].supply.hydrogen_kg;return v;
    };
    const auto take_hydrogen=[&](Component& g,double wanted){
        double remaining=wanted;
        for(auto i:g.habitats){
            auto& h=placed_habitats_[i];if(h.kind!=HabitatKind::OxygenGeneratorDome)continue;
            const double q=std::min(h.supply.hydrogen_kg,remaining);h.supply.hydrogen_kg-=q;remaining-=q;if(remaining<=1e-12)break;
        }
        return wanted-remaining;
    };
    // A zero-time refresh restores volume/percentages and input availability
    // immediately after loading a paused save, without processing any resources.
    for(auto& [key,g]:groups){
        (void)key;double available_kw=g.solar_kw;
        if(hydrogen_total(g)>1e-9)for(auto i:g.hydrogen_generators)available_kw+=hydrogen_generator_peak_kw(placed_resources_[i]);
        for(auto i:g.batteries)if(placed_resources_[i].supply.energy_kwh>0)available_kw+=10.0*scale(placed_resources_[i]);
        g.power=g.load_kw>0?std::clamp(available_kw/g.load_kw,0.0,1.0):1.0;
    }
    std::size_t occupied=0;for(auto& h:placed_habitats_)if(h.construction.complete() && h.kind!=HabitatKind::OxygenGeneratorDome)++occupied;
    const double crew_per_occupied=occupied?(double)std::max(0,status_.crew_current())/(double)occupied:0.0;
    for(auto& h:placed_habitats_)if(h.construction.complete() && h.kind!=HabitatKind::OxygenGeneratorDome){
        h.supply.oxygen_demand_kg_per_day+=crew_per_occupied*kCrewOxygenKgPerDay;
        h.supply.water_demand_litres_per_day+=crew_per_occupied*kCrewWaterLitresPerDay;
    }
    // Every pressurized volume has a small continuous seal/hatch leakage loss.
    // The loss is proportional to the oxygen actually present, so an empty
    // structure does not consume a full-pressure leak budget while filling.
    for(auto* s:nodes)if(s->volume_m3>0.05 && s->breathability()>=99.0)s->oxygen_demand_kg_per_day+=s->oxygen_kg*kPressurizedOxygenLeakFractionPerDay;
    // Substeps use actual game time and simultaneous, donor/receiver-limited air flux.
    // No frame-rate-dependent overdraw or same-tick propagation through an entire chain.
    double remaining=std::max(0.0f,dt);
    while(remaining>1e-8){
        const double step=std::min(remaining,0.1);remaining-=step;const double hours=step/10.0,days=step/240.0;
        for(auto& [key,g]:groups){
            (void)key;
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
            for(auto i:g.drillers){
                const double produced=500.0*scale(placed_resources_[i])*days*g.power;
                double left=produced;for(auto j:g.tanks){auto& tank=placed_resources_[j];const double q=std::min(left,tank_capacity(tank)-tank.supply.water_litres);tank.supply.water_litres+=q;left-=q;}
            }
            for(auto i:g.habitats){auto& h=placed_habitats_[i];
                if(h.kind==HabitatKind::OxygenGeneratorDome){
                    const double deficit=h.supply.volume_m3*kOxygenKgPerM3-h.supply.oxygen_kg;
                    const double capacity=9.0*habitat_scale(h);
                    const double oxygen=std::min({deficit,capacity*days*g.power,water_total(g)/kWaterPerOxygen});
                    const double processed_water=take_water(g,oxygen*kWaterPerOxygen);
                    const double produced_oxygen=processed_water/kWaterPerOxygen;
                    h.supply.oxygen_kg+=produced_oxygen;
                    h.supply.water_processed_litres_per_day+=processed_water;
                    h.supply.oxygen_produced_kg_per_day+=produced_oxygen;
                    const double produced_hydrogen=produced_oxygen*kHydrogenPerOxygen;
                    const double stored_hydrogen=std::min(produced_hydrogen,std::max(0.0,hydrogen_buffer_capacity(h)-h.supply.hydrogen_kg));
                    h.supply.hydrogen_kg+=stored_hydrogen;
                    h.supply.hydrogen_produced_kg_per_day+=produced_hydrogen;
                    h.supply.hydrogen_vented_kg_per_day+=produced_hydrogen-stored_hydrogen;
                }else if(occupied){
                    h.supply.oxygen_kg=std::max(0.0,h.supply.oxygen_kg-crew_per_occupied*kCrewOxygenKgPerDay*days);
                    const double need=crew_per_occupied*kCrewWaterLitresPerDay*days;
                    if(take_water(g,need)+1e-9<need)h.supply.warnings|=NoWater;
                }
            }
        }
        for(auto* s:nodes){
            if(s->volume_m3<=0.05 || s->oxygen_kg<=0 || s->breathability()<99.0)continue;
            const double leaked=s->oxygen_kg*kPressurizedOxygenLeakFractionPerDay*days;
            s->oxygen_kg=std::max(0.0,s->oxygen_kg-leaked);
        }
        struct Flux {std::size_t a,b;double mass;};std::vector<Flux> fluxes;
        std::vector<double> out(nodes.size(),0),in(nodes.size(),0);
        for(const auto& [a,b]:services.links){
            if(a>=nodes.size() || b>=nodes.size() || nodes[a]->volume_m3<=0 || nodes[b]->volume_m3<=0)continue;
            const double difference=nodes[a]->breathability()/100.0-nodes[b]->breathability()/100.0;
            const std::size_t from=difference>=0?a:b,to=difference>=0?b:a;
            const double route=std::max((double)eng::length(positions[a]-positions[b]),(runs[a]+runs[b])*0.5);
            const double mass=std::abs(difference)*12.0*hours/(1.0+route/10.0);
            fluxes.push_back({from,to,mass});out[from]+=mass;in[to]+=mass;
        }
        std::vector<double> delta(nodes.size(),0);
        for(const auto& f:fluxes){
            const double space=nodes[f.b]->volume_m3*kOxygenKgPerM3-nodes[f.b]->oxygen_kg;
            const double factor=std::min({1.0,out[f.a]>0?nodes[f.a]->oxygen_kg/out[f.a]:1.0,in[f.b]>0?space/in[f.b]:1.0});
            const double mass=f.mass*std::max(0.0,factor);delta[f.a]-=mass;delta[f.b]+=mass;
        }
        for(std::size_t i=0;i<nodes.size();++i)nodes[i]->oxygen_kg+=delta[i];
    }
    double stored_power=0,power_capacity=0,stored_water=0,water_capacity=0;
    for(auto& [key,g]:groups){
        (void)key;const bool water=water_total(g)>1e-6;
        for(auto i:g.habitats){auto& h=placed_habitats_[i];h.supply.power_fraction=g.power;
            services.habitats[i].power=g.power>=0.99;services.habitats[i].water=water;
            if(g.power<0.99)h.supply.warnings|=NoPower;
            if(!water)h.supply.warnings|=NoWater;
            if(h.supply.breathability()<95)h.supply.warnings|=LowAir;
            if((h.kind==HabitatKind::CantineDome || h.kind==HabitatKind::CrewQuarters) && resources_.amounts().food<std::max(1.0,status_.crew_current()*0.85))h.supply.warnings|=NoFood;
            if(h.kind==HabitatKind::SickBay && resources_.amounts().medical<std::max(1.0,status_.crew_current()*0.03))h.supply.warnings|=NoMedical;
        }
        for(auto i:g.resources){auto& r=placed_resources_[i];r.supply.power_fraction=g.power;
            services.resources[i].power=g.power>=0.99;services.resources[i].water=water;
            if(load_kw(r)>0 && g.power<0.99)r.supply.warnings|=NoPower;
            if(r.kind==ResourceKind::WaterPurifier && !water)r.supply.warnings|=NoWater;
            if(r.kind==ResourceKind::SolarPanel && g.batteries.empty())r.supply.warnings|=NoStorage;
            if(r.kind==ResourceKind::WaterDriller && g.tanks.empty())r.supply.warnings|=NoStorage;
            if(r.kind==ResourceKind::HydrogenGenerator && hydrogen_total(g)<=1e-9)r.supply.warnings|=NoHydrogen;
            if(r.kind==ResourceKind::PowerStorage){stored_power+=r.supply.energy_kwh;power_capacity+=battery_capacity(r);if(r.supply.energy_kwh<0.1*battery_capacity(r))r.supply.warnings|=LowStock;}
            if(r.kind==ResourceKind::WaterStorage){stored_water+=r.supply.water_litres;water_capacity+=tank_capacity(r);if(r.supply.water_litres<0.1*tank_capacity(r))r.supply.warnings|=LowStock;}
            if(r.kind==ResourceKind::StorageDome && r.supply.breathability()<95)r.supply.warnings|=LowAir;
        }
    }
    for(auto* s:nodes)s->water_to_fill_litres=std::max(0.0,s->volume_m3*kOxygenKgPerM3-s->oxygen_kg)*kWaterPerOxygen;
    std::vector<std::size_t> parent(nodes.size());std::iota(parent.begin(),parent.end(),0);
    const auto root=[&](std::size_t i){while(parent[i]!=i)i=parent[i];return i;};
    for(const auto& link:services.links)if(link.first<nodes.size() && link.second<nodes.size())parent[root(link.first)]=root(link.second);
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
    supply_power_load_kw_=dt>0?demanded_energy/((double)dt/10.0):0;
    supply_water_load_litres_per_day_=dt>0?consumed_water/((double)dt/240.0):0;
    auto state=status_.state();state.power_current=(int)std::floor(stored_power);state.power_capacity=(int)power_capacity;
    state.water_current=(int)std::floor(stored_water);state.water_capacity=(int)water_capacity;
    state.crew_capacity=std::max(state.crew_current,crew_quarters_capacity());
    status_.restore(state);
    last_supply_services_=services;
    return services;
}
}
