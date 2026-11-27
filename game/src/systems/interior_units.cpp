#include "game/game_app.hpp"
#include "game/world/interior_unit_mesh.hpp"
#include "game/world/habitat_builder.hpp"
#include <bit>
#include "game/world/gas_flow.hpp"
namespace game {
bool GameApp::interior_unit_reserved(std::size_t index,const InteriorUnit& unit) const {
    const auto& host=placed_habitats_[index];
    if(host.kind==HabitatKind::AirLock)return true; // Integral fit replaces legacy optional terminals.
    for(const auto& run:corridor_network_.connections())for(const auto* e:{&run.endpoint_a,&run.endpoint_b}){
        if(e->kind!=CorridorEndpointKind::Habitat||e->index!=index)continue;
        auto wall=world_math::rotate_y(e->door.wall_position-host.position,-host.rotation_degrees);wall.y=habitat_models_.mesh(host).walkable_floor_y;
        auto normal=world_math::rotate_y(e->door.normal,-host.rotation_degrees);normal.y=0;
        if(runtime::length(normal)>.01f&&interior_unit_reservation_hits(unit,wall,runtime::normalize(normal)))return true;
    }
    return false;
}
const HabitatMesh& GameApp::assemble_interior_units(std::size_t index,const HabitatMesh& source) const {
    if(interior_unit_meshes_.size()<placed_habitats_.size())interior_unit_meshes_.resize(placed_habitats_.size());
    const auto& host=placed_habitats_[index];if(host.units.empty())return source;
    auto& cache=interior_unit_meshes_[index];std::vector<std::uint64_t> signature{std::uint64_t(host.kind),std::uint64_t(host.size),source.blocked_grow_bays};
    for(const auto& station:source.stations)signature.push_back(station.active);
    for(const auto& u:host.units){
        signature.insert(signature.end(),{std::uint64_t(u.kind),std::uint64_t(u.size),std::bit_cast<std::uint32_t>(u.position.x),std::bit_cast<std::uint32_t>(u.position.y),std::bit_cast<std::uint32_t>(u.position.z),std::bit_cast<std::uint32_t>(u.rotation_degrees),std::uint64_t(u.enabled),u.grow_bay,std::uint64_t(interior_unit_reserved(index,u))});
    }
    if(cache.source==&source&&cache.signature==signature)return cache.mesh;
    cache.source=&source;cache.signature=std::move(signature);cache.mesh=source;
    for(const auto& unit:host.units){
        if(interior_unit_reserved(index,unit))continue;
        const auto& model=interior_unit_mesh(unit);
        const auto append=[&](std::vector<runtime::Vec3>& dst,const std::vector<runtime::Vec3>& src){
            for(const auto p:src)dst.push_back(unit.position+world_math::rotate_y(p,unit.rotation_degrees));
        };
        const auto src=interior_layout::layers(model);const auto dst=interior_layout::layers(cache.mesh);
        for(std::size_t layer=0;layer<src.size();++layer){
            // Crop lights have their own schedule; room lights follow occupancy.
            if(layer==5&&unit.kind==InteriorUnitKind::HydroponicRack)continue;
            // Disabled unit lamps keep their fixtures, without emission.
            if(layer==5&&!unit.enabled)append(cache.mesh.service_metal,*src[layer]);
            else append(*dst[layer],*src[layer]);
        }
        if(interior_unit_wall_anchored(unit.kind)&&source.dome_radii.x>0){
            const auto r=source.dome_radii,size=interior_unit_profile(unit).size;
            const auto back=world_math::rotate_y(runtime::Vec3{0,0,1},unit.rotation_degrees);
            habitats::Builder bracket;
            for(float side:{-.34f,.34f})for(float height:{.35f,.70f}){
                const auto start=unit.position+world_math::rotate_y(runtime::Vec3{side*size.x,height*size.y,size.z*.5f},unit.rotation_degrees);
                const float a=back.x*back.x/(r.x*r.x)+back.z*back.z/(r.z*r.z),b=2*(start.x*back.x/(r.x*r.x)+start.z*back.z/(r.z*r.z));
                const float c=start.x*start.x/(r.x*r.x)+start.z*start.z/(r.z*r.z)-1,disc=b*b-4*a*c;
                if(disc<0||a<1e-7f)continue;
                const float reach=(-b+std::sqrt(disc))/(2*a)-.045f;
                if(reach>.03f&&reach<.85f)bracket.cylinder_between(cache.mesh.service_metal,start,start+back*reach,.025f,8,true,false);
            }
        }
        if(unit.kind==InteriorUnitKind::HydroponicRack&&unit.enabled&&unit.grow_bay<hydroponics_bay_count(host.size))
            cache.mesh.blocked_grow_bays&=~std::uint16_t(1u<<unit.grow_bay);
    }
    return cache.mesh;
}
void GameApp::focus_interior(std::size_t index){
    if(first_person_mode_!=FirstPersonMode::Off||index>=placed_habitats_.size())return;
    const auto& host=placed_habitats_[index];const auto& model=habitat_models_.mesh(host);
    camera_.restore({std::max(model.footprint_x,model.footprint_z)*1.75f,35.26439f,host.rotation_degrees+45.f,
                     host.position+runtime::Vec3{0,model.walkable_floor_y+.8f,0}});
}
bool GameApp::place_interior_unit(std::size_t index,InteriorUnit unit,std::string& error){
    // Enforce the same orientation for programmatic installation as for the preview.
    if(index<placed_habitats_.size())unit.rotation_degrees=interior_unit_wall_yaw(habitat_models_.mesh(placed_habitats_[index]),unit.position);
    if(!interior_unit_position_clear(index,unit,error))return false;
    auto& host=placed_habitats_[index];const auto& profile=interior_unit_profile(unit);
    if(!resources_.can_consume(profile.cost)){error="NOT ENOUGH CONSTRUCTION MATERIAL";return false;}
    if(unit.kind==InteriorUnitKind::HydroponicRack){
        std::size_t bay=0;for(;bay<hydroponics_bay_count(host.size);++bay){
            bool occupied=false;for(const auto& other:host.units)occupied|=other.kind==InteriorUnitKind::HydroponicRack&&other.grow_bay==bay;
            if(!occupied)break;
        }
        if(bay>=hydroponics_bay_count(host.size)){error="ALL GROW BAYS ALREADY HAVE A RACK";return false;}unit.grow_bay=static_cast<std::uint8_t>(bay);
    }
    for(std::size_t i=0;i<kMaterialResourceCount;++i){const auto material=static_cast<MaterialResource>(i);
        if(profile.cost.amount(material)>0)consume_colony_stock(material,profile.cost.amount(material));
    }
    host.units.push_back(unit);error.clear();return true;
}
std::string GameApp::interior_unit_status(std::size_t host_index,const InteriorUnit& unit) const {
    const auto& host=placed_habitats_[host_index];
    if(interior_unit_reserved(host_index,unit))return "GATE APPROACH RESERVED";
    if(unit.kind==InteriorUnitKind::DiningTable)return "FOUR DINING PLACES";
    if(unit.kind==InteriorUnitKind::SeatingBench)return "TWO SEATS AVAILABLE";
    if(!unit.enabled||!host.power_enabled)return "SWITCHED OFF";
    if(host.supply.power_fraction<.01)return "WAITING FOR POWER";
    if(unit.kind==InteriorUnitKind::DrinkingWell)return "DRINKING WATER "+structure_display::value(unit.water_litres,1)+" / 10 L";
    if(unit.kind==InteriorUnitKind::FoodProcessor&&unit.food_kg>=20)return "PREPARED MEALS FULL / 20 KG";
    if(unit.kind==InteriorUnitKind::OxygenGenerator){
        double capacity=0;for(const auto& u:host.units)if(u.enabled&&!interior_unit_reserved(host_index,u))capacity+=oxygen_unit_capacity(u);
        return "ELECTROLYSIS / "+structure_display::value(host.supply.oxygen_produced_kg_per_day*oxygen_unit_capacity(unit)/std::max(.001,capacity),1)+" KG O2/DAY";
    }
    if(unit.kind==InteriorUnitKind::ServiceTerminal)return "SERVICES ONLINE";
    if(unit.kind==InteriorUnitKind::HydroponicRack){
        if(host.supply.breathability()<80)return "WAITING FOR CABIN AIR";
        return "GROW BAY "+std::to_string(unsigned(unit.grow_bay)+1)+(host.supply.breathability()<95?" / REDUCED GROWTH":" / READY");
    }
    if(host.supply.breathability()<95)return "WAITING FOR CABIN AIR";
    if(unit.batch_active)return unit.progress>=1?"OUTPUT WAITING FOR CLEARANCE":"PROCESSING "+std::to_string(int(unit.progress*100))+"%";
    return "WAITING FOR INPUT";
}
bool GameApp::find_interior_unit_bay(std::size_t index,InteriorUnit& unit) const {
    const auto& model=habitat_models_.mesh(placed_habitats_[index]);std::string error;
    for(float ratio:{.58f,.565f,.56f,.55f,.68f,.76f,.48f,.84f,.40f,.44f,.50f,.62f,.64f,.72f,.80f})for(int sector=0;sector<64;++sector){
        const float angle=6.2831853f*sector/64;
        unit.position={model.dome_radii.x*ratio*std::sin(angle),model.walkable_floor_y,model.dome_radii.z*ratio*std::cos(angle)};
        unit.rotation_degrees=interior_unit_wall_yaw(model,unit.position);
        if(interior_unit_position_clear(index,unit,error))return true;
    }return false;
}
void GameApp::align_saved_interior_units(){
    const auto prior_mode=first_person_mode_;first_person_mode_=FirstPersonMode::Off;
    for(std::size_t i=0;i<placed_habitats_.size();++i){auto& h=placed_habitats_[i];if(h.units.empty())continue;
        // Keep all other units reserved while rotating or relocating one. This
        // prevents a repaired unit from occupying a later legacy unit's space.
        for(std::size_t j=0;j<h.units.size();++j){
            const auto saved=h.units[j];auto unit=saved;const auto& model=habitat_models_.mesh(h);
            unit.position.y=model.walkable_floor_y;
            unit.rotation_degrees=interior_unit_wall_yaw(model,unit.position);
            h.units.erase(h.units.begin()+static_cast<std::ptrdiff_t>(j));
            std::string error;
            if(!interior_unit_position_clear(i,unit,error)){
                // Prefer a nearby valid bay over a large move across the room.
                bool found=false;
                for(float radius:{.25f,.5f,.75f,1.f,1.5f,2.f}){
                    for(int sector=0;sector<32&&!found;++sector){
                        const float angle=6.2831853f*sector/32;
                        unit=saved;unit.position.y=model.walkable_floor_y;
                        unit.position.x+=radius*std::sin(angle);unit.position.z+=radius*std::cos(angle);
                        unit.rotation_degrees=interior_unit_wall_yaw(model,unit.position);
                        found=interior_unit_position_clear(i,unit,error);
                    }
                    if(found)break;
                }
                if(!found&&!find_interior_unit_bay(i,unit))unit=saved;
            }
            h.units.insert(h.units.begin()+static_cast<std::ptrdiff_t>(j),unit);
        }
    }
    first_person_mode_=prior_mode;
}
void GameApp::fit_oxygen_units(){
    for(std::size_t i=0;i<placed_habitats_.size();++i){auto& host=placed_habitats_[i];
        if(host.kind!=HabitatKind::OxygenGeneratorDome||!host.construction.complete()||host.oxygen_unit_fitted)continue;
        if(std::any_of(host.units.begin(),host.units.end(),[](const auto& u){return u.kind==InteriorUnitKind::OxygenGenerator;})){host.oxygen_unit_fitted=true;continue;}
        InteriorUnit unit;unit.kind=InteriorUnitKind::OxygenGenerator;unit.position.y=habitat_models_.mesh(host).walkable_floor_y;
        if(find_interior_unit_bay(i,unit))host.units.push_back(unit);
        host.oxygen_unit_fitted=true; // Never retry a blocked fit every frame.
    }
}
void GameApp::update_interior_units(float dt){
    if(dt<=0)return;
    for(std::size_t index=0;index<placed_habitats_.size();++index){auto& host=placed_habitats_[index];
        if(!host.construction.complete()||!host.power_enabled||host.supply.power_fraction<=0||host.supply.breathability()<95)continue;
        for(auto& unit:host.units){const auto& recipe=interior_unit_profile(unit);
            if(!unit.enabled||interior_unit_reserved(index,unit)||recipe.cycle_days<=0)continue;
            if(unit.kind==InteriorUnitKind::FoodProcessor&&!unit.batch_active&&unit.food_kg>18)continue;
            if(!unit.batch_active){
                if(unit.kind==InteriorUnitKind::MetalFabricator||unit.kind==InteriorUnitKind::OreRefinery){
                    std::size_t ore_kind=0;
                    while(ore_kind<4&&host.unit_ore_kg[ore_kind]<ore::box_kg)++ore_kind;
                    if(ore_kind>=4||host.unit_ore_kg[ore_kind]<ore::box_kg)continue;
                    host.unit_ore_kg[ore_kind]-=ore::box_kg;unit.ore_kind=static_cast<std::uint8_t>(ore_kind);
                }else{
                    if(!resources_.can_consume(recipe.input))continue;
                    for(std::size_t m=0;m<kMaterialResourceCount;++m){const auto material=static_cast<MaterialResource>(m);
                        if(recipe.input.amount(material)>0)consume_colony_stock(material,recipe.input.amount(material));
                    }
                }
                unit.batch_active=true;unit.progress=0;
            }
            unit.progress=std::min(1.,unit.progress+double(dt)*host.supply.power_fraction/(kGameDaySeconds*recipe.cycle_days));
            if(unit.progress>=1){
                if(unit.kind==InteriorUnitKind::FoodProcessor){
                    unit.food_kg+=2;unit.batch_active=false;unit.progress=0;++unit.batches;continue;
                }
                const auto output=(unit.kind==InteriorUnitKind::OreRefinery||unit.kind==InteriorUnitKind::MetalFabricator)?ore::materials[unit.ore_kind]:recipe.output;
                const auto& model=habitat_models_.mesh(host);
                const auto position=host.position+world_math::rotate_y({model.footprint_x*.5f+2.f,0,0},host.rotation_degrees);
                if(spawn_produced_resource(output,recipe.output_kg,position,false)){unit.batch_active=false;unit.progress=0;++unit.batches;}
            }
        }
    }
}
void GameApp::render_interior_unit_lamps(runtime::SceneRenderer& renderer,std::size_t index) const {
    using namespace runtime;const auto& host=placed_habitats_[index];if(!host.construction.complete()||mesh_layout_active())return;
    for(const auto& unit:host.units){if(interior_unit_reserved(index,unit))continue;
        if(unit.kind==InteriorUnitKind::HydroponicRack&&unit.enabled&&hydroponics_lights_on(status_.hour())&&door_power_available(host.supply,host.power_enabled)){
            std::vector<Vec3> lamps;for(const auto p:interior_unit_mesh(unit).lamps)
                lamps.push_back(host.position+world_math::rotate_y(unit.position+world_math::rotate_y(p,unit.rotation_degrees),host.rotation_degrees));
            renderer.draw_lunar_surface(lamps,LunarMaterial::Emissive,{.90f,.76f,.92f,1});
        }
    }
}
}
