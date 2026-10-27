#include "game/systems/resource_management.hpp"
#include <algorithm>

namespace game {

std::int64_t ResourceBundle::amount(MaterialResource resource) const {
    switch(resource){
        case MaterialResource::Iron: return iron;
        case MaterialResource::Copper: return copper;
        case MaterialResource::Silicon: return silicon;
        case MaterialResource::Plastic: return plastic;
        case MaterialResource::Count: break;
    }
    return 0;
}

std::int64_t& ResourceBundle::amount(MaterialResource resource){
    switch(resource){
        case MaterialResource::Iron: return iron;
        case MaterialResource::Copper: return copper;
        case MaterialResource::Silicon: return silicon;
        case MaterialResource::Plastic: return plastic;
        case MaterialResource::Count: break;
    }
    return iron;
}

bool ResourceBundle::empty() const {
    return iron==0 && copper==0 && silicon==0 && plastic==0;
}

const ResourceBundle& ConstructionResourcePlan::next_step_cost(std::uint8_t completed_steps) const {
    static const ResourceBundle zero{};
    if(completed_steps>=total_steps || completed_steps>=steps.size()) return zero;
    return steps[completed_steps];
}

ResourceBundle ConstructionResourcePlan::total() const {
    ResourceBundle result{};
    const std::size_t count=std::min<std::size_t>(total_steps,steps.size());
    for(std::size_t i=0;i<count;++i){
        result.iron+=steps[i].iron;
        result.copper+=steps[i].copper;
        result.silicon+=steps[i].silicon;
        result.plastic+=steps[i].plastic;
    }
    return result;
}

ResourceBundle ConstructionResourcePlan::consumed(std::uint8_t completed_steps) const {
    ResourceBundle result{};
    const std::size_t count=std::min<std::size_t>({static_cast<std::size_t>(completed_steps),static_cast<std::size_t>(total_steps),steps.size()});
    for(std::size_t i=0;i<count;++i){
        result.iron+=steps[i].iron;
        result.copper+=steps[i].copper;
        result.silicon+=steps[i].silicon;
        result.plastic+=steps[i].plastic;
    }
    return result;
}

ResourceBundle ConstructionResourcePlan::remaining(std::uint8_t completed_steps) const {
    const ResourceBundle all=total();
    const ResourceBundle used=consumed(completed_steps);
    return {all.iron-used.iron,all.copper-used.copper,all.silicon-used.silicon,all.plastic-used.plastic};
}

void ResourceManagement::reset(){
    // A new colony starts with no construction stock. Debug mode is the only
    // intentional overdraft mechanism until extraction/logistics gameplay is
    // added later.
    resources_={};
}

bool ResourceManagement::can_consume(const ResourceBundle& cost) const {
    return resources_.iron>=cost.iron &&
           resources_.copper>=cost.copper &&
           resources_.silicon>=cost.silicon &&
           resources_.plastic>=cost.plastic;
}

bool ResourceManagement::consume(const ResourceBundle& cost,bool allow_negative){
    if(!allow_negative && !can_consume(cost)) return false;
    resources_.iron-=cost.iron;
    resources_.copper-=cost.copper;
    resources_.silicon-=cost.silicon;
    resources_.plastic-=cost.plastic;
    return true;
}

void ResourceManagement::add(const ResourceBundle& amount){
    resources_.iron+=amount.iron;
    resources_.copper+=amount.copper;
    resources_.silicon+=amount.silicon;
    resources_.plastic+=amount.plastic;
}

const char* material_resource_name(MaterialResource resource){
    switch(resource){
        case MaterialResource::Iron: return "Iron";
        case MaterialResource::Copper: return "Copper";
        case MaterialResource::Silicon: return "Silicon";
        case MaterialResource::Plastic: return "Plastic";
        case MaterialResource::Count: break;
    }
    return "Resource";
}

const char* material_resource_symbol(MaterialResource resource){
    switch(resource){
        case MaterialResource::Iron: return "Fe";
        case MaterialResource::Copper: return "Cu";
        case MaterialResource::Silicon: return "Si";
        case MaterialResource::Plastic: return "P";
        case MaterialResource::Count: break;
    }
    return "?";
}

} // namespace game
