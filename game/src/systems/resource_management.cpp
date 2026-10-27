#include "game/systems/resource_management.hpp"
#include <algorithm>

namespace game {

std::int64_t ResourceBundle::amount(MaterialResource resource) const {
    switch(resource){
        case MaterialResource::Iron: return iron;
        case MaterialResource::Copper: return copper;
        case MaterialResource::Silicon: return silicon;
        case MaterialResource::Plastic: return plastic;
        case MaterialResource::Water: return water;
        case MaterialResource::Food: return food;
        case MaterialResource::Medical: return medical;
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
        case MaterialResource::Water: return water;
        case MaterialResource::Food: return food;
        case MaterialResource::Medical: return medical;
        case MaterialResource::Count: break;
    }
    return iron;
}

std::int64_t ResourceBundle::total_amount() const {
    std::int64_t total=0;
    for(std::size_t i=0;i<kMaterialResourceCount;++i) total+=amount(static_cast<MaterialResource>(i));
    return total;
}

bool ResourceBundle::empty() const {
    for(std::size_t i=0;i<kMaterialResourceCount;++i)
        if(amount(static_cast<MaterialResource>(i))!=0) return false;
    return true;
}

const ResourceBundle& ConstructionResourcePlan::next_step_cost(std::uint8_t completed_steps) const {
    static const ResourceBundle zero{};
    if(completed_steps>=total_steps || completed_steps>=steps.size()) return zero;
    return steps[completed_steps];
}

ResourceBundle ConstructionResourcePlan::total() const {
    ResourceBundle result{};
    const std::size_t count=std::min<std::size_t>(total_steps,steps.size());
    for(std::size_t i=0;i<count;++i)
        for(std::size_t r=0;r<kMaterialResourceCount;++r){
            const auto resource=static_cast<MaterialResource>(r);
            result.amount(resource)+=steps[i].amount(resource);
        }
    return result;
}

ResourceBundle ConstructionResourcePlan::consumed(std::uint8_t completed_steps) const {
    ResourceBundle result{};
    const std::size_t count=std::min<std::size_t>({static_cast<std::size_t>(completed_steps),static_cast<std::size_t>(total_steps),steps.size()});
    for(std::size_t i=0;i<count;++i)
        for(std::size_t r=0;r<kMaterialResourceCount;++r){
            const auto resource=static_cast<MaterialResource>(r);
            result.amount(resource)+=steps[i].amount(resource);
        }
    return result;
}

ResourceBundle ConstructionResourcePlan::remaining(std::uint8_t completed_steps) const {
    const ResourceBundle all=total();
    const ResourceBundle used=consumed(completed_steps);
    ResourceBundle result{};
    for(std::size_t r=0;r<kMaterialResourceCount;++r){
        const auto resource=static_cast<MaterialResource>(r);
        result.amount(resource)=all.amount(resource)-used.amount(resource);
    }
    return result;
}

void ResourceManagement::reset(){
    // A new colony starts with no stock. Debug mode remains the intentional
    // overdraft mechanism until extraction/logistics production is expanded.
    resources_={};
}

bool ResourceManagement::can_consume(const ResourceBundle& cost) const {
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto resource=static_cast<MaterialResource>(i);
        if(resources_.amount(resource)<cost.amount(resource)) return false;
    }
    return true;
}

bool ResourceManagement::consume(const ResourceBundle& cost,bool allow_negative){
    if(!allow_negative && !can_consume(cost)) return false;
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto resource=static_cast<MaterialResource>(i);
        resources_.amount(resource)-=cost.amount(resource);
    }
    return true;
}

void ResourceManagement::add(const ResourceBundle& amount){
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto resource=static_cast<MaterialResource>(i);
        resources_.amount(resource)+=amount.amount(resource);
    }
}

const char* material_resource_name(MaterialResource resource){
    switch(resource){
        case MaterialResource::Iron: return "Iron";
        case MaterialResource::Copper: return "Copper";
        case MaterialResource::Silicon: return "Silicon";
        case MaterialResource::Plastic: return "Plastic";
        case MaterialResource::Water: return "Water";
        case MaterialResource::Food: return "Food";
        case MaterialResource::Medical: return "Medical Supplies";
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
        case MaterialResource::Water: return "H2O";
        case MaterialResource::Food: return "Fd";
        case MaterialResource::Medical: return "Med";
        case MaterialResource::Count: break;
    }
    return "?";
}

} // namespace game
