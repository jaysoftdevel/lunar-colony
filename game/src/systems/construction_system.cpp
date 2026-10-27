#include "game/game_app.hpp"

namespace game {
namespace {
void add_bundle(ResourceBundle& dst,const ResourceBundle& src){
    dst.iron+=src.iron;
    dst.copper+=src.copper;
    dst.silicon+=src.silicon;
    dst.plastic+=src.plastic;
}
}

ResourceBundle GameApp::committed_construction_resources() const {
    ResourceBundle committed{};
    for(const auto& habitat:placed_habitats_){
        add_bundle(committed,habitat.resource_plan.remaining(habitat.construction.step));
    }
    for(const auto& resource:placed_resources_){
        add_bundle(committed,resource.resource_plan.remaining(resource.construction.step));
    }
    for(const auto& corridor:corridor_network_.connections()){
        add_bundle(committed,corridor.resource_plan.remaining(corridor.construction.step));
    }
    for(const auto& channel:utility_network_.connections()){
        add_bundle(committed,channel.resource_plan.remaining(channel.construction.step));
    }
    for(const auto& walkway:walkway_network_.connections()){
        add_bundle(committed,walkway.resource_plan.remaining(walkway.construction.step));
    }
    return committed;
}

bool GameApp::can_start_construction(const ConstructionResourcePlan& plan) const {
    if(debug_frame_enabled_) return true;
    ResourceBundle need=committed_construction_resources();
    add_bundle(need,plan.total());
    return resources_.amount(MaterialResource::Iron)>=need.iron &&
           resources_.amount(MaterialResource::Copper)>=need.copper &&
           resources_.amount(MaterialResource::Silicon)>=need.silicon &&
           resources_.amount(MaterialResource::Plastic)>=need.plastic;
}


void GameApp::consume_storage_inventory(const ResourceBundle& cost){
    ResourceBundle remaining=cost;
    for(auto& structure:placed_resources_){
        if(structure.kind!=ResourceKind::StorageDome || !structure.construction.complete()) continue;
        for(MaterialResource material:{MaterialResource::Iron,MaterialResource::Copper,MaterialResource::Silicon,MaterialResource::Plastic}){
            auto& need=remaining.amount(material);
            if(need<=0) continue;
            auto& stored=structure.stored_resources.amount(material);
            const std::int64_t take=std::min(stored,need);
            stored-=take;
            need-=take;
        }
        if(remaining.empty()) break;
    }
}

void GameApp::update_construction(float dt){
    const bool allow_negative=debug_frame_enabled_;
    auto consume=[&](const ResourceBundle& cost){
        if(!resources_.consume(cost,allow_negative)) return false;
        consume_storage_inventory(cost);
        return true;
    };

    for(auto& habitat:placed_habitats_){
        habitat.construction.update_gated(dt,[&](std::uint8_t next_step){
            return consume(habitat.resource_plan.next_step_cost((std::uint8_t)(next_step-1)));
        });
    }
    for(auto& resource:placed_resources_){
        resource.construction.update_gated(dt,[&](std::uint8_t next_step){
            return consume(resource.resource_plan.next_step_cost((std::uint8_t)(next_step-1)));
        });
    }
    corridor_network_.update_construction(dt,resources_,allow_negative);
    utility_network_.update_construction(dt,resources_,allow_negative);
    walkway_network_.update_construction(dt,resources_,allow_negative);

    // Corridor/channel/walkway networks consume the shared inventory directly.
    // Clamp physical dome contents to the resulting global stock so the dome
    // status remains a truthful representation of what is actually available.
    for(MaterialResource material:{MaterialResource::Iron,MaterialResource::Copper,MaterialResource::Silicon,MaterialResource::Plastic}){
        std::int64_t stored_total=0;
        for(const auto& structure:placed_resources_)
            if(structure.kind==ResourceKind::StorageDome && structure.construction.complete()) stored_total+=structure.stored_resources.amount(material);
        std::int64_t excess=stored_total-std::max<std::int64_t>(0,resources_.amount(material));
        if(excess<=0) continue;
        for(auto& structure:placed_resources_){
            if(structure.kind!=ResourceKind::StorageDome || !structure.construction.complete()) continue;
            auto& stored=structure.stored_resources.amount(material);
            const std::int64_t take=std::min(stored,excess);
            stored-=take; excess-=take;
            if(excess<=0) break;
        }
    }
}

} // namespace game
