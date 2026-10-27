#include "game/game_app.hpp"
#include <algorithm>

namespace game {
namespace {
void add_bundle(ResourceBundle& dst,const ResourceBundle& src){
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto resource=static_cast<MaterialResource>(i);
        dst.amount(resource)+=src.amount(resource);
    }
}

void consume_stored_boxes(PlacedResource& structure,MaterialResource material,std::int64_t amount){
    std::int64_t remaining=amount;
    for(auto it=structure.stored_boxes.begin();it!=structure.stored_boxes.end() && remaining>0;){
        if(it->resource!=material || it->amount<=0){ ++it; continue; }
        const std::int64_t take=std::min(it->amount,remaining);
        it->amount-=take;
        remaining-=take;
        if(it->amount<=0) it=structure.stored_boxes.erase(it);
        else ++it;
    }
}

ResourceBundle consumed_between(const ResourceBundle& before,const ResourceBundle& after){
    ResourceBundle consumed{};
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto material=static_cast<MaterialResource>(i);
        consumed.amount(material)=std::max<std::int64_t>(0,before.amount(material)-after.amount(material));
    }
    return consumed;
}
}

ResourceBundle GameApp::committed_construction_resources() const {
    ResourceBundle committed{};
    for(const auto& habitat:placed_habitats_)
        add_bundle(committed,habitat.resource_plan.remaining(habitat.construction.step));
    for(const auto& resource:placed_resources_)
        add_bundle(committed,resource.resource_plan.remaining(resource.construction.step));
    for(const auto& corridor:corridor_network_.connections())
        add_bundle(committed,corridor.resource_plan.remaining(corridor.construction.step));
    for(const auto& channel:utility_network_.connections())
        add_bundle(committed,channel.resource_plan.remaining(channel.construction.step));
    for(const auto& walkway:walkway_network_.connections())
        add_bundle(committed,walkway.resource_plan.remaining(walkway.construction.step));
    return committed;
}

bool GameApp::can_start_construction(const ConstructionResourcePlan& plan) const {
    if(debug_frame_enabled_) return true;
    ResourceBundle need=committed_construction_resources();
    add_bundle(need,plan.total());
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto resource=static_cast<MaterialResource>(i);
        if(resources_.amount(resource)<need.amount(resource)) return false;
    }
    return true;
}

void GameApp::consume_storage_inventory(const ResourceBundle& cost){
    ResourceBundle remaining=cost;
    for(auto& structure:placed_resources_){
        if(structure.kind!=ResourceKind::StorageDome || !structure.construction.complete()) continue;
        sync_stored_resource_totals(structure);
        for(std::size_t i=0;i<kMaterialResourceCount;++i){
            const auto material=static_cast<MaterialResource>(i);
            auto& need=remaining.amount(material);
            if(need<=0) continue;
            const std::int64_t take=std::min(structure.stored_resources.amount(material),need);
            if(take<=0) continue;
            need-=take;
            consume_stored_boxes(structure,material,take);
            sync_stored_resource_totals(structure);
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
    // Routed networks update the shared inventory internally. Snapshot the
    // inventory around each network update and remove exactly the newly spent
    // material from physical Storage Dome boxes. Do not clamp dome contents to
    // the global inventory: debug overdraft can legitimately make the global
    // counter negative while real boxes are still stored in a dome.
    ResourceBundle before=resources_.amounts();
    corridor_network_.update_construction(dt,resources_,allow_negative);
    consume_storage_inventory(consumed_between(before,resources_.amounts()));

    before=resources_.amounts();
    utility_network_.update_construction(dt,resources_,allow_negative);
    consume_storage_inventory(consumed_between(before,resources_.amounts()));

    before=resources_.amounts();
    walkway_network_.update_construction(dt,resources_,allow_negative);
    consume_storage_inventory(consumed_between(before,resources_.amounts()));

    for(auto& structure:placed_resources_)
        if(structure.kind==ResourceKind::StorageDome) sync_stored_resource_totals(structure);
}

} // namespace game
