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
    // Placement creates a blueprint, not an instantly funded build order.
    // Material availability therefore never blocks layout planning; BR-02 will
    // wait at the logistics stage until the required physical stock exists.
    (void)plan;
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
    // v0.31: construction never advances by timer alone. The autonomous
    // builder robot is the sole stage gate: fetch -> transport -> deliver ->
    // build. Resource facilities run in parallel and replenish physical stock.
    update_resource_economy(dt);

    // Robot navigation/operations use bounded simulation substeps. QUICK/FAST
    // used to feed one large scaled dt into path following, causing visible
    // stepping, interaction overshoot and more frequent robot-vs-robot stalls.
    // Keep the economy on the full simulation delta but advance the fleet in
    // small deterministic slices.
    // Route calculation runs synchronously on the main thread. Budget it once
    // per rendered update, not once per 90 Hz substep. A frame that arrived late
    // may integrate several motion slices, but it cannot multiply A* work.
    robot_constructor_route_plans_budget_=2;
    robot_carrier_route_plans_budget_=2;
    robot_full_route_plans_budget_=1;

    float remaining=std::max(0.0f,dt);
    // Continuous fleet motion is integrated at 90 Hz.  A larger guard prevents
    // low-render-FPS / FAST mode frames from silently dropping the tail of the
    // robot simulation delta, which previously appeared as periodic motion
    // stalls even when the planned route itself was smooth.
    constexpr float kRobotMaxStep=1.0f/90.0f;
    int guard=0;
    while(remaining>0.000001f && guard++<64){
        const float step=std::min(remaining,kRobotMaxStep);
        update_builder_robot(step);
        remaining-=step;
    }

    for(auto& structure:placed_resources_)
        if(structure.kind==ResourceKind::StorageDome) sync_stored_resource_totals(structure);
}

} // namespace game
