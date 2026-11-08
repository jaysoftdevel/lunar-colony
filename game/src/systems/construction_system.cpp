#include "game/game_app.hpp"
#include "game/robots/path_planner.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

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

ConstructionTargetRef GameApp::selected_construction_target() const {
    switch(selection_kind_){
        case SelectionKind::Habitat: return {ConstructionTargetKind::Habitat,(std::size_t)selected_habitat_};
        case SelectionKind::Resource: return {ConstructionTargetKind::Resource,(std::size_t)selected_resource_};
        case SelectionKind::Corridor: return {ConstructionTargetKind::Corridor,(std::size_t)selected_corridor_};
        case SelectionKind::Utility: return {ConstructionTargetKind::Utility,(std::size_t)selected_utility_};
        case SelectionKind::Walkway: return {ConstructionTargetKind::Walkway,(std::size_t)selected_walkway_};
        default: return {};
    }
}
std::uint64_t GameApp::construction_target_sequence(const ConstructionTargetRef& t) const {
    switch(t.kind){
        case ConstructionTargetKind::Habitat: return t.index<placed_habitats_.size()?placed_habitats_[t.index].placement_sequence:0;
        case ConstructionTargetKind::Resource: return t.index<placed_resources_.size()?placed_resources_[t.index].placement_sequence:0;
        case ConstructionTargetKind::Corridor: return t.index<corridor_network_.connections().size()?corridor_network_.connections()[t.index].placement_sequence:0;
        case ConstructionTargetKind::Utility: return t.index<utility_network_.connections().size()?utility_network_.connections()[t.index].placement_sequence:0;
        case ConstructionTargetKind::Walkway: return t.index<walkway_network_.connections().size()?walkway_network_.connections()[t.index].placement_sequence:0;
        default: return 0;
    }
}
std::size_t GameApp::construction_priority_rank(const ConstructionTargetRef& t) const {
    const auto seq=construction_target_sequence(t);
    const auto it=std::find(construction_priority_.begin(),construction_priority_.end(),seq);
    return seq && it!=construction_priority_.end()?(std::size_t)(it-construction_priority_.begin()):kNoRobotIndex;
}

std::vector<GameApp::BuildQueueItem> GameApp::construction_queue() const {
    std::unordered_map<std::uint64_t,std::size_t> explicit_rank,manual_rank;
    for(std::size_t i=0;i<construction_order_.size();++i) explicit_rank.emplace(construction_order_[i].sequence,i);
    for(std::size_t i=0;i<construction_priority_.size();++i) manual_rank.emplace(construction_priority_[i],i);

    struct RankedItem {
        BuildQueueItem item;
        std::tuple<int,std::size_t,std::uint64_t> key;
    };
    std::vector<RankedItem> pending;
    const auto collect=[&](const auto& items,ConstructionTargetKind kind){
        for(std::size_t i=0;i<items.size();++i){
            const auto& placed=items[i];
            if(placed.construction.complete()) continue;
            BuildQueueItem item{{kind,i},placed.placement_sequence};
            const auto ordered=explicit_rank.find(item.sequence);
            const auto manual=manual_rank.find(item.sequence);
            item.automatic_priority=construction_ready_connection(item.target);
            item.manual_priority=manual!=manual_rank.end();
            const auto sequence=item.sequence?item.sequence:std::numeric_limits<std::uint64_t>::max();
            if(ordered!=explicit_rank.end()){
                item.automatic_priority&=construction_order_[ordered->second].automatic_priority;
                pending.push_back({item,{1,ordered->second,sequence}});
            }else if(item.automatic_priority){
                pending.push_back({item,{0,0,sequence}});
            }else if(item.manual_priority){
                pending.push_back({item,{2,manual->second,sequence}});
            }else{
                pending.push_back({item,{3,0,sequence}});
            }
        }
    };
    collect(placed_habitats_,ConstructionTargetKind::Habitat);
    collect(placed_resources_,ConstructionTargetKind::Resource);
    collect(corridor_network_.connections(),ConstructionTargetKind::Corridor);
    collect(utility_network_.connections(),ConstructionTargetKind::Utility);
    collect(walkway_network_.connections(),ConstructionTargetKind::Walkway);
    std::stable_sort(pending.begin(),pending.end(),[](const auto& a,const auto& b){return a.key<b.key;});

    std::vector<BuildQueueItem> queue;
    queue.reserve(pending.size());
    for(const auto& job:pending) queue.push_back(job.item);
    return queue;
}

void GameApp::set_construction_order(const std::vector<BuildQueueItem>& queue){
    build_queue_revealed_={};
    construction_order_.clear();
    construction_order_.reserve(queue.size());
    construction_priority_.clear();
    for(const auto& item:queue){
        if(!item.sequence) continue;
        construction_order_.push_back({item.sequence,item.automatic_priority});
        if(item.manual_priority) construction_priority_.push_back(item.sequence);
    }
    astronaut_schedule_seconds_=0.f;
}

bool GameApp::move_construction_order(std::uint64_t sequence,int direction){
    if(!sequence || (direction!=-1 && direction!=1)) return false;
    auto queue=construction_queue();
    const auto item=std::find_if(queue.begin(),queue.end(),[&](const auto& job){return job.sequence==sequence;});
    if(item==queue.end()) return false;
    const auto row=item-queue.begin();
    const auto neighbor=row+direction;
    if(neighbor<0 || neighbor>=static_cast<std::ptrdiff_t>(queue.size()) || !queue[neighbor].sequence) return false;

    std::swap(queue[row],queue[neighbor]);
    const auto upper=std::min(row,neighbor),lower=std::max(row,neighbor);
    if(!queue[upper].prioritized()){
        queue[lower].manual_priority=false;
        queue[lower].automatic_priority=false;
    }
    set_construction_order(queue);
    if(game_session_active_)
        activity_log_.record(ui_time_seconds_,LogEvent::Note,construction_target_name(queue[neighbor].target)+
            " / build order moved to "+std::to_string(neighbor+1));
    return true;
}

void GameApp::toggle_construction_priority(const ConstructionTargetRef& t){
    if(!t.valid() || construction_target_complete(t))return;
    prune_construction_priorities();
    auto seq=construction_target_sequence(t);if(!seq)return;
    const auto log_priority=[&](bool enabled){
        if(game_session_active_)
            activity_log_.record(ui_time_seconds_,LogEvent::Note,construction_target_name(t)+
                (enabled?" / build priority set":" / build priority cleared"));
    };
    if(!construction_order_.empty()){
        auto queue=construction_queue();
        const auto row=std::find_if(queue.begin(),queue.end(),[&](const auto& item){return item.sequence==seq;});
        if(row==queue.end()) return;
        auto item=*row;
        item.manual_priority=!item.manual_priority;
        if(!item.automatic_priority){
            queue.erase(row);
            const auto ordinary=std::find_if(queue.begin(),queue.end(),[](const auto& job){return !job.prioritized();});
            queue.insert(ordinary,item);
        }else *row=item;
        set_construction_order(queue);
        log_priority(item.manual_priority);
        return;
    }
    const auto it=std::find(construction_priority_.begin(),construction_priority_.end(),seq);
    const bool enabled=it==construction_priority_.end();
    if(enabled)construction_priority_.push_back(seq);
    else construction_priority_.erase(it);
    log_priority(enabled);
    build_queue_revealed_={};
    astronaut_schedule_seconds_=0.f;
}
bool GameApp::construction_ready_connection(const ConstructionTargetRef& t) const {
    if(construction_target_complete(t) || !construction_target_dependencies_ready(t))return false;
    // Open-ended segments are not connections between completed structures.
    if(t.kind==ConstructionTargetKind::Corridor && t.index<corridor_network_.connections().size()){
        const auto& c=corridor_network_.connections()[t.index];return c.endpoint_a.kind!=CorridorEndpointKind::OpenDoor && c.endpoint_b.kind!=CorridorEndpointKind::OpenDoor;
    }
    if(t.kind==ConstructionTargetKind::Utility && t.index<utility_network_.connections().size()){
        const auto& c=utility_network_.connections()[t.index];return c.endpoint_a.kind!=UtilityEndpointKind::Open && c.endpoint_b.kind!=UtilityEndpointKind::Open;
    }
    return false;
}

bool GameApp::construction_queue_allows(const BuildQueueItem& item) const {
    return !construction_target_complete(item.target) &&
           (!item.manual_priority || item.automatic_priority || construction_dispatch_rank(item.target)!=3);
}

bool GameApp::construction_automatic_priority(const ConstructionTargetRef& target) const {
    if(!construction_ready_connection(target)) return false;
    const auto sequence=construction_target_sequence(target);
    const auto slot=std::find_if(construction_order_.begin(),construction_order_.end(),[&](const auto& item){
        return item.sequence==sequence;
    });
    return slot==construction_order_.end() || slot->automatic_priority;
}

void GameApp::prune_construction_priorities(){
    if(construction_priority_.empty() && construction_order_.empty())return;
    std::unordered_set<std::uint64_t> pending;
    const auto collect=[&](const auto& items){for(const auto& item:items)if(!item.construction.complete())pending.insert(item.placement_sequence);};
    collect(placed_habitats_);collect(placed_resources_);collect(corridor_network_.connections());collect(utility_network_.connections());collect(walkway_network_.connections());
    std::erase_if(construction_priority_,[&](auto seq){return !pending.count(seq);});
    std::erase_if(construction_order_,[&](const auto& slot){return !pending.count(slot.sequence);});
}
std::size_t GameApp::construction_dispatch_rank(const ConstructionTargetRef& candidate) const {
    if(construction_automatic_priority(candidate))return 0;
    if(construction_priority_.empty())return 2;
    const auto manual=construction_priority_rank(candidate);
    if(manual==0)return 1;
    // Prerequisites of the FIFO head must remain executable, even if they were
    // queued later. Never deadlock a gate/corridor behind its own foundation.
    std::vector<ConstructionTargetRef> stack,seen;
    const auto find_head=[&](const auto& items,ConstructionTargetKind kind){for(std::size_t i=0;i<items.size();++i)if(items[i].placement_sequence==construction_priority_.front())stack.push_back({kind,i});};
    find_head(placed_habitats_,ConstructionTargetKind::Habitat);find_head(placed_resources_,ConstructionTargetKind::Resource);find_head(corridor_network_.connections(),ConstructionTargetKind::Corridor);find_head(utility_network_.connections(),ConstructionTargetKind::Utility);find_head(walkway_network_.connections(),ConstructionTargetKind::Walkway);
    while(!stack.empty()){
        const auto t=stack.back();stack.pop_back();if(construction_target_complete(t))continue;
        if(std::any_of(seen.begin(),seen.end(),[&](auto old){return old.same_as(t);}))continue;
        seen.push_back(t);
        if(t.same_as(candidate))return 1;
        if(t.kind==ConstructionTargetKind::Resource && t.index<placed_resources_.size()){
            const auto& r=placed_resources_[t.index];if(r.kind==ResourceKind::StorageGate && r.storage_parent_index<placed_resources_.size())stack.push_back({ConstructionTargetKind::Resource,r.storage_parent_index});
        }else if(t.kind==ConstructionTargetKind::Corridor && t.index<corridor_network_.connections().size()){
            const auto& c=corridor_network_.connections()[t.index];for(const auto* e:{&c.endpoint_a,&c.endpoint_b}){
                if(e->kind==CorridorEndpointKind::Habitat)stack.push_back({ConstructionTargetKind::Habitat,e->index});
                if(e->kind==CorridorEndpointKind::Resource)stack.push_back({ConstructionTargetKind::Resource,e->index});
                if(e->kind==CorridorEndpointKind::Corridor)stack.push_back({ConstructionTargetKind::Corridor,e->index});
            }
        }else if(t.kind==ConstructionTargetKind::Utility && t.index<utility_network_.connections().size()){
            const auto& c=utility_network_.connections()[t.index];for(const auto* e:{&c.endpoint_a,&c.endpoint_b}){
                if(e->kind==UtilityEndpointKind::Habitat)stack.push_back({ConstructionTargetKind::Habitat,e->index});
                if(e->kind==UtilityEndpointKind::Resource)stack.push_back({ConstructionTargetKind::Resource,e->index});
                if(e->kind==UtilityEndpointKind::Corridor)stack.push_back({ConstructionTargetKind::Corridor,e->index});
            }
        }else if(t.kind==ConstructionTargetKind::Walkway && t.index<walkway_network_.connections().size()){
            const auto& c=walkway_network_.connections()[t.index];for(const auto* e:{&c.endpoint_a,&c.endpoint_b}){
                if(e->kind==WalkwayEndpointKind::Habitat)stack.push_back({ConstructionTargetKind::Habitat,e->index});
                if(e->kind==WalkwayEndpointKind::Resource)stack.push_back({ConstructionTargetKind::Resource,e->index});
                if(e->kind==WalkwayEndpointKind::Corridor)stack.push_back({ConstructionTargetKind::Corridor,e->index});
                if(e->kind==WalkwayEndpointKind::Walkway)stack.push_back({ConstructionTargetKind::Walkway,e->index});
            }
        }
    }
    return manual==kNoRobotIndex?2:3; // queued followers wait for actual completion
}
bool GameApp::debug_robot_move_available(std::size_t index) const {
    if(!debug_mode_active() || index>=builder_robots_.size() || lander_.salvage_uses_robot(index))return false;
    const auto& r=builder_robots_[index];
    return r.mode==BuilderRobotMode::Idle && !r.target.valid() && r.cargo.empty() && r.logistics_reservation.empty() &&
        r.action==BuilderRobotAction::None && !r.drop_exit_pending;
}
bool GameApp::issue_debug_robot_move(std::size_t index,eng::Vec3 destination){
    const float limit=(float)playfield_.size()*0.5f-1.5f;
    if(!debug_robot_move_available(index) || !std::isfinite(destination.x) || !std::isfinite(destination.z) ||
        std::abs(destination.x)>limit || std::abs(destination.z)>limit)return false;
    auto& r=builder_robots_[index];r.debug_move_active=true;r.debug_move_destination=destination;
    r.workshop_parked=false;r.workshop_returning=false;r.post_delivery_hold=false;
    // Invalidate advisory detours, without bypassing the motion authority or
    // discarding its active braking contract. The next safety tick certifies
    // a fresh route from the actual pose to the requested destination.
    r.safety.temporary_goal_valid=false;r.safety.yielding_for_traffic=false;
    r.safety.cached_route.clear();robots::reset_incremental_path_state(r.safety.incremental_path);
    r.safety.route_cursor=0;r.safety.route_replan_required=true;
    return true;
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
    if(lander_.deploying())return;
    prune_construction_priorities();
    assign_storage_loading_faces();
    synchronize_storage_gates();
    // Rovers use the physical material ledger. In the astronaut test, crew
    // assemble pre-stocked blueprints through their own movement update.
    update_resource_economy(dt);

    // Robot navigation/operations use bounded simulation substeps. QUICK/FAST
    // used to feed one large scaled dt into path following, causing visible
    // stepping, interaction overshoot and more frequent robot-vs-robot stalls.
    // Keep the economy on the full simulation delta but advance the fleet in
    // small deterministic slices.
    // Robot pose changes are authorized by the shared safety coordinator. The
    // outer integration remains bounded; the coordinator further subdivides each
    // slice into 10 ms contract/verification ticks.
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
        if(lander_.salvage_worker>=0&&lander_.salvage_seconds>=kLanderSalvageSeconds){
            auto& worker=builder_robots_[(std::size_t)lander_.salvage_worker];
            worker.action=BuilderRobotAction::None;
            worker.action_elapsed=worker.action_duration=0;
            complete_lander_salvage();
        }
        remaining-=step;
    }

    synchronize_storage_gates();
    for(auto& structure:placed_resources_)
        if(structure.kind==ResourceKind::StorageDome) sync_stored_resource_totals(structure);
}

} // namespace game
