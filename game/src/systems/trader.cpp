#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/world/storage_loading_gate.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>

namespace game {
namespace {
using eng::Vec3;
float distance(Vec3 a,Vec3 b){a.y=b.y=0;return eng::length(a-b);}
std::int64_t mass(const TradeManifest& a){return std::accumulate(a.begin(),a.end(),std::int64_t{0});}
}
const char* trade_good_name(TradeGood good){
    static constexpr const char* names[]={"IRON","COPPER","SILICON","PLASTIC","WATER","FOOD","MEDICAL","ALUMINUM","OXYGEN","HYDROGEN",
        "IRON ORE","COPPER ORE","SILICON ORE","ALUMINUM ORE"};
    return names[std::min((std::size_t)good,kTradeGoodCount-1)];
}
std::int64_t trade_price_eur(TradeGood good){
    // Fixed gameplay buy prices per kg; quotes freeze when the sale is sealed.
    static constexpr std::int64_t prices[]={4,12,18,6,2,8,40,9,15,80,2,7,10,5};
    return prices[std::min((std::size_t)good,kTradeGoodCount-1)];
}
const char* trade_phase_name(TradePhase p){
    switch(p){case TradePhase::Approach:return "TRADER CALLED / APPROACHING";case TradePhase::Docked:return "TRADER DOCKED / SELECT SALE";
        case TradePhase::Loading:return "LOADING RESERVED GOODS";case TradePhase::Departing:return "TRADER DEPARTING / PAYMENT PENDING";
        case TradePhase::Complete:return "TRADE COMPLETE / CASH RECEIVED";default:return "NO TRADER CALLED";}
}
std::size_t GameApp::trader_pad_index() const {
    for(std::size_t i=0;i<placed_resources_.size();++i)if(placed_resources_[i].placement_sequence==trader_.pad_sequence&&placed_resources_[i].kind==ResourceKind::LandingPad)return i;
    return kNoRobotIndex;
}
const UtilityConnection* GameApp::trader_conveyor(std::size_t dome) const {
    const auto pad=trader_pad_index();
    if(pad>=placed_resources_.size())return nullptr;
    for(const auto& belt:utility_network_.connections())
        if(belt.kind==UtilityKind::ConveyorBelt && belt.placement_sequence && belt.construction.complete() &&
           belt.endpoint_a.index==dome && belt.endpoint_b.index==pad &&
           conveyor_route(belt.endpoint_a,belt.endpoint_b,placed_habitats_,placed_resources_)==ConveyorRoute::Trade)
            return &belt;
    return nullptr;
}
bool GameApp::trader_powered()const{
    const auto i=trader_pad_index();return i<placed_resources_.size()&&placed_resources_[i].construction.complete()&&placed_resources_[i].supply.power_fraction>=.5;
}
bool GameApp::trade_antenna_enabled() const {
    return std::any_of(placed_resources_.begin(),placed_resources_.end(),[](const auto& item){
        return item.kind==ResourceKind::Antenna && item.construction.complete() && item.power_enabled;
    });
}
double& GameApp::trader_oxygen_stock(std::uint64_t seq){
    for(auto& s:trader_.oxygen)if(s.source_sequence==seq)return s.kg;
    trader_.oxygen.push_back({seq,0});return trader_.oxygen.back().kg;
}
double GameApp::trader_oxygen_stock_amount(std::uint64_t seq)const{
    for(const auto& s:trader_.oxygen)if(s.source_sequence==seq)return s.kg;
    return 0;
}
bool GameApp::trader_robot_busy(std::size_t ri)const{
    for(const auto& h:trader_.hauls)if(h.robot==ri)return true;
    return false;
}
bool GameApp::trader_position_clear(Vec3 p,float radius,std::size_t ignore_resource,std::size_t ignore_habitat)const{
    if(std::abs(p.x)+radius>playfield_.size()*.5f-.3f||std::abs(p.z)+radius>playfield_.size()*.5f-.3f)return false;
    if(lunar_lander_segment_blocked(lander_,p,p,radius))return false;
    const auto blocked=[&](Vec3 center,float rot,float sx,float sz){auto local=world_math::rotate_y(p-center,-rot);return std::abs(local.x)<sx*.5f+radius&&std::abs(local.z)<sz*.5f+radius;};
    for(std::size_t i=0;i<placed_habitats_.size();++i)if(i!=ignore_habitat){const auto& h=placed_habitats_[i];const auto& m=habitat_models_.mesh(h);if(blocked(h.position,h.rotation_degrees,m.footprint_x,m.footprint_z))return false;}
    for(std::size_t i=0;i<placed_resources_.size();++i)if(i!=ignore_resource){const auto& r=placed_resources_[i];const auto& m=resource_models_.mesh(r);if(blocked(r.position,r.rotation_degrees,m.footprint_x,m.footprint_z))return false;}
    for(const auto& pile:ground_resource_piles_)if(pile.amount>0&&distance(p,pile.position)<radius+std::max(.36f,pile.radius))return false;
    for(const auto& c:corridor_network_.connections())for(std::size_t j=1;j<c.path.size();++j)if(world_math::point_segment_distance_xz(p,c.path[j-1],c.path[j])<radius+geometry::kCorridorHalfWidth)return false;
    for(const auto& w:walkway_network_.connections())for(std::size_t j=1;j<w.path.size();++j)if(world_math::point_segment_distance_xz(p,w.path[j-1],w.path[j])<radius+geometry::kWalkwayHalfWidth)return false;
    return true;
}
bool GameApp::trader_placement_clear(Vec3 p,float rot,float width,float depth)const{
    if(!trader_.active())return true;
    // Protect the receiving bay and its outward departure/queue aisle.
    if(trader_.phase==TradePhase::Loading)for(const auto& l:trader_.lots)if(l.remaining>0&&world_math::overlaps(world_math::footprint(p,rot,width,depth),world_math::footprint(l.position,0,2.4f,2.4f),.4f))return false;
    const auto pi=trader_pad_index();if(pi>=placed_resources_.size())return true;
    Vec3 out=trader_.dock-placed_resources_[pi].position;out.y=0;out=eng::normalize(out);
    const Vec3 center=trader_.dock+out*2.5f;
    return !world_math::overlaps(world_math::footprint(p,rot,width,depth),world_math::footprint(center,std::atan2(out.x,out.z)*180/3.14159265f,9,9),.5f);
}
bool GameApp::call_trader(std::size_t pad){
    if(active_game_phase_!=GamePhase::RobotPreparation||trader_.active()||lander_.deploying()||pad>=placed_resources_.size())return false;
    auto& p=placed_resources_[pad];if(p.kind!=ResourceKind::LandingPad||!p.construction.complete()||p.supply.power_fraction<.5)return false;
    if(!trade_antenna_enabled())return false;
    const auto& m=resource_models_.mesh(p);Vec3 dock{};bool found=false;
    for(const auto local:std::array<Vec3,4>{{{0,0,-m.footprint_z*.5f-2.6f},{m.footprint_x*.5f+2.6f,0,0},{0,0,m.footprint_z*.5f+2.6f},{-m.footprint_x*.5f-2.6f,0,0}}}){
        const auto candidate=p.position+world_math::rotate_y(local,p.rotation_degrees);
        if(trader_position_clear(candidate,1.25f,pad)){dock=candidate;found=true;break;}
    }
    if(!found){trade_message_="CLEAR A ROBOT LOADING BAY BESIDE THE PAD";return false;}
    if(!p.placement_sequence)p.placement_sequence=next_blueprint_placement_sequence_++;
    const auto oxygen=std::move(trader_.oxygen);const auto last=trader_.last_payment_eur;
    trader_={};trader_.oxygen=oxygen;trader_.oxygen_packaging=true;trader_.last_payment_eur=last;
    trader_.phase=TradePhase::Approach;trader_.pad_sequence=p.placement_sequence;trader_.dock=dock;
    trade_message_="TRADER CALLED. SELECT GOODS WHILE THE SHIP APPROACHES.";return true;
}
TradeManifest GameApp::trader_available()const{
    TradeManifest result{};
    for(const auto& p:ground_resource_piles_)if(p.amount>0)result[(std::size_t)p.resource]+=p.amount;
    for(std::size_t i=0;i<placed_resources_.size();++i){
        const auto& r=placed_resources_[i];
        if(!r.construction.complete())continue;
        if(r.kind==ResourceKind::StorageDome){
            const auto* belt=trader_conveyor(i);
            if(belt || storage_loading::has_ready_gate(r))
                for(const auto& box:r.stored_boxes)if(box.amount>0)result[(std::size_t)box.resource]+=box.amount;
            if(belt)for(std::size_t kind=0;kind<r.stored_ore_boxes.size();++kind)
                result[kLegacyTradeGoodCount+kind]+=ore::box_kg*r.stored_ore_boxes[kind];
        }
        if(r.kind==ResourceKind::OxygenTank)result[8]+=(std::int64_t)std::floor(std::max(0.0,r.stored_oxygen_kg));
        if(r.kind==ResourceKind::HydrogenTank)result[9]+=(std::int64_t)std::floor(std::max(0.0,r.supply.hydrogen_kg));
    }
    for(std::size_t i=0;i<kMaterialResourceCount;++i)result[i]=std::min(result[i],std::max<std::int64_t>(0,resources_.amount((MaterialResource)i)));
    for(const auto& h:placed_habitats_)if(h.kind==HabitatKind::OxygenGeneratorDome&&h.construction.complete()){
        result[8]+=(std::int64_t)std::floor(trader_oxygen_stock_amount(h.placement_sequence));
        result[9]+=(std::int64_t)std::floor(std::max(0.0,h.supply.hydrogen_kg));
    }
    return result;
}
bool GameApp::start_trader_sale(){
    if(trader_.phase!=TradePhase::Docked||!trader_powered())return false;
    const auto available=trader_available();const auto total=mass(trader_.requested);
    if(total<=0||total>kTraderCapacityKg){trade_message_="SELECT 1 TO 20000 KG TO SELL";return false;}
    for(std::size_t i=0;i<kTradeGoodCount;++i){
        const auto amount=trader_.requested[i];
        if(amount<0 || amount>available[i] || (i>=kLegacyTradeGoodCount && amount%ore::box_kg)){
            trade_message_="SELECTED GOODS ARE NO LONGER AVAILABLE";
            return false;
        }
    }
    enum class Stock { Ground, Dome, HydrogenTank, BottledOxygen, GeneratorHydrogen, OxygenTank, RawOre };
    struct Allocation {TradeLot lot;Stock kind;std::size_t source;};
    std::vector<Allocation> allocations;
    auto need=trader_.requested;
    const auto port=[&](Vec3 center,float rot,float fx,float fz,std::size_t ri,std::size_t hi,Vec3& position){
        for(const auto local:std::array<Vec3,4>{{{0,0,-fz*.5f-2.8f},{fx*.5f+2.8f,0,0},{0,0,fz*.5f+2.8f},{-fx*.5f-2.8f,0,0}}}){
            const auto p=center+world_math::rotate_y(local,rot);if(!trader_position_clear(p,.5f,ri,hi))continue;
            bool occupied=false;for(const auto& r:builder_robots_){
                if(r.safety.active_contract.valid){if(robots::contract_intersects_region(r.safety.active_contract,[&](Vec3 q,float radius){return distance(p,q)<radius+.6f;}))occupied=true;}
                else if(distance(p,r.position)<1.6f)occupied=true;
            }
            if(!occupied){position=p;return true;}
        }return false;
    };
    const auto reserve_dome=[&](std::size_t i,Vec3 position,std::uint64_t sequence){
        const auto& dome=placed_resources_[i];
        std::array<std::int64_t,kMaterialResourceCount> stock{};
        for(const auto& box:dome.stored_boxes)stock[(std::size_t)box.resource]+=box.amount;
        for(std::size_t m=0;m<stock.size();++m){
            const auto amount=std::min(need[m],stock[m]);
            if(!amount)continue;
            allocations.push_back({{(TradeGood)m,amount,position,sequence},Stock::Dome,i});
            need[m]-=amount;
        }
        if(!sequence)return;
        for(std::size_t kind=0;kind<dome.stored_ore_boxes.size();++kind){
            const auto good=kLegacyTradeGoodCount+kind;
            const auto amount=std::min(need[good],ore::box_kg*dome.stored_ore_boxes[kind]);
            if(!amount)continue;
            allocations.push_back({{(TradeGood)good,amount,position,sequence},Stock::RawOre,i});
            need[good]-=amount;
        }
    };
    // Offer completed belt routes to this pad before assigning loose stock
    // and other domes to rover transport.
    for(std::size_t i=0;i<placed_resources_.size();++i)
        if(placed_resources_[i].construction.complete())
            if(const auto* belt=trader_conveyor(i))
                reserve_dome(i,belt->endpoint_a.position,belt->placement_sequence);
    for(std::size_t i=0;i<ground_resource_piles_.size();++i){const auto& p=ground_resource_piles_[i];auto& n=need[(std::size_t)p.resource];const auto q=std::min(n,std::max<std::int64_t>(0,p.amount));if(q){allocations.push_back({{(TradeGood)p.resource,q,p.position},Stock::Ground,i});n-=q;}}
    for(std::size_t i=0;i<placed_resources_.size();++i){const auto& r=placed_resources_[i];if(!r.construction.complete())continue;
        if(r.kind==ResourceKind::StorageDome&&storage_loading::has_ready_gate(r)&&!trader_conveyor(i)){
            Vec3 pos{};bool has_pos=false;
            for(int face:storage_loading::faces(r,true))if(face>=0){auto p=storage_loading::service(r,face);auto out=eng::normalize(p-r.position);p+=out*2.0f;if(trader_position_clear(p,.5f,i)){pos=p;has_pos=true;break;}}
            if(has_pos)reserve_dome(i,pos,0);
        }else if(r.kind==ResourceKind::OxygenTank&&need[8]>0){
            Vec3 position;
            const auto& mesh=resource_models_.mesh(r);
            const auto amount=std::min(need[8],(std::int64_t)std::floor(r.stored_oxygen_kg));
            if(amount>0&&port(r.position,r.rotation_degrees,mesh.footprint_x,mesh.footprint_z,i,kNoRobotIndex,position)){
                allocations.push_back({{TradeGood::Oxygen,amount,position},Stock::OxygenTank,i});
                need[8]-=amount;
            }
        }else if(r.kind==ResourceKind::HydrogenTank&&need[9]>0){Vec3 pos;const auto& m=resource_models_.mesh(r);auto q=std::min(need[9],(std::int64_t)std::floor(r.supply.hydrogen_kg));if(q&&port(r.position,r.rotation_degrees,m.footprint_x,m.footprint_z,i,kNoRobotIndex,pos)){allocations.push_back({{TradeGood::Hydrogen,q,pos},Stock::HydrogenTank,i});need[9]-=q;}}
    }
    for(std::size_t i=0;i<placed_habitats_.size();++i){const auto& h=placed_habitats_[i];if(h.kind!=HabitatKind::OxygenGeneratorDome||!h.construction.complete())continue;
        const auto& m=habitat_models_.mesh(h);Vec3 pos;if(!port(h.position,h.rotation_degrees,m.footprint_x,m.footprint_z,kNoRobotIndex,i,pos))continue;
        auto q=std::min(need[8],(std::int64_t)std::floor(trader_oxygen_stock_amount(h.placement_sequence)));if(q){allocations.push_back({{TradeGood::Oxygen,q,pos},Stock::BottledOxygen,i});need[8]-=q;}
        q=std::min(need[9],(std::int64_t)std::floor(h.supply.hydrogen_kg));if(q){allocations.push_back({{TradeGood::Hydrogen,q,pos},Stock::GeneratorHydrogen,i});need[9]-=q;}
    }
    if(mass(need)!=0){trade_message_="CLEAR PACKING ACCESS AT THE RESOURCE SOURCE";return false;}
    // Atomic reservation: selected stock remains in physical export lots at its
    // source. Normal construction/fuel cells cannot consume the same kg again.
    ResourceBundle reserved;
    for(const auto& a:allocations){const auto good=(std::size_t)a.lot.good;const auto q=a.lot.remaining;
        switch(a.kind){
            case Stock::Ground:ground_resource_piles_[a.source].amount-=q;break;
            case Stock::Dome:{
                auto& dome=placed_resources_[a.source];
                auto remaining=q;
                for(auto& box:dome.stored_boxes)if((std::size_t)box.resource==good){
                    const auto take=std::min(remaining,box.amount);
                    box.amount-=take;remaining-=take;
                }
                std::erase_if(dome.stored_boxes,[](const auto& box){return box.amount<=0;});
                sync_stored_resource_totals(dome);
                break;
            }
            case Stock::RawOre:placed_resources_[a.source].stored_ore_boxes[good-kLegacyTradeGoodCount]-=
                (std::uint32_t)(q/ore::box_kg);break;
            case Stock::HydrogenTank:placed_resources_[a.source].supply.hydrogen_kg-=q;break;
            case Stock::BottledOxygen:trader_oxygen_stock(placed_habitats_[a.source].placement_sequence)-=q;break;
            case Stock::OxygenTank:placed_resources_[a.source].stored_oxygen_kg-=q;break;
            case Stock::GeneratorHydrogen:placed_habitats_[a.source].supply.hydrogen_kg-=q;break;
        }
        if(good<kMaterialResourceCount)reserved.amount((MaterialResource)good)+=q;
        trader_.lots.push_back(a.lot);
    }
    std::erase_if(ground_resource_piles_,[](const auto& p){return p.amount<=0;});resources_.consume(reserved,false);
    trader_.loaded={};trader_.sale_value_eur=0;
    for(std::size_t i=0;i<kTradeGoodCount;++i)trader_.sale_value_eur+=trader_.requested[i]*trade_price_eur((TradeGood)i);
    trader_.phase=TradePhase::Loading;trader_.elapsed=0;trade_message_="SALE SEALED. BELTS AND IDLE ROBOTS LOAD RESERVED GOODS.";return true;
}
void GameApp::update_trader(float dt){
    if(!trader_.active())return;
    if(trader_.phase!=TradePhase::Departing&&!trader_powered())return;
    if(trader_.phase==TradePhase::Approach){trader_.elapsed=std::min(24.0f,trader_.elapsed+std::max(0.0f,dt));if(trader_.elapsed>=24){trader_.phase=TradePhase::Docked;trader_.elapsed=0;}}
    else if(trader_.phase==TradePhase::Loading){
        const auto pad=trader_pad_index();
        const auto& belts=utility_network_.connections();
        for(std::size_t i=0;i<belts.size();++i){
            const auto& belt=belts[i];
            if(belt.kind!=UtilityKind::ConveyorBelt || !belt.construction.complete() || !belt.placement_sequence ||
               belt.endpoint_b.index!=pad ||
               conveyor_route(belt.endpoint_a,belt.endpoint_b,placed_habitats_,placed_resources_)!=ConveyorRoute::Trade)continue;
            float travel=ore::belt_speed_mps*std::max(0.f,dt);
            const float length=utility_network_.connection_length(i);
            if(length<=0.f)continue;
            for(auto& lot:trader_.lots){
                if(lot.conveyor_sequence!=belt.placement_sequence)continue;
                while(lot.remaining>0 && travel>0.f){
                    const float advance=std::min(travel,std::max(0.f,length-lot.conveyor_distance_m));
                    lot.conveyor_distance_m+=advance;
                    travel-=advance;
                    if(lot.conveyor_distance_m<length)break;
                    const auto amount=std::min(lot.remaining,kTradeConveyorCrateKg);
                    lot.remaining-=amount;
                    lot.conveyor_distance_m=0.f;
                    trader_.loaded[(std::size_t)lot.good]+=amount;
                }
                if(travel<=0.f)break;
            }
        }
        bool complete=trader_.hauls.empty();for(const auto& l:trader_.lots)if(l.remaining)complete=false;
        if(complete&&trader_.loaded==trader_.requested){trader_.phase=TradePhase::Departing;trader_.elapsed=0;}
    }else if(trader_.phase==TradePhase::Departing){
        trader_.elapsed=std::min(18.0f,trader_.elapsed+std::max(0.0f,dt));
        if(trader_.elapsed>=18){planning_.funds_eur+=trader_.sale_value_eur;trader_.last_payment_eur=trader_.sale_value_eur;trader_.phase=TradePhase::Complete;trader_.elapsed=0;trader_.lots.clear();trade_message_="TRADER DEPARTED. EUR "+std::to_string(trader_.last_payment_eur)+" RECEIVED.";}
    }
}
void GameApp::update_trade_hauling(float dt,const std::function<bool(std::size_t,Vec3)>& clear,
    const std::function<void(std::size_t,Vec3)>& move,const std::function<void(std::size_t)>& stop){
    if(trader_.phase!=TradePhase::Loading)return;
    // Construction dispatch already ran. Only genuinely idle, empty robots
    // accept export work; carriers are offered each lot before builders.
    if(trader_powered())for(int role=1;role>=0;--role)for(std::size_t ri=0;ri<builder_robots_.size();++ri){
        auto& r=builder_robots_[ri];
        if((int)r.role!=role||lander_.salvage_uses_robot(ri)||r.operation_retry.replan_pending()||trader_robot_busy(ri)||r.ore_haul.active()||r.mode!=BuilderRobotMode::Idle||r.target.valid()||!r.cargo.empty()||r.action!=BuilderRobotAction::None||r.debug_move_active||r.drop_exit_pending||r.starter_charge_active||r.battery_charge_kwh<r.battery_capacity_kwh*.2f)continue;
        std::size_t pick=kNoRobotIndex;float best=std::numeric_limits<float>::max();
        for(std::size_t i=0;i<trader_.lots.size();++i){if(trader_.lots[i].remaining<=0 || trader_.lots[i].conveyor_sequence)continue;bool leased=false;for(const auto& h:trader_.hauls)if(h.lot==i&&h.phase==0)leased=true;if(leased)continue;const float d=distance(r.position,trader_.lots[i].position);if(d<best){pick=i;best=d;}}
        if(pick!=kNoRobotIndex){TradeHaul h;h.robot=(std::uint8_t)ri;h.lot=(std::uint32_t)pick;trader_.hauls.push_back(h);r.mode=BuilderRobotMode::Fetching;r.workshop_returning=false;r.workshop_parked=false;}
    }
    std::size_t owner=kNoRobotIndex;
    // A rover owns the receiving station until it has cleared the apron.
    for(const auto& h:trader_.hauls)if(h.phase==2){owner=h.robot;break;}
    if(owner==kNoRobotIndex)for(const auto& h:trader_.hauls)if(h.phase==1){owner=h.robot;break;}
    const auto pi=trader_pad_index();if(pi>=placed_resources_.size())return;
    Vec3 out=trader_.dock-placed_resources_[pi].position;out.y=0;out=eng::normalize(out);const Vec3 side{-out.z,0,out.x};
    std::vector<std::uint8_t> finished;
    for(auto& h:trader_.hauls){
        auto& r=builder_robots_[h.robot];auto& lot=trader_.lots[h.lot];
        r.mode=h.phase==0?BuilderRobotMode::Fetching:BuilderRobotMode::Delivering;
        r.workshop_returning=false;r.workshop_parked=false;r.starter_charge_active=false;
        if(!trader_powered()){stop(h.robot);r.action=BuilderRobotAction::None;continue;}
        // Safety evacuation remains authoritative over an export operation.
        if(!clear(h.robot,r.position))continue;
        if(h.phase==0&&!h.service_valid){
            const float preferred=std::atan2(r.position.x-lot.position.x,r.position.z-lot.position.z);
            for(int i=0;i<24;++i){const int n=i==0?0:((i+1)/2)*(i%2?1:-1);const float a=preferred+n*6.2831853f/24;
                Vec3 candidate=lot.position+Vec3{std::sin(a)*1.75f,0,std::cos(a)*1.75f};
                if(clear(h.robot,candidate)){h.service=candidate;h.service_valid=true;break;}
            }
            if(!h.service_valid){stop(h.robot);continue;}
        }
        Vec3 goal=h.phase==0?h.service:h.phase==1?trader_.dock:h.exit;
        if(h.phase==1&&owner!=h.robot){
            const int rank=1+(int)h.robot;const Vec3 queue=trader_.dock+out*(3.4f+2.6f*(float)(rank/2))+side*(rank%2?3.2f:-3.2f);
            r.action=BuilderRobotAction::None;if(clear(h.robot,queue))move(h.robot,queue);else stop(h.robot);continue;
        }
        if(h.phase==2){
            r.action=BuilderRobotAction::None;
            if(distance(r.position,goal)>.28f){move(h.robot,goal);continue;}
            stop(h.robot);r.mode=BuilderRobotMode::Idle;r.action=BuilderRobotAction::None;r.interaction_position_valid=false;finished.push_back(h.robot);continue;
        }
        r.interaction_position=goal;r.interaction_focus_position=h.phase==0?lot.position:trader_.dock;r.interaction_position_valid=true;
        if(distance(r.position,goal)>.30f){r.action=BuilderRobotAction::None;move(h.robot,goal);continue;}
        stop(h.robot);
        const auto action=h.phase==0?BuilderRobotAction::Pickup:BuilderRobotAction::PutDown;
        if(r.action!=action){r.action=action;r.action_elapsed=0;r.action_duration=2;r.action_material=(std::size_t)lot.good>=kLegacyTradeGoodCount?
            ore::materials[(std::size_t)lot.good-kLegacyTradeGoodCount]:
            (MaterialResource)std::min<std::size_t>((std::size_t)lot.good,kMaterialResourceCount-1);r.action_amount=h.phase==0?std::min<std::int64_t>(lot.remaining,r.payload_capacity_kg):h.cargo;r.action_focus_position=r.interaction_focus_position;}
        r.action_elapsed+=dt;if(r.action_elapsed<2)continue;
        if(h.phase==0){
            h.cargo=std::min<std::int64_t>(lot.remaining,r.payload_capacity_kg);lot.remaining-=h.cargo;
            record_robot_operation_result(r,h.cargo>0,trader_.pad_sequence);
            r.action=BuilderRobotAction::None;
            if(!h.cargo){r.mode=BuilderRobotMode::Idle;finished.push_back(h.robot);continue;}
            h.phase=1;h.service_valid=false;
        }else{
            // Do not drop cargo until a clear egress exists. Shipment custody
            // changes only once, when the physical put-down finishes.
            bool exit_valid=false;
            for(const auto p:std::array<Vec3,3>{{trader_.dock+out*3+side*3,trader_.dock+out*3-side*3,trader_.dock+out*4}})
                if(clear(h.robot,p)){h.exit=p;exit_valid=true;break;}
            record_robot_operation_result(r,exit_valid,trader_.pad_sequence);
            r.action=BuilderRobotAction::None;
            if(!exit_valid) continue;
            trader_.loaded[(std::size_t)lot.good]+=h.cargo;h.cargo=0;h.phase=2;
        }
    }
    std::erase_if(trader_.hauls,[&](const auto& h){return std::find(finished.begin(),finished.end(),h.robot)!=finished.end();});
}
void GameApp::render_pad_trade(eng::Renderer& r,std::size_t index,eng::Rect p,float scroll,float extra){
    using namespace eng;using namespace layout;
    if(active_game_phase_!=GamePhase::RobotPreparation||index>=placed_resources_.size())return;
    const auto& pad=placed_resources_[index];const bool own=trader_.active()&&pad.placement_sequence==trader_.pad_sequence;
    if(!own&&(!pad.construction.complete()||pad.supply.power_fraction<.5))return;
    const auto visible=[&](float y,float h){return y>=p.y+4&&y+h<=p.y+p.h-4;};
    const auto y=[&](float offset){return p.y+offset+extra-scroll;};
    const auto label=[&](float offset,const std::string& text,float scale=.43f){float yy=y(offset);if(visible(yy,14))game_label_fitted(r,{p.x+14,yy,p.w-28,14},text,scale,theme_);};
    Rect call{p.x+14,y(350),p.w-28,29};
    if(!own && !trader_.active() && !trade_antenna_enabled()){
        const bool built=std::any_of(placed_resources_.begin(),placed_resources_.end(),[](const auto& item){
            return item.kind==ResourceKind::Antenna && item.construction.complete();
        });
        label(356,built?"POWER ON THE ANTENNA TO CALL A TRADER":"BUILD AN ANTENNA TO CALL A TRADER",.39f);
        return;
    }
    if(visible(call.y,call.h)){
        if(own)r.draw_line({call.x,call.y-2},{call.x+call.w,call.y-2},theme_.accent,3);
        if(ui::button(r,ui_input_,call,own?"[ACTIVE] TRADE":"TRADE",theme_,!trader_.active())){call_trader(index);return;}
    }
    label(390,"CASH EUR "+std::to_string(planning_.funds_eur)+"   LAST SALE "+std::to_string(trader_.last_payment_eur),.39f);
    if(!own){if(trader_.active())label(412,"TRADER IS SERVING ANOTHER PAD",.38f);else if(!trade_message_.empty())label(412,trade_message_,.36f);return;}
    label(416,trade_phase_name(trader_.phase),.43f);
    const bool edit=trader_.phase==TradePhase::Approach||trader_.phase==TradePhase::Docked;
    if(!edit){
        label(442,"CARGO STILL TO LOAD",.46f);
        float row=470;
        for(std::size_t i=0;i<kTradeGoodCount;++i){
            const auto remaining=std::max<std::int64_t>(0,trader_.requested[i]-trader_.loaded[i]);
            if(!remaining) continue;
            label(row,std::string(trade_good_name((TradeGood)i))+"   "+std::to_string(remaining)+" KG REMAINING",.40f);
            row+=20;
        }
        if(row==470){label(row,"ALL CARGO ABOARD",.43f);row+=20;}
        label(row+4,"LOADED "+std::to_string(mass(trader_.loaded))+" / "+std::to_string(mass(trader_.requested))+" KG",.40f);
        label(row+26,trader_.phase==TradePhase::Loading&&!trader_powered()?"WAITING FOR LANDING PAD POWER":trade_message_,.37f);
        return;
    }
    const auto available=trader_available();
    for(std::size_t i=0;i<kTradeGoodCount;++i){
        const float x=p.x+14+(p.w-28)*.5f*(float)(i%2),yy=y(446+54*(float)(i/2));const float width=(p.w-28)*.5f-6;
        if(!visible(yy,49))continue;
        r.draw_rect({x,yy,width,49},theme_.panel2);
        const std::string title=std::string(trade_good_name((TradeGood)i))+"  "+std::to_string(trade_price_eur((TradeGood)i))+" EUR/KG";
        game_label_fitted(r,{x+4,yy+3,width-8,12},title,.29f,theme_);
        const auto q=trader_.requested[i];
        const std::int64_t unit=i>=kLegacyTradeGoodCount?ore::box_kg:1;
        const std::int64_t step=i<kMaterialResourceCount?100:unit;
        game_label_fitted(r,{x+29,yy+23,63,13},std::to_string(edit?q:trader_.loaded[i])+" / "+std::to_string(edit?available[i]:q),.28f,theme_,true);
        if(ui::button(r,ui_input_,{x+2,yy+20,24,24},"-",theme_,edit&&q>0))trader_.requested[i]=std::max<std::int64_t>(0,q-step);
        const auto room=std::max<std::int64_t>(0,kTraderCapacityKg-mass(trader_.requested));
        const auto add=std::min(std::max<std::int64_t>(0,available[i]-q),room)/unit*unit;
        if(ui::button(r,ui_input_,{x+94,yy+20,24,24},"+",theme_,edit&&add>0))trader_.requested[i]+=std::min(step,add);
        if(ui::button(r,ui_input_,{x+122,yy+20,width-124,24},"ALL",theme_,edit&&add>0))trader_.requested[i]+=add;
    }
    std::int64_t value=0;for(std::size_t i=0;i<kTradeGoodCount;++i)value+=trader_.requested[i]*trade_price_eur((TradeGood)i);
    label(725+kTradeManifestExtraHeight,"CARGO "+std::to_string(mass(trader_.requested))+" / 20000 KG   SALE EUR "+std::to_string(value),.39f);
    Rect load{p.x+14,y(750+kTradeManifestExtraHeight),p.w-28,29};
    if(visible(load.y,load.h)&&ui::button(r,ui_input_,load,"START LOADING",theme_,trader_.phase==TradePhase::Docked&&trader_powered()&&mass(trader_.requested)>0)){start_trader_sale();return;}
    Rect dismiss{p.x+14,y(790+kTradeManifestExtraHeight),p.w-28,27};
    if(visible(dismiss.y,dismiss.h)&&ui::button(r,ui_input_,dismiss,"SEND TRADER AWAY",theme_,trader_.phase==TradePhase::Docked)){trader_.requested={};trader_.sale_value_eur=0;trader_.phase=TradePhase::Departing;trader_.elapsed=0;return;}
    label(830+kTradeManifestExtraHeight,trader_powered()?"RESERVED STOCK -> BELTS OR ROBOTS -> SHIP":"WAITING FOR LANDING PAD POWER",.35f);
    label(851+kTradeManifestExtraHeight,trade_message_,.34f);
}
} // namespace game
