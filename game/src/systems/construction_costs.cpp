#include "game/systems/construction_costs.hpp"
#include "game/world/construction.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace game::construction_costs {
namespace {

std::int64_t kg(double value){ return std::max<std::int64_t>(0,(std::int64_t)std::llround(value)); }

void split_material(std::int64_t total,const int* weights,std::size_t count,
                    MaterialResource material,ConstructionResourcePlan& plan){
    if(total<=0 || count==0) return;
    int weight_total=0;
    for(std::size_t i=0;i<count;++i) weight_total+=std::max(0,weights[i]);
    if(weight_total<=0){ plan.steps[count-1].amount(material)+=total; return; }

    std::int64_t assigned=0;
    for(std::size_t i=0;i<count;++i){
        const std::int64_t part=(total*(std::int64_t)std::max(0,weights[i]))/(std::int64_t)weight_total;
        plan.steps[i].amount(material)+=part;
        assigned+=part;
    }
    plan.steps[count-1].amount(material)+=total-assigned;
}

ConstructionResourcePlan distribute(std::uint8_t steps,const ResourceBundle& total,
                                    const int* iron,const int* copper,const int* silicon,const int* plastic){
    ConstructionResourcePlan plan;
    plan.total_steps=steps;
    const std::size_t n=std::min<std::size_t>(steps,kMaxConstructionResourceSteps);
    split_material(total.iron,iron,n,MaterialResource::Iron,plan);
    split_material(total.copper,copper,n,MaterialResource::Copper,plan);
    split_material(total.silicon,silicon,n,MaterialResource::Silicon,plan);
    split_material(total.plastic,plastic,n,MaterialResource::Plastic,plan);
    return plan;
}

ResourceBundle scaled(ResourceBundle base,double iron,double copper,double silicon,double plastic){
    base.iron=kg((double)base.iron*iron);
    base.copper=kg((double)base.copper*copper);
    base.silicon=kg((double)base.silicon*silicon);
    base.plastic=kg((double)base.plastic*plastic);
    return base;
}

constexpr int habitat_iron[10]   ={18,18,14,12,10,8,6,5,5,4};
constexpr int habitat_copper[10] ={1,2,3,5,8,15,20,18,17,11};
constexpr int habitat_silicon[10]={0,0,1,2,5,12,20,23,22,15};
constexpr int habitat_plastic[10]={4,6,10,12,14,15,15,12,8,4};

constexpr int structure_iron[10]   ={20,20,16,12,10,8,6,4,3,1};
constexpr int structure_copper[10] ={2,3,5,8,12,17,18,16,12,7};
constexpr int structure_silicon[10]={0,1,2,5,9,14,19,21,18,11};
constexpr int structure_plastic[10]={8,10,12,14,14,13,11,8,6,4};

constexpr int corridor_iron[5]   ={25,25,22,18,10};
constexpr int corridor_copper[5] ={5,15,25,30,25};
constexpr int corridor_silicon[5]={0,5,20,35,40};
constexpr int corridor_plastic[5]={10,30,30,20,10};

constexpr int supply_iron[3]   ={45,35,20};
constexpr int supply_copper[3] ={25,45,30};
constexpr int supply_silicon[3]={10,35,55};
constexpr int supply_plastic[3]={35,40,25};

constexpr int walkway_iron[3]   ={55,30,15};
constexpr int walkway_copper[3] ={15,45,40};
constexpr int walkway_silicon[3]={5,30,65};
constexpr int walkway_plastic[3]={25,45,30};

} // namespace

ConstructionResourcePlan habitat(HabitatKind kind,HabitatSizeTier,const HabitatMesh& mesh){
    const double area=(double)mesh.footprint_x*(double)mesh.footprint_z;
    const double perimeter=2.0*((double)mesh.footprint_x+(double)mesh.footprint_z);
    const double envelope=area+perimeter*(double)mesh.height;

    // kg-equivalent prefabricated lunar modules: structural frame + pressure
    // shell, wiring/electronics, and polymer insulation/interior panels.
    ResourceBundle total{
        kg(area*28.0+perimeter*(double)mesh.height*9.0),
        kg(area*1.20+(double)mesh.height*7.0),
        kg(area*0.42+(double)mesh.height*2.5),
        kg(envelope*1.85)
    };

    switch(kind){
        case HabitatKind::AirLock: total=scaled(total,1.00,1.00,1.00,1.00); break;
        case HabitatKind::OxygenGeneratorDome: total=scaled(total,0.90,1.35,1.30,1.45); break;
        case HabitatKind::CantineDome: total=scaled(total,0.92,1.05,0.90,1.35); break;
        case HabitatKind::BioDome: total=scaled(total,0.85,1.25,1.20,1.60); break;
        case HabitatKind::CrewQuarters: total=scaled(total,0.92,1.05,1.00,1.45); break;
        case HabitatKind::ProcessingFacility: total=scaled(total,1.45,1.35,1.20,1.00); break;
        case HabitatKind::Factory: total=scaled(total,1.60,1.45,1.40,1.00); break;
        case HabitatKind::Manufactory: total=scaled(total,1.75,1.55,1.55,1.05); break;
        case HabitatKind::SickBay: total=scaled(total,1.00,1.20,1.55,1.45); break;
        case HabitatKind::Laboratory: total=scaled(total,1.05,1.30,1.80,1.30); break;
        case HabitatKind::Count: break;
    }
    return distribute(kConstructionSteps,total,habitat_iron,habitat_copper,habitat_silicon,habitat_plastic);
}

ConstructionResourcePlan structure(ResourceKind kind,const ResourceMesh& mesh){
    const double area=(double)mesh.footprint_x*(double)mesh.footprint_z;
    const double volume_proxy=area*std::max(1.0,(double)mesh.height);
    ResourceBundle total{
        kg(area*22.0+(double)mesh.height*58.0),
        kg(area*0.72+(double)mesh.height*3.8),
        kg(area*0.22+(double)mesh.height*1.7),
        kg(area*0.70+(double)mesh.height*2.0)
    };

    switch(kind){
        case ResourceKind::SolarPanel: total=scaled(total,0.70,1.65,2.60,1.10); break;
        case ResourceKind::WaterDriller: total=scaled(total,1.45,1.20,1.15,0.95); break;
        case ResourceKind::PowerStorage: total=scaled(total,1.10,2.10,1.80,1.25); break;
        case ResourceKind::WaterStorage: total=scaled(total,1.20,0.70,0.55,1.10); break;
        case ResourceKind::OreDriller: total=scaled(total,1.80,1.45,1.20,0.90); break;
        case ResourceKind::WaterPurifier: total=scaled(total,1.15,1.35,1.55,1.50); break;
        case ResourceKind::Antenna: total=scaled(total,0.85,1.55,2.20,0.85); break;
        case ResourceKind::LandingPad:
            total.iron=kg(area*36.0+volume_proxy*0.10);
            total.copper=kg(area*0.18);
            total.silicon=kg(area*0.08);
            total.plastic=kg(area*0.24);
            break;
        case ResourceKind::StorageDome:
            // Large reinforced shell, cargo door/handling rails, inventory electronics and liners.
            total.iron=kg(area*31.0+(double)mesh.height*90.0);
            total.copper=kg(area*0.85+(double)mesh.height*5.0);
            total.silicon=kg(area*0.32+(double)mesh.height*2.8);
            total.plastic=kg(area*1.15+(double)mesh.height*4.2);
            break;
        case ResourceKind::Count: break;
    }
    return distribute(kConstructionSteps,total,structure_iron,structure_copper,structure_silicon,structure_plastic);
}

ConstructionResourcePlan corridor(float length_m){
    const double m=std::max(0.5,(double)length_m);
    const ResourceBundle total{kg(m*82.0+70.0),kg(m*4.0+12.0),kg(m*1.0+6.0),kg(m*18.0+25.0)};
    return distribute(kCorridorConstructionSteps,total,corridor_iron,corridor_copper,corridor_silicon,corridor_plastic);
}

ConstructionResourcePlan supply_channel(float length_m){
    const double m=std::max(0.5,(double)length_m);
    const ResourceBundle total{kg(m*7.0+16.0),kg(m*5.0+12.0),kg(m*0.7+4.0),kg(m*2.5+8.0)};
    return distribute(kSupplyConstructionSteps,total,supply_iron,supply_copper,supply_silicon,supply_plastic);
}

ConstructionResourcePlan walkway(float length_m){
    const double m=std::max(0.5,(double)length_m);
    // 1.56 m wide open deck with beams/grating; no railing mass is included.
    const ResourceBundle total{kg(m*36.0+24.0),kg(m*1.2+5.0),kg(m*0.2+2.0),kg(m*1.5+5.0)};
    return distribute(kWalkwayConstructionSteps,total,walkway_iron,walkway_copper,walkway_silicon,walkway_plastic);
}

} // namespace game::construction_costs
