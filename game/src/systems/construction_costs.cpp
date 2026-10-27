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

constexpr int corridor_iron[5]   ={48,30,15,5,2};
constexpr int corridor_copper[5] ={0,4,10,54,32};
constexpr int corridor_silicon[5]={0,0,4,20,76};
constexpr int corridor_plastic[5]={12,24,40,18,6};

constexpr int supply_iron[3]   ={68,25,7};
constexpr int supply_copper[3] ={5,70,25};
constexpr int supply_silicon[3]={0,15,85};
constexpr int supply_plastic[3]={30,55,15};

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

namespace {
struct RouteMetrics {
    double length_m{0.0};
    double quarter_turns{0.0}; // accumulated turn angle expressed as 90-degree bends
};

RouteMetrics route_metrics(const std::vector<eng::Vec3>& path){
    RouteMetrics out{};
    if(path.size()<2) return out;
    for(std::size_t i=0;i+1<path.size();++i){
        const double dx=(double)path[i+1].x-(double)path[i].x;
        const double dz=(double)path[i+1].z-(double)path[i].z;
        out.length_m+=std::sqrt(dx*dx+dz*dz);
    }
    for(std::size_t i=1;i+1<path.size();++i){
        double ax=(double)path[i].x-(double)path[i-1].x;
        double az=(double)path[i].z-(double)path[i-1].z;
        double bx=(double)path[i+1].x-(double)path[i].x;
        double bz=(double)path[i+1].z-(double)path[i].z;
        const double al=std::sqrt(ax*ax+az*az), bl=std::sqrt(bx*bx+bz*bz);
        if(al<1.0e-5 || bl<1.0e-5) continue;
        ax/=al; az/=al; bx/=bl; bz/=bl;
        const double dot=std::clamp(ax*bx+az*bz,-1.0,1.0);
        out.quarter_turns+=std::acos(dot)/(3.14159265358979323846*0.5);
    }
    return out;
}

ConstructionResourcePlan corridor_from_metrics(double length_m,double quarter_turns,std::size_t network_junctions){
    const double m=std::max(0.5,length_m);
    // Pressurised corridor mass model (kg-equivalent): steel frame/shell and floor,
    // copper power/data trunks, silicon controls/sensors and polymer pressure seals.
    // Two terminal collars are always required. Curvature adds reinforced ribs;
    // branches into an existing corridor add a heavier pressure-rated junction fitting.
    const double terminal_collars=2.0;
    ResourceBundle total{
        kg(m*112.0 + terminal_collars*46.0 + quarter_turns*36.0 + (double)network_junctions*66.0),
        kg(m*3.8  + terminal_collars*1.6  + quarter_turns*1.2  + (double)network_junctions*2.2),
        kg(m*0.8  + terminal_collars*0.5  + quarter_turns*0.5  + (double)network_junctions*1.0),
        kg(m*19.0 + terminal_collars*6.5  + quarter_turns*7.0  + (double)network_junctions*10.0)
    };
    return distribute(kCorridorConstructionSteps,total,corridor_iron,corridor_copper,corridor_silicon,corridor_plastic);
}

ConstructionResourcePlan supply_from_metrics(double length_m,double quarter_turns,std::size_t junctions){
    const double m=std::max(0.5,length_m);
    // Shared utility tray: iron support rail / pipe brackets, copper conductors,
    // silicon monitoring/control hardware and polymer water/insulation lines.
    // Every route has two terminal fittings; bends and manifold intersections
    // add local hardware instead of being treated as free geometry.
    const double terminals=2.0;
    ResourceBundle total{
        kg(m*8.5 + terminals*8.0 + quarter_turns*3.5 + (double)junctions*10.0),
        kg(m*5.8 + terminals*2.0 + quarter_turns*1.2 + (double)junctions*5.0),
        kg(m*0.55+ terminals*0.4 + quarter_turns*0.25+ (double)junctions*1.8),
        kg(m*3.6 + terminals*1.5 + quarter_turns*1.4 + (double)junctions*3.5)
    };
    return distribute(kSupplyConstructionSteps,total,supply_iron,supply_copper,supply_silicon,supply_plastic);
}
} // namespace

ConstructionResourcePlan corridor(float length_m){
    return corridor_from_metrics((double)length_m,0.0,0);
}

ConstructionResourcePlan corridor(const std::vector<eng::Vec3>& path,std::size_t network_junctions){
    const RouteMetrics metrics=route_metrics(path);
    return corridor_from_metrics(metrics.length_m,metrics.quarter_turns,network_junctions);
}

ConstructionResourcePlan supply_channel(float length_m){
    return supply_from_metrics((double)length_m,0.0,0);
}

ConstructionResourcePlan supply_channel(const std::vector<eng::Vec3>& path,std::size_t junctions){
    const RouteMetrics metrics=route_metrics(path);
    return supply_from_metrics(metrics.length_m,metrics.quarter_turns,junctions);
}

ConstructionResourcePlan walkway(float length_m){
    const double m=std::max(0.5,(double)length_m);
    // 1.90 m wide open deck with beams/grating; no railing mass is included.
    const ResourceBundle total{kg(m*44.0+28.0),kg(m*1.4+5.0),kg(m*0.22+2.0),kg(m*1.8+5.0)};
    return distribute(kWalkwayConstructionSteps,total,walkway_iron,walkway_copper,walkway_silicon,walkway_plastic);
}

} // namespace game::construction_costs
