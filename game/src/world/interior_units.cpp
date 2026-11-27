#include "game/world/interior_unit_mesh.hpp"
#include "game/world/habitat_builder.hpp"
namespace game {
const InteriorUnitProfile& interior_unit_profile(InteriorUnitKind kind){
    static const std::array<InteriorUnitProfile,static_cast<std::size_t>(InteriorUnitKind::Count)> profiles{{
        {"METAL FABRICATOR",{1.80f,1.85f,1.20f},{12,3,0,2,0,0,0,0},3.f,.25,{},MaterialResource::Iron,3},
        {"PLASTIC FABRICATOR",{1.70f,1.80f,1.10f},{10,2,2,2,0,0,0,0},2.f,.50,{0,0,1,0,2,0,0,0},MaterialResource::Plastic,2},
        {"ORE REFINERY",{2.f,2.10f,1.40f},{16,4,0,2,0,0,0,2},4.f,.20,{},MaterialResource::Iron,3},
        {"MEDICAL SYNTHESIZER",{1.35f,1.75f,.90f},{8,2,2,3,0,0,0,0},1.5f,1.,{0,0,0,1,1,1,0,0},MaterialResource::Medical,1},
        {"HYDROPONIC RACK",{1.20f,1.50f,.80f},{5,1,0,3,0,0,0,0},0.f,0.,{},MaterialResource::Food,0},
        {"SERVICE TERMINAL",{.90f,1.20f,.60f},{4,1,1,0,0,0,0,0},.1f,0.,{},MaterialResource::Iron,0},
        {"FOOD PROCESSOR",{1.60f,1.70f,1.0f},{8,2,1,3,0,0,0,0},1.f,.10,{0,0,0,0,1,2,0,0},MaterialResource::Food,2},
        {"DRINKING WELL",{.80f,1.40f,.70f},{4,1,0,2,0,0,0,0},.08f,0.,{},MaterialResource::Water,0},
        {"DINING TABLE",{1.60f,.78f,.90f},{4,0,0,2,0,0,0,0},0.f,0.,{},MaterialResource::Iron,0},
        {"SEATING BENCH",{1.50f,.85f,.55f},{3,0,0,1,0,0,0,0},0.f,0.,{},MaterialResource::Iron,0},
        {"OXYGEN GENERATOR",{3.f,2.70f,1.40f},{24,6,4,4,0,0,0,4},6.f,0.,{},MaterialResource::Iron,0}
    }};
    return profiles.at(static_cast<std::size_t>(kind));
}
const InteriorUnitProfile& interior_unit_profile(InteriorUnitKind kind,InteriorUnitSize size){
    if(kind!=InteriorUnitKind::OxygenGenerator)return interior_unit_profile(kind);
    static const auto variants=[] {
        std::array<InteriorUnitProfile,3> out;
        constexpr float scales[]{.55f,.75f,1.f};
        constexpr const char* names[]{"VERY SMALL OXYGEN GENERATOR","SMALL OXYGEN GENERATOR","MEDIUM OXYGEN GENERATOR"};
        for(std::size_t i=0;i<out.size();++i){auto p=interior_unit_profile(InteriorUnitKind::OxygenGenerator);const auto tier=InteriorUnitSize(i);const auto factor=oxygen_unit_factor(tier);
            p.name=names[i];p.size=p.size*scales[i];p.power_kw*=factor;
            for(std::size_t m=0;m<kMaterialResourceCount;++m){auto& n=p.cost.amount(MaterialResource(m));n=std::int64_t(std::ceil(double(n)*factor));}
            out[i]=p;
        }return out;
    }();
    return variants.at(std::size_t(size));
}
const char* interior_unit_recipe(InteriorUnitKind kind,InteriorUnitSize size){
    if(kind!=InteriorUnitKind::OxygenGenerator)return interior_unit_recipe(kind);
    switch(size){
        case InteriorUnitSize::VerySmall:return "9 KG O2/D / 10.13 L WATER/D / 1.5 KW";
        case InteriorUnitSize::Small:return "18 KG O2/D / 20.25 L WATER/D / 3 KW";
        default:return "36 KG O2/D / 40.50 L WATER/D / 6 KW";
    }
}
const char* interior_unit_recipe(InteriorUnitKind kind){
    switch(kind){
        case InteriorUnitKind::MetalFabricator:return "5 KG RAW ORE -> 3 KG REFINED / 0.25 DAY";
        case InteriorUnitKind::PlasticFabricator:return "1 SI + 2 WATER -> 2 PLASTIC / 0.5 DAY";
        case InteriorUnitKind::OreRefinery:return "5 KG RAW ORE -> 3 KG REFINED / 0.20 DAY";
        case InteriorUnitKind::MedicalSynthesizer:return "1 FOOD + 1 WATER + 1 PLASTIC -> 1 MED / DAY";
        case InteriorUnitKind::HydroponicRack:return "ONE PLANTABLE GROW BAY / CROP-SPECIFIC LOAD";
        case InteriorUnitKind::FoodProcessor:return "2 FOOD + 1 WATER -> 2 KG PREPARED MEALS / 0.1 DAY";
        case InteriorUnitKind::DrinkingWell:return "NETWORK WATER -> 10 L DRINKING RESERVE";
        case InteriorUnitKind::DiningTable:return "PASSIVE DINING SURFACE / FOUR PLACES";
        case InteriorUnitKind::SeatingBench:return "PASSIVE SEATING / TWO PLACES";
        case InteriorUnitKind::OxygenGenerator:return "36 KG O2 / DAY / 40.50 L WATER / 6 KW";
        case InteriorUnitKind::ServiceTerminal:return "LIVE ROOM AIR / POWER / WATER TERMINAL";
        default:return "";
    }
}
bool interior_unit_allowed(HabitatKind host,InteriorUnitKind kind){
    if(host==HabitatKind::AirLock)return false; // Fixed pressure-system fit.
    switch(kind){
        case InteriorUnitKind::MetalFabricator:return host==HabitatKind::Factory||host==HabitatKind::Manufactory;
        case InteriorUnitKind::PlasticFabricator:return host==HabitatKind::Factory||host==HabitatKind::Manufactory||host==HabitatKind::ProcessingFacility;
        case InteriorUnitKind::OreRefinery:return host==HabitatKind::ProcessingFacility;
        case InteriorUnitKind::MedicalSynthesizer:return host==HabitatKind::Laboratory||host==HabitatKind::SickBay;
        case InteriorUnitKind::HydroponicRack:return host==HabitatKind::BioDome;
        case InteriorUnitKind::ServiceTerminal:return true;
        case InteriorUnitKind::FoodProcessor:case InteriorUnitKind::DrinkingWell:
        case InteriorUnitKind::DiningTable:case InteriorUnitKind::SeatingBench:return host==HabitatKind::CantineDome;
        case InteriorUnitKind::OxygenGenerator:return host==HabitatKind::OxygenGeneratorDome;
        default:return false;
    }
}
const HabitatMesh& interior_unit_mesh(InteriorUnitKind kind){
    using namespace runtime;
    static const auto models=[]{
        std::array<HabitatMesh,static_cast<std::size_t>(InteriorUnitKind::Count)> result;
        for(std::size_t i=0;i<result.size();++i){
            const auto type=static_cast<InteriorUnitKind>(i);const auto size=interior_unit_profile(type).size;
            habitats::Builder b;
            b.box(b.mesh.interior,{0,.08f,0},{size.x,.16f,size.z},false);
            for(float x:{-1.f,1.f})for(float z:{-1.f,1.f})
                b.box(b.mesh.service_metal,{x*size.x*.40f,.24f,z*size.z*.36f},{.10f,.32f,.10f},false);
            if(type==InteriorUnitKind::DiningTable||type==InteriorUnitKind::SeatingBench){
                const float top=type==InteriorUnitKind::DiningTable?.74f:.45f;
                b.box(b.mesh.interior,{0,top,0},{size.x,.08f,size.z},false);
                for(float x:{-.40f,.40f})for(float z:{-.35f,.35f})
                    b.box(b.mesh.service_metal,{x*size.x,top*.5f,z*size.z},{.06f,top,.06f},false);
                if(type==InteriorUnitKind::SeatingBench){
                    for(float x:{-.60f,.60f})b.box(b.mesh.service_metal,{x,.64f,.23f},{.05f,.40f,.05f},false);
                    b.box(b.mesh.fabric,{0,.72f,.23f},{size.x,.25f,.07f},false);
                }
                b.mesh.footprint_x=size.x;b.mesh.footprint_z=size.z;b.mesh.height=size.y;
                result[i]=std::move(b.mesh);continue;
            }
            b.box(b.mesh.interior,{0,.52f,0},{size.x*.90f,.55f,size.z*.85f},false);
            for(float x:{-1.f,1.f})b.box(b.mesh.interior,{x*size.x*.45f,size.y*.60f,0},{.11f,size.y*.65f,size.z*.90f},false);
            b.box(b.mesh.interior,{0,size.y-.08f,0},{size.x,.16f,size.z},false);
            if(type==InteriorUnitKind::FoodProcessor){
                b.box(b.mesh.service_metal,{0,.91f,0},{1.42f,.06f,.86f},false);
                b.box(b.mesh.interior,{-.32f,1.23f,.10f},{.70f,.58f,.62f},false);
                b.box(b.mesh.service_metal,{-.32f,1.20f,-.22f},{.49f,.27f,.03f},false);
                b.cylinder_y(b.mesh.service_metal,{.43f,1.03f,-.05f},.15f,.19f,20,false);
            }else if(type==InteriorUnitKind::DrinkingWell){
                b.cylinder_y(b.mesh.service_metal,{0,1.13f,0},.08f,.27f,24,false);
                b.cylinder_between(b.mesh.service_metal,{.16f,1.13f,.13f},{.16f,1.35f,.13f},.025f,12,true,false);
                b.cylinder_between(b.mesh.service_metal,{.16f,1.35f,.13f},{0,1.35f,.13f},.025f,12,true,false);
                b.cylinder_y(b.mesh.fabric,{0,.76f,.03f},.55f,.19f,16,false);
            }else if(type==InteriorUnitKind::OxygenGenerator){
                b.box(b.mesh.interior,{-.85f,1.22f,.15f},{.95f,1.85f,1.10f},false);
                for(float x:{.15f,.92f}){
                    b.cylinder_y(b.mesh.service_metal,{x,1.38f,.20f},2.25f,.29f,24,false);
                    for(float y:{.45f,1.35f,2.25f})b.cylinder_y(b.mesh.markings,{x,y,.20f},.06f,.31f,24,false);
                    b.cylinder_between(b.mesh.service_metal,{x,2.50f,.20f},{-.85f,2.50f,.20f},.045f,12,true,false);
                }
                for(int fin=0;fin<8;++fin)b.box(b.mesh.service_metal,{-.85f,1.10f+fin*.11f,-.48f},{.70f,.025f,.05f},false);
            }else if(type==InteriorUnitKind::MetalFabricator){
                for(int roller=0;roller<6;++roller){const float x=(roller-2.5f)*.20f;
                    b.cylinder_between(b.mesh.service_metal,{x,.91f,-.40f},{x,.91f,.40f},.065f,12,false,false);}
                b.box(b.mesh.service_metal,{0,1.30f,.15f},{.28f,.48f,.34f},false);
                b.box(b.mesh.markings,{0,.805f,-.45f},{1.30f,.018f,.13f},false);
            }else if(type==InteriorUnitKind::PlasticFabricator){
                b.cylinder_between(b.mesh.service_metal,{-.50f,1.12f,0},{.50f,1.12f,0},.16f,16,true,false);
                b.cylinder_y(b.mesh.interior,{-.36f,1.42f,0},.52f,.20f,16,false);
                for(int n=0;n<6;++n)b.box(b.mesh.service_metal,{-.48f+n*.16f,1.12f,.01f},{.035f,.42f,.40f},false);
            }else if(type==InteriorUnitKind::OreRefinery){
                for(float x:{-.38f,.38f})b.cylinder_y(b.mesh.service_metal,{x,1.22f,0},.85f,.27f,16,false);
                b.cylinder_between(b.mesh.service_metal,{-.38f,1.55f,0},{.38f,1.55f,0},.065f,10,true,false);
                b.box(b.mesh.markings,{0,.83f,-.55f},{1.4f,.02f,.14f},false);
            }else if(type==InteriorUnitKind::MedicalSynthesizer){
                for(float x:{-.27f,0.f,.27f}){
                    b.cylinder_y(b.mesh.service_metal,{x,1.14f,.06f},.55f,.085f,12,false);
                    b.cylinder_y(b.mesh.markings,{x,1.43f,.06f},.04f,.095f,12,false);
                }
            }else if(type==InteriorUnitKind::HydroponicRack){
                b.box(b.mesh.interior,{0,.88f,0},{size.x*.85f,.12f,size.z*.80f},false);
                for(int seed=0;seed<6;++seed){const float x=(seed%3-1)*.25f,z=(seed/3-.5f)*.27f;
                    b.cylinder_y(b.mesh.fabric,{x,1.02f,z},.25f,.085f,8,false);
                    b.box(b.mesh.markings,{x,1.15f,z},{.18f,.025f,.12f},false);
                }
                b.box(b.mesh.lamps,{0,size.y-.18f,0},{size.x*.75f,.025f,.06f},false);
            }
            b.mesh.footprint_x=size.x;b.mesh.footprint_z=size.z;b.mesh.height=size.y;
            result[i]=std::move(b.mesh);
        }
        return result;
    }();
    return models.at(static_cast<std::size_t>(kind));
}
const HabitatMesh& interior_unit_mesh(InteriorUnitKind kind,InteriorUnitSize size){
    if(kind!=InteriorUnitKind::OxygenGenerator)return interior_unit_mesh(kind);
    static const auto variants=[] {
        std::array<HabitatMesh,3> out;constexpr float scales[]{.55f,.75f,1.f};
        for(std::size_t i=0;i<out.size();++i){auto mesh=interior_unit_mesh(InteriorUnitKind::OxygenGenerator);const float scale=scales[i];
            for(auto* layer:{&mesh.foundation,&mesh.interior,&mesh.details,&mesh.service_metal,&mesh.fabric,&mesh.displays,&mesh.lamps,&mesh.markings,&mesh.edges})for(auto& p:*layer)p=p*scale;
            for(auto& screen:mesh.screens){screen.center=screen.center*scale;screen.width*=scale;screen.height*=scale;}
            mesh.footprint_x*=scale;mesh.footprint_z*=scale;mesh.height*=scale;out[i]=std::move(mesh);
        }return out;
    }();return variants.at(std::size_t(size));
}
}
