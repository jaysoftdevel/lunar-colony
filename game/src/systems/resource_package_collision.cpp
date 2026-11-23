#include "game/game_app.hpp"
#include "game/world/ore_stockyard.hpp"
namespace game {
using namespace runtime;
bool GameApp::nearby_resource_packages(Vec3 p,float radius,const std::function<bool(const packages::Box&)>& emit) const {
    for(const auto& pile:ground_resource_piles_){
        const float half=resource_package_half_width(pile.resource,pile.kind==GroundPileKind::Loose?pile.amount:std::min(pile.amount,resource_package_capacity_kg(pile.resource)));
        if(std::abs(p.x-pile.position.x)>half+radius||std::abs(p.z-pile.position.z)>half+radius)continue;
        if(!packages::each(pile,emit))return false;
    }
    for(std::size_t i=0;i<placed_resources_.size();++i){const auto& item=placed_resources_[i];
        if(!item.construction.complete())continue;
        if(item.kind==ResourceKind::StorageDome){
            const auto& mesh=resource_models_.mesh(item);
            const auto local=world_math::rotate_y(p-item.position,-item.rotation_degrees);
            if(std::abs(local.x)>mesh.footprint_x*.5f+radius||std::abs(local.z)>mesh.footprint_z*.5f+radius)continue;
            std::size_t count=item.stored_boxes.size();for(auto n:item.stored_ore_boxes)count+=std::min(n,kVisibleOreBoxesPerType);
            for(std::size_t n=0;n<count;++n){Vec3 base;float half=0,height=0;stored_resource_box_pose(item,mesh,n,base,half,height);
                if(!emit({base,half,height,MaterialResource::Iron}))return false;
            }
        }else if(item.kind==ResourceKind::OreDriller&&world_math::distance_xz(p,item.position)<32+radius){
            const auto pad=driller_stockyard(item);
            for(std::size_t k=0;pad&&k<ore::materials.size();++k)for(int n=0;n<ore::boxes(item.mined_ore_kg[k]);++n)
                if(!emit({pad->box(k,n),.20f,.31f,ore::materials[k]}))return false;
        }
    }
    for(const auto& link:utility_network_.connections())if(link.kind==UtilityKind::ConveyorBelt&&link.construction.complete()&&link.ore_cargo<ore::no_cargo){
        const auto base=link.ore_box_position();
        if(std::abs(p.x-base.x)<.23f+radius&&std::abs(p.z-base.z)<.23f+radius&&!emit({base,.23f,.34f,ore::materials[link.ore_cargo]}))return false;
    }
    return true;
}
bool GameApp::resource_packages_block(Vec3 p) const {
    return !nearby_resource_packages(p,first_person_.body_radius(),[&](const auto& box){return !box.intersects(p,first_person_.body_radius(),first_person_.body_height());});
}
float GameApp::resource_package_support(Vec3 p) const {
    float floor=0;
    nearby_resource_packages(p,.03f,[&](const auto& box){
        if(std::abs(p.x-box.base.x)<=box.half-.01f&&std::abs(p.z-box.base.z)<=box.half-.01f&&box.top()<=p.y+.18f)floor=std::max(floor,box.top());
        return true;
    });
    return floor;
}
}

