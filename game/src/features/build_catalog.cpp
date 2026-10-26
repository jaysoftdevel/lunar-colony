#include "game/features/build_catalog.hpp"

namespace game {
BuildCatalog::BuildCatalog(){
    items_={
        {"habitat","Habitat","Crew",BuildIcon::Habitat},
        {"solar_array","Solar Array","Power",BuildIcon::Solar},
        {"water_tank","Water Tank","Water",BuildIcon::Water},
        {"storage","Storage","Logistics",BuildIcon::Storage},
        {"workshop","Workshop","Industry",BuildIcon::Workshop},
        {"farm","Farm","Food",BuildIcon::Farm}
    };
}

void BuildCatalog::select(int index){
    if(index>=0 && index<(int)items_.size()) selected_index_=index;
}
}
