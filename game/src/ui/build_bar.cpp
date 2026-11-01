#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include <algorithm>

namespace game {
using namespace eng;
using namespace layout;

namespace {
struct CategoryButton {
    BuildCategory category;
    BuildIcon icon;
    const char* name;
};
constexpr CategoryButton CATEGORIES[]={
    {BuildCategory::Habitats,BuildIcon::CategoryHabitats,"HABITATS / INFRASTRUCTURE"},
    {BuildCategory::Structures,BuildIcon::CategoryResources,"RESOURCES / FACILITIES"}
};
constexpr int CATEGORY_COUNT=(int)(sizeof(CATEGORIES)/sizeof(CATEGORIES[0]));
}

void GameApp::render_build_bar(Renderer& r){
    const float logical_w=ui_width(r);
    const float logical_h=ui_height(r);
    const float y=logical_h-BUILD_BAR_H;
    r.draw_rect({0,y,logical_w,BUILD_BAR_H},{0.025f,0.035f,0.045f,0.97f});
    r.draw_line({0,y},{logical_w,y},theme_.border,1.0f);

    constexpr float view_area_w=345.0f;
    constexpr float category_size=42.0f;
    constexpr float category_gap=7.0f;
    constexpr float category_x=12.0f;
    const float category_y=y+(BUILD_BAR_H-category_size)*0.5f;
    const float category_area_w=category_size*(float)CATEGORY_COUNT+category_gap*(float)(CATEGORY_COUNT-1)+28.0f;

    int hovered_category=-1;
    for(int i=0;i<CATEGORY_COUNT;++i){
        const Rect box={category_x+(category_size+category_gap)*(float)i,category_y,category_size,category_size};
        const bool hover=box.contains((float)ui_input_.mouse_x,(float)ui_input_.mouse_y);
        const bool selected=build_catalog_.category()==CATEGORIES[i].category;
        if(hover) hovered_category=i;
        r.draw_rect(box,hover?Color{0.11f,0.15f,0.18f,1.0f}:Color{0.055f,0.07f,0.09f,1.0f});
        const Color border=selected?theme_.accent:(hover?theme_.accent_hover:theme_.border);
        const float bw=selected?2.5f:1.3f;
        r.draw_line({box.x,box.y},{box.x+box.w,box.y},border,bw);
        r.draw_line({box.x+box.w,box.y},{box.x+box.w,box.y+box.h},border,bw);
        r.draw_line({box.x+box.w,box.y+box.h},{box.x,box.y+box.h},border,bw);
        r.draw_line({box.x,box.y+box.h},{box.x,box.y},border,bw);
        render_build_icon(r,CATEGORIES[i].icon,{box.x+7,box.y+7,box.w-14,box.h-14},selected?theme_.accent:theme_.text);
        if(hover && ui_input_.left_pressed){
            build_catalog_.set_category(CATEGORIES[i].category);
            build_catalog_scroll_=0;
            placement_rotation_degrees_=0.0f;
            placement_habitat_size_=HabitatSizeTier::Small;
            placement_resource_size_=ResourceSizeTier::Small;
            placement_storage_size_=StorageDomeSize::Small;
            storage_gate_placement_active_=false;
            storage_gate_placement_owner_=kNoRobotIndex;
            corridor_build_start_.reset();
            utility_build_start_.reset();
            walkway_build_start_.reset();
            walkway_build_target_.reset();
            corridor_link_source_=-1;
            clear_world_selection();
        }
    }

    const float build_left=category_x+category_area_w;
    const float build_right=logical_w-view_area_w-18.0f;
    const float build_area_w=std::max(0.0f,build_right-build_left);

    std::vector<int> visible;
    const auto& items=build_catalog_.items();
    for(size_t i=0;i<items.size();++i){
        if(items[i].category==build_catalog_.category()) visible.push_back((int)i);
    }

    const float icon_size=BUILD_ICON_SIZE,icon_gap=BUILD_ICON_GAP;
    const float natural=visible.empty()?0:(float)visible.size()*icon_size+(float)(visible.size()-1)*icon_gap;
    const bool paged=natural>build_area_w;
    const float arrow_space=paged?30.0f:0.0f;
    const float available=std::max(0.0f,build_area_w-2*arrow_space);
    const int capacity=std::max(1,(int)((available+icon_gap)/(icon_size+icon_gap)));
    const int maximum=std::max(0,(int)visible.size()-capacity);
    build_catalog_scroll_=std::clamp(build_catalog_scroll_,0,maximum);
    const bool over_catalog=Rect{build_left,y,build_area_w,BUILD_BAR_H}.contains((float)ui_input_.mouse_x,(float)ui_input_.mouse_y);
    if(paged && over_catalog && ui_input_.wheel!=0)build_catalog_scroll_=std::clamp(build_catalog_scroll_+(ui_input_.wheel<0?1:-1),0,maximum);
    if(paged){
        if(ui::button(r,ui_input_,{build_left,y+24,25,32},"<",theme_,build_catalog_scroll_>0))--build_catalog_scroll_;
        if(ui::button(r,ui_input_,{build_right-25,y+24,25,32},">",theme_,build_catalog_scroll_<maximum))++build_catalog_scroll_;
    }
    const int first=paged?build_catalog_scroll_:0;
    const int shown=std::min(capacity,(int)visible.size()-first);
    const float total=shown>0?(float)shown*icon_size+(float)(shown-1)*icon_gap:0;
    const float start_x=build_left+arrow_space+std::max(0.0f,(available-total)*0.5f);
    const float icon_y=y+(BUILD_BAR_H-icon_size)*0.5f;

    int hovered_item=-1;
    for(int slot=0;slot<shown;++slot){
        const int item_index=visible[(std::size_t)(first+slot)];
        const Rect box={start_x+(icon_size+icon_gap)*(float)slot,icon_y,icon_size,icon_size};
        const bool hover=box.contains((float)ui_input_.mouse_x,(float)ui_input_.mouse_y);
        const bool selected=item_index==build_catalog_.selected_index();
        if(hover) hovered_item=item_index;
        r.draw_rect(box,hover?Color{0.12f,0.16f,0.19f,1.0f}:Color{0.065f,0.08f,0.10f,1.0f});
        const Color border=selected?theme_.accent:(hover?theme_.accent_hover:theme_.border);
        const float bw=selected?3.0f:1.5f;
        r.draw_line({box.x,box.y},{box.x+box.w,box.y},border,bw);
        r.draw_line({box.x+box.w,box.y},{box.x+box.w,box.y+box.h},border,bw);
        r.draw_line({box.x+box.w,box.y+box.h},{box.x,box.y+box.h},border,bw);
        r.draw_line({box.x,box.y+box.h},{box.x,box.y},border,bw);
        render_build_icon(r,items[(size_t)item_index].icon,{box.x+8,box.y+8,box.w-16,box.h-16},selected?theme_.accent:theme_.text);
        if(hover && ui_input_.left_pressed){
            const int before=build_catalog_.selected_index();
            build_catalog_.select(item_index);
            if(build_catalog_.selected_index()!=before){
                placement_rotation_degrees_=0.0f;
                placement_habitat_size_=HabitatSizeTier::Small;
                placement_resource_size_=ResourceSizeTier::Small;
                placement_storage_size_=StorageDomeSize::Small;
                storage_gate_placement_active_=false;
            storage_gate_placement_owner_=kNoRobotIndex;
                corridor_build_start_.reset();
                utility_build_start_.reset();
                walkway_build_start_.reset();
                walkway_build_target_.reset();
            }
            clear_world_selection();
            corridor_link_source_=-1;
        }
    }

    render_view_selector(r,y,logical_w);

    std::string line1,line2;
    if(hovered_item>=0){
        const auto& item=items[(size_t)hovered_item];
        line1=item.name;
        line2="TYPE " + item.type;
        if(item.kind==BuildObjectKind::Habitat && hovered_item==build_catalog_.selected_index())
            line2 += " / SIZE " + std::string(habitat_size_name(placement_habitat_size_));
        if(item.kind==BuildObjectKind::Resource && hovered_item==build_catalog_.selected_index()){
            if(item.resource_kind==ResourceKind::RobotWorkshop)
                line2 += " / FOUR SERVICE BAYS / SUPPLY REQUIRED";
            else if(item.resource_kind==ResourceKind::StorageDome)
                line2 += " / SIZE " + std::string(storage_dome_size_name(placement_storage_size_)) +
                         " / 1-3 OPTIONAL GATES AFTER PLACEMENT";
            else
                line2 += " / SIZE " + std::string(resource_size_name(placement_resource_size_));
        }
    }else if(hovered_category>=0){
        line1=CATEGORIES[hovered_category].name;
        line2="BUILD SUBMENU";
    }
    if(!line1.empty()){
        const float tw=std::min(logical_w-16.0f,std::max(168.0f,std::max(game_text_width(line1,0.78f,theme_),game_text_width(line2,0.60f,theme_))+22.0f));
        const float tx=std::clamp((float)ui_input_.mouse_x-tw*0.5f,8.0f,logical_w-tw-8.0f);
        const Rect tip={tx,y-60.0f,tw,52.0f};
        ui::panel(r,tip,theme_);
        game_label_fitted(r,{tip.x+10,tip.y+5,tip.w-20,16},line1,0.78f,theme_);
        game_label_fitted(r,{tip.x+10,tip.y+25,tip.w-20,16},line2,0.60f,theme_,true);
    }
}

} // namespace game
