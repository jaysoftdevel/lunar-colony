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
    {BuildCategory::Resources,BuildIcon::CategoryResources,"RESOURCE STRUCTURES"},
    {BuildCategory::Facilities,BuildIcon::CategoryFacilities,"FACILITIES"}
};
}

void GameApp::render_build_bar(Renderer& r){
    const float logical_w=ui_width(r);
    const float logical_h=ui_height(r);
    const float y=logical_h-BUILD_BAR_H;
    r.draw_rect({0,y,logical_w,BUILD_BAR_H},{0.025f,0.035f,0.045f,0.97f});
    r.draw_line({0,y},{logical_w,y},theme_.border,1.0f);

    constexpr float view_area_w=170.0f;
    constexpr float category_size=42.0f;
    constexpr float category_gap=7.0f;
    constexpr float category_x=12.0f;
    const float category_y=y+(BUILD_BAR_H-category_size)*0.5f;
    const float category_area_w=category_size*3.0f+category_gap*2.0f+28.0f;

    int hovered_category=-1;
    for(int i=0;i<(int)(sizeof(CATEGORIES)/sizeof(CATEGORIES[0]));++i){
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
            placement_rotation_degrees_=0.0f;
            placement_habitat_size_=HabitatSizeTier::Standard;
            corridor_build_start_.reset();
            utility_build_start_.reset();
            walkway_build_start_.reset();
            corridor_link_source_=-1;
            clear_world_selection();
        }
    }

    const float build_left=category_x+category_area_w;
    const float build_right=logical_w-view_area_w-18.0f;
    const float build_area_w=std::max(260.0f,build_right-build_left);

    std::vector<int> visible;
    const auto& items=build_catalog_.items();
    for(size_t i=0;i<items.size();++i){
        if(items[i].category==build_catalog_.category()) visible.push_back((int)i);
    }

    // Keep large catalogs usable at lower resolutions / higher UI scales by
    // shrinking only the build icons when needed. The normal 50 px size is
    // retained whenever the category fits.
    float icon_size=BUILD_ICON_SIZE;
    float icon_gap=BUILD_ICON_GAP;
    if(!visible.empty()){
        const float natural=(float)visible.size()*icon_size+(float)(visible.size()-1)*icon_gap;
        if(natural>build_area_w){
            icon_gap=5.0f;
            icon_size=std::max(20.0f,(build_area_w-icon_gap*(float)(visible.size()-1))/(float)visible.size());
        }
    }
    const float total=visible.empty()?0.0f:(float)visible.size()*icon_size+(float)(visible.size()-1)*icon_gap;
    const float start_x=build_left+std::max(0.0f,(build_area_w-total)*0.5f);
    const float icon_y=y+(BUILD_BAR_H-icon_size)*0.5f;

    int hovered_item=-1;
    for(size_t slot=0;slot<visible.size();++slot){
        const int item_index=visible[slot];
        const Rect box={start_x+(icon_size+icon_gap)*(float)slot,icon_y,icon_size,icon_size};
        const bool hover=box.contains((float)ui_input_.mouse_x,(float)ui_input_.mouse_y);
        const bool selected=hovered_item==build_catalog_.selected_index();
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
                placement_habitat_size_=HabitatSizeTier::Standard;
                corridor_build_start_.reset();
                utility_build_start_.reset();
                walkway_build_start_.reset();
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
        if(item.kind==BuildObjectKind::Habitat && habitat_has_size_tiers(item.habitat_kind) && hovered_item==build_catalog_.selected_index())
            line2 += " / SIZE " + std::string(habitat_size_name(placement_habitat_size_));
    }else if(hovered_category>=0){
        line1=CATEGORIES[hovered_category].name;
        line2="BUILD SUBMENU";
    }
    if(!line1.empty()){
        const float tw=std::max(168.0f,std::max(text_width(line1,0.78f*GAME_FONT_BOOST,theme_),text_width(line2,0.60f*GAME_FONT_BOOST,theme_))+22.0f);
        const float tx=std::clamp((float)ui_input_.mouse_x-tw*0.5f,8.0f,logical_w-tw-8.0f);
        const Rect tip={tx,y-60.0f,tw,52.0f};
        ui::panel(r,tip,theme_);
        game_label(r,{tip.x+10,tip.y+5},line1,0.78f,theme_);
        game_label(r,{tip.x+10,tip.y+25},line2,0.60f,theme_,true);
    }
}

} // namespace game
