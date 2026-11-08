#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include <algorithm>
#include <string_view>

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
constexpr float category_size=42.f,category_gap=7.f,category_x=12.f;
constexpr float category_right=category_x+CATEGORY_COUNT*(category_size+category_gap)-category_gap;
}

Rect GameApp::activity_log_button_rect() const {
    const float h=viewport_height_/ui_scale_;
    if(active_game_phase_==GamePhase::Planning)return {PLAYFIELD_MARGIN,h-18.f,115.f,17.f};
    return {category_right+10.f,h-(BUILD_BAR_H+category_size)*.5f,76.f,category_size};
}

void GameApp::render_build_bar(Renderer& r){
    if(lander_.deploying()||lander_.awaiting_site||lander_.awaiting_stash){
        const float w=ui_width(r),y=ui_height(r)-BUILD_BAR_H;
        r.draw_rect({0,y,w,BUILD_BAR_H},kHudFill);
        const char* message=lander_.awaiting_site?"CLICK TOUCHDOWN SITE  /  GREEN OUTLINE = CLEAR  /  ESC MENU":
            lander_.phase<LanderPhase::Landed?"LANDER DESCENDING  /  PLACE THE STASH AFTER TOUCHDOWN":
            lander_.awaiting_stash?"TOUCHDOWN COMPLETE  /  CLICK CLEAR GROUND BESIDE THE LANDER TO PLACE THE STASH":
            "LUNAR DEPLOYMENT  /  SELECT LANDER FOR STATUS AND UNLOAD  /  ESC MENU";
        const auto log=activity_log_button_rect();
        const float x=activity_log_visible()?log.x+log.w+12.f:18.f;
        game_label_fitted(r,{x,y+16,w-x-18,22},message,0.49f,theme_,true);
        return;
    }
    const float logical_w=ui_width(r);
    const float logical_h=ui_height(r);
    const float y=logical_h-BUILD_BAR_H;
    r.draw_rect({0,y,logical_w,BUILD_BAR_H},kHudFill);
    r.draw_line({0,y},{logical_w,y},theme_.border,1.0f);

    constexpr float view_area_w=345.0f;
    const float category_y=y+(BUILD_BAR_H-category_size)*0.5f;

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

    const auto log=activity_log_button_rect();
    const float build_left=(activity_log_visible()?log.x+log.w:category_right)+28.f;
    const float build_right=logical_w-view_area_w-18.0f;
    const float build_area_w=std::max(0.0f,build_right-build_left);

    std::vector<int> visible;
    const auto& items=build_catalog_.items();
    for(size_t i=0;i<items.size();++i){
        if(items[i].category==build_catalog_.category()) visible.push_back((int)i);
    }

    constexpr float icon_size=BUILD_ICON_SIZE,icon_gap=BUILD_ICON_GAP,section_gap=18.0f;
    const int count=(int)visible.size();
    const auto group=[&](int slot){return std::string_view(items[(std::size_t)visible[(std::size_t)slot]].group);};
    const auto gap=[&](int slot){return group(slot)==group(slot-1)?icon_gap:section_gap;};
    const auto width=[&](int first,int end){
        float total=first<end?icon_size:0;
        for(int i=first+1;i<end;++i)total+=gap(i)+icon_size;
        return total;
    };
    const bool paged=width(0,count)>build_area_w;
    const float arrow_space=paged?30.0f:0.0f;
    const float available=std::max(0.0f,build_area_w-2*arrow_space);
    int maximum=std::max(0,count-1);
    while(maximum>0 && width(maximum-1,count)<=available)--maximum;
    build_catalog_scroll_=std::clamp(build_catalog_scroll_,0,paged?maximum:0);
    const bool over_catalog=Rect{build_left,y,build_area_w,BUILD_BAR_H}.contains((float)ui_input_.mouse_x,(float)ui_input_.mouse_y);
    if(paged && over_catalog && ui_input_.wheel!=0)build_catalog_scroll_=std::clamp(build_catalog_scroll_+(ui_input_.wheel<0?1:-1),0,maximum);
    if(paged){
        if(ui::button(r,ui_input_,{build_left,y+30,25,32},"<",theme_,build_catalog_scroll_>0))--build_catalog_scroll_;
        if(ui::button(r,ui_input_,{build_right-25,y+30,25,32},">",theme_,build_catalog_scroll_<maximum))++build_catalog_scroll_;
    }
    const int first=build_catalog_scroll_;
    int end=first;
    while(end<count && width(first,end+1)<=available)++end;
    float tile_x=build_left+arrow_space+std::max(0.0f,(available-width(first,end))*0.5f);
    const float icon_y=y+22.0f;

    int hovered_item=-1;
    for(int slot=first;slot<end;++slot){
        if(slot>first)tile_x+=icon_size+gap(slot);
        const int item_index=visible[(std::size_t)slot];
        const Rect box={tile_x,icon_y,icon_size,icon_size};
        if(slot==first || group(slot)!=group(slot-1)){
            int group_end=slot+1;
            while(group_end<end && group(group_end)==group(slot))++group_end;
            if(slot>first){
                const float divider=tile_x-section_gap*.5f;
                r.draw_line({divider,y+8},{divider,y+72},theme_.border,1.4f);
            }
            game_label_fitted(r,{tile_x,y+5,width(slot,group_end),13},items[(std::size_t)item_index].group,.38f,theme_,true);
        }
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
        Rect tip={tx,y-60.0f,tw,52.0f};
        if(!keep_clear_of_activity_log(tip))return;
        layout::hud_panel(r,tip,theme_);
        game_label_fitted(r,{tip.x+10,tip.y+5,tip.w-20,16},line1,0.78f,theme_);
        game_label_fitted(r,{tip.x+10,tip.y+25,tip.w-20,16},line2,0.60f,theme_,true);
    }
}

} // namespace game
