#pragma once
#include "game/world/resource_mesh.hpp"
#include "engine/ui_metrics.hpp"

namespace game {
inline std::vector<std::string> supply_warning_lines(const SupplyState& supply,float width,float font_scale,ResourceKind kind=ResourceKind::Count){
    std::string text;
    const auto add=[&](const char* reason){if(!text.empty())text+=" / ";text+=reason;};
    if(supply.warnings&NoPower)add("INSUFFICIENT POWER");
    if(supply.warnings&NoWater)add("NO TANK WATER");
    if(supply.warnings&LowAir)add("AIR BELOW 95%");
    if(supply.warnings&LowStock)add("RESERVE BELOW 10%");
    if(supply.warnings&NoStorage)add(kind==ResourceKind::SolarPanel?"NO CONNECTED BATTERY":kind==ResourceKind::WaterDriller?"NO CONNECTED WATER TANK":"NO CONNECTED STORAGE");
    if(supply.warnings&NoFood)add("LOW FOOD");
    if(supply.warnings&NoMedical)add("LOW MEDICAL");
    if(text.empty())return {};
    return eng::ui::wrap_text("WARNING: "+text,width,font_scale);
}
inline float supply_warning_extra_height(const std::vector<std::string>& lines){return lines.empty()?0.0f:12.0f*(float)(lines.size()-1);}
}
