#pragma once
#include "game/world/resource_mesh.hpp"
#include "engine/ui_metrics.hpp"
#include <array>

namespace game {
struct SupplyWarningReason { SupplyWarning flag; const char* text; };
inline std::array<SupplyWarningReason,8> supply_warning_reasons(ResourceKind kind){
    return {{{NoPower,"INSUFFICIENT POWER"},{NoWater,"NO TANK WATER"},{LowAir,"AIR BELOW 95%"},
        {LowStock,"RESERVE BELOW 10%"},{NoStorage,kind==ResourceKind::SolarPanel?"NO CONNECTED BATTERY":
            kind==ResourceKind::WaterDriller?"NO CONNECTED WATER TANK":"NO CONNECTED STORAGE"},
        {NoFood,"LOW FOOD"},{NoMedical,"LOW MEDICAL"},{NoHydrogen,"NO CONNECTED H2"}}};
}
inline std::vector<std::string> supply_warning_lines(const SupplyState& supply,float width,float font_scale,ResourceKind kind=ResourceKind::Count){
    std::string text;
    for(const auto& reason:supply_warning_reasons(kind)){
        if(!(supply.warnings&reason.flag))continue;
        if(!text.empty())text+=" / ";
        text+=reason.text;
    }
    if(text.empty())return {};
    return eng::ui::wrap_text("WARNING: "+text,width,font_scale);
}
inline float supply_warning_extra_height(const std::vector<std::string>& lines){return lines.empty()?0.0f:12.0f*(float)(lines.size()-1);}
}
