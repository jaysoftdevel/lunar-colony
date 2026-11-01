#include "engine/engine.hpp"
#include "engine/ui_metrics.hpp"

namespace eng::ui {
void panel(Renderer& r, Rect rect, const Theme& t){
    r.draw_rect(rect,t.panel);
    r.draw_line({rect.x,rect.y},{rect.x+rect.w,rect.y},t.border);
    r.draw_line({rect.x+rect.w,rect.y},{rect.x+rect.w,rect.y+rect.h},t.border);
    r.draw_line({rect.x+rect.w,rect.y+rect.h},{rect.x,rect.y+rect.h},t.border);
    r.draw_line({rect.x,rect.y+rect.h},{rect.x,rect.y},t.border);
}

void label(Renderer& r,Vec2 pos,const std::string& text,float scale,const Theme& t,bool muted){
    r.draw_text(pos,text,font_scale(scale,t),muted?t.muted:t.text);
}

void title(Renderer& r,Vec2 pos,const std::string& text,const Theme& t){
    r.draw_text(pos,text,3.0f*t.font_scale,t.text);
}

bool button(Renderer& r,const InputState& input,Rect rect,const std::string& label_text,const Theme& t,bool enabled){
    const bool hover=rect.contains((float)input.mouse_x,(float)input.mouse_y);
    const Color fill=enabled ? (hover?t.accent_hover:t.panel2) : Color{0.07f,0.075f,0.085f,1};
    r.draw_rect(rect,fill);
    const Color border=hover&&enabled?t.accent:t.border;
    r.draw_line({rect.x,rect.y},{rect.x+rect.w,rect.y},border,2);
    r.draw_line({rect.x+rect.w,rect.y},{rect.x+rect.w,rect.y+rect.h},border,2);
    r.draw_line({rect.x+rect.w,rect.y+rect.h},{rect.x,rect.y+rect.h},border,2);
    r.draw_line({rect.x,rect.y+rect.h},{rect.x,rect.y},border,2);

    const float padding_x=rect.w<60.0f?kCompactControlPaddingX:kControlPaddingX;
    const float scale=fit_text(label_text,kButtonTextScale*t.font_scale,
        {std::max(0.0f,rect.w-2*padding_x),std::max(0.0f,rect.h-2*kControlPaddingY)});
    const auto extent=text_extent(label_text,scale);
    if(scale>0)r.draw_text({rect.x+(rect.w-extent.x)*0.5f,rect.y+(rect.h-extent.y)*0.5f},label_text,scale,enabled?t.text:t.muted);
    return enabled&&hover&&input.left_pressed;
}
}
