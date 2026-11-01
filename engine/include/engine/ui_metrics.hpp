#pragma once
#include "engine/engine.hpp"
#include <array>
#include <cmath>
#include <string_view>

namespace eng::ui {
inline constexpr float kGlyphAdvance=6.0f,kGlyphHeight=7.0f,kLineAdvance=8.0f;
inline constexpr float kControlPaddingX=10.0f,kControlPaddingY=6.0f;
inline constexpr float kCompactControlPaddingX=4.0f,kButtonTextScale=1.15f;
inline Vec2 text_extent(std::string_view text,float scale){
    if(text.empty())return {};
    std::size_t longest=0,current=0,lines=1;
    for(char ch:text){if(ch=='\n'){longest=std::max(longest,current);current=0;++lines;}else ++current;}
    longest=std::max(longest,current);scale=std::max(0.0f,scale);
    return {(float)longest*kGlyphAdvance*scale,(kGlyphHeight+(float)(lines-1)*kLineAdvance)*scale};
}
inline float font_scale(float requested,const Theme& theme){return std::max(1.0f,requested*theme.font_scale);}
inline std::vector<std::string> wrap_text(const std::string& text,float width,float scale){
    const std::size_t columns=std::max<std::size_t>(1,(std::size_t)(std::max(0.0f,width)/(kGlyphAdvance*std::max(0.01f,scale))));
    std::vector<std::string> lines;std::string line;
    for(char ch:text){
        if(ch=='\n'){lines.push_back(line);line.clear();continue;}
        line+=ch;
        if(line.size()>columns){const auto space=line.find_last_of(' ',columns);
            if(space!=std::string::npos && space>0){lines.push_back(line.substr(0,space));line.erase(0,space+1);}
            else {lines.push_back(line.substr(0,columns));line.erase(0,columns);}
        }
    }
    if(!line.empty() || lines.empty())lines.push_back(line);
    return lines;
}
inline float fit_text(std::string_view text,float desired,Vec2 space){
    const auto extent=text_extent(text,1.0f);
    float scale=std::max(0.0f,desired);
    if(extent.x>0)scale=std::min(scale,std::max(0.0f,space.x)/extent.x);
    if(extent.y>0)scale=std::min(scale,std::max(0.0f,space.y)/extent.y);
    return scale;
}
inline void label_fitted(Renderer& r,Rect bounds,const std::string& text,float scale,const Theme& theme,bool muted=false){
    const float fitted=fit_text(text,font_scale(scale,theme),{bounds.w,bounds.h});
    if(fitted>0)r.draw_text({bounds.x,bounds.y},text,fitted,muted?theme.muted:theme.text);
}
// UI strokes are geometry in logical units, so thickness scales exactly like
// panels/fonts on both OpenGL and D3D12, including fractional scale factors.
inline std::array<float,18> line_quad(Vec2 a,Vec2 b,float width){
    const float dx=b.x-a.x,dy=b.y-a.y,len=std::sqrt(dx*dx+dy*dy);
    const float half=std::max(0.0f,width)*0.5f;
    const float nx=len>1e-6f?-dy*half/len:half,ny=len>1e-6f?dx*half/len:0;
    if(len<=1e-6f)b.y+=width;
    return {a.x+nx,a.y+ny,0,b.x+nx,b.y+ny,0,b.x-nx,b.y-ny,0,
            a.x+nx,a.y+ny,0,b.x-nx,b.y-ny,0,a.x-nx,a.y-ny,0};
}
// Procedural icons use one design grid, rather than mixing fixed offsets with
// a shrinking button rectangle. Aspect ratio and stroke proportions stay intact.
class IconCanvas {
    Renderer& renderer_;float scale_;Vec2 origin_;
    Vec2 point(Vec2 p) const{return {origin_.x+p.x*scale_,origin_.y+p.y*scale_};}
public:
    IconCanvas(Renderer& renderer,Rect bounds,Vec2 design):renderer_(renderer),
        scale_(std::max(0.0f,std::min(bounds.w/design.x,bounds.h/design.y))),
        origin_{bounds.x+(bounds.w-design.x*scale_)*0.5f,bounds.y+(bounds.h-design.y*scale_)*0.5f}{}
    void draw_rect(Rect box,Color color) const {const auto p=point({box.x,box.y});renderer_.draw_rect({p.x,p.y,box.w*scale_,box.h*scale_},color);}
    void draw_line(Vec2 a,Vec2 b,Color color,float width=1) const {renderer_.draw_line(point(a),point(b),color,width*scale_);}
};
}
