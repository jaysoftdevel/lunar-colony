#pragma once
#include "engine/ui_font.hpp"
#include "engine/ui_metrics.hpp"

namespace eng::ui {

inline constexpr int atlas_width=512,atlas_height=256;
inline constexpr int glyph_width=32,glyph_height=40;

inline const auto& font_atlas(){
    static const auto atlas=[]{
        constexpr std::uint8_t data[]{
#include "ui_font_atlas.inc"
        };
        std::array<std::uint8_t,atlas_width*atlas_height> pixels{};
        std::size_t source=0,destination=0;
        while(source<std::size(data)){
            const auto packet=data[source++];
            const std::size_t count=(packet&127u)+1;
            if(packet&128u){
                std::copy_n(data+source,count,pixels.data()+destination);
                source+=count;
            }
            destination+=count;
        }
        return pixels;
    }();
    return atlas;
}

struct Vertex {
    Vec2 position,uv;
    std::array<std::uint8_t,4> color;
};
static_assert(sizeof(Vertex)==20);

class Batch {
    using Rgba=std::array<std::uint8_t,4>;
    static auto pack(Color color){
        const auto byte=[](float value){
            return static_cast<std::uint8_t>(std::clamp(value,0.0f,1.0f)*255.0f+0.5f);
        };
        return std::array{byte(color.r),byte(color.g),byte(color.b),byte(color.a)};
    }
    void append_quad(Rect rect,Rgba a,Rgba b,Vec2 from,Vec2 to){
        const Vertex tl{{rect.x,rect.y},from,a},tr{{rect.x+rect.w,rect.y},{to.x,from.y},a};
        const Vertex bl{{rect.x,rect.y+rect.h},{from.x,to.y},b},br{{rect.x+rect.w,rect.y+rect.h},to,b};
        vertices.insert(vertices.end(),{tl,tr,br,tl,br,bl});
    }
public:
    std::vector<Vertex> vertices;

    void quad(Rect rect,Color top,Color bottom){
        if(rect.w>0 && rect.h>0) append_quad(rect,pack(top),pack(bottom),{-1,-1},{-1,-1});
    }

    void stroke(Vec2 a,Vec2 b,Color color,float width){
        if(width<=0 || color.a<=0) return;
        const auto positions=line_quad(a,b,width);
        const auto rgba=pack(color);
        for(std::size_t i=0;i<positions.size();i+=3)
            vertices.push_back({{positions[i],positions[i+1]},{-1,-1},rgba});
    }

    void text(Vec2 p,std::string_view text,float scale,Color color){
        if(scale<=0 || color.a<=0) return;
        const float start=p.x;
        const auto rgba=pack(color);
        for(unsigned char ch:text){
            if(ch=='\n'){ p.y+=kLineAdvance*scale;p.x=start;continue; }
            if(ch!=' '){
                const int index=(ch>=33 && ch<=126?ch:'?')-32;
                const float x=float(index%16*glyph_width),y=float(index/16*glyph_height);
                // Insets keep bilinear filtering inside this glyph's atlas cell.
                const Vec2 uv0{(x+.5f)/atlas_width,(y+.5f)/atlas_height};
                const Vec2 uv1{(x+glyph_width-.5f)/atlas_width,(y+glyph_height-.5f)/atlas_height};
                append_quad({p.x-.48f*scale,p.y-.34f*scale,6.4f*scale,8.0f*scale},rgba,rgba,uv0,uv1);
            }
            p.x+=kGlyphAdvance*scale;
        }
    }
};

// Legacy rendering remains available if an adapter cannot initialize the atlas pipeline.
inline std::vector<float> bitmap_text(Vec2 p,std::string_view text,float scale){
    std::vector<float> vertices;
    const float start=p.x;
    for(char ch:text){
        if(ch=='\n'){ p.y+=kLineAdvance*scale;p.x=start;continue; }
        if(ch>='a' && ch<='z') ch=static_cast<char>(ch-'a'+'A');
        const auto it=kBitmapFont.find(ch);
        if(it!=kBitmapFont.end()){
            for(int y=0;y<7;++y){
                for(int x=0;x<5;++x){
                    if(!(it->second[y]&(1<<(4-x)))) continue;
                    const float left=p.x+x*scale,top=p.y+y*scale,right=left+scale,bottom=top+scale;
                    vertices.insert(vertices.end(),{left,top,0,right,top,0,right,bottom,0,left,top,0,right,bottom,0,left,bottom,0});
                }
            }
        }
        p.x+=kGlyphAdvance*scale;
    }
    return vertices;
}

} // namespace eng::ui
